/**
 * @file lazy.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief Implementation of Lazy TSP algorithm
 * @version 0.1
 * @date 2020-12-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __LAZY_H__
#define __LAZY_H__

#define TEMP_TSP      "tempTsp.tsp"
#define TEMP_RESULT   "tempTsp.result"

#include <deque>

#include "primitives.h"
#include "randGen.h"
#include "problemStruct.h"

template <class T, class R>
class LazyTSP : public Solver<T, R> {
  public:
    LazyTSP(Problem<T> &problem); 
    ~LazyTSP();

    void Solve() override;
  private:
    int numRoots;
    int numTrees;
    inline static std::string resultDelimiter = " , ";
    std::deque<Node<T, R>> rootNodes;
    std::deque<Tree<T, Node<T, R>> *> treesToDel;
    
    void runRRT(DistanceHolder<T, Node<T, R>> *edge, int &iterations);
    void processResults(std::string &line, std::deque<std::tuple<int,int>> &edgePairs, T &pathLength);

    void getPaths() override;
    void smoothPaths() override;
    void saveTsp(const FileStruct file) override;
    void savePaths(const FileStruct file, const std::deque<std::tuple<int,int>> &selectedPaths);
    void saveParams(const FileStruct file, const int iterations, const bool solved, const std::chrono::duration<double> elapsedTime, const std::deque<std::tuple<int,int>> &selectedEdges);
};

template <class T, class R>
LazyTSP<T, R>::LazyTSP(Problem<T> &problem) : Solver<T, R>(problem), numRoots{this->problem.GetNumRoots()} {
  // create adjacency matrix
  for (int i{0}; i < numRoots; ++i) {
    Node<T, R> &node{rootNodes.emplace_back(this->problem.roots[i], nullptr, nullptr, 0, 0, 0)};
    this->allNodes.push_back(&node);
  }
  for (int i{0}; i < numRoots; ++i) {
    for (int j{i + 1}; j < numRoots; ++j) {
      this->neighboringMatrix(i, j) = DistanceHolder<T, Node<T, R>>(&(rootNodes[i]), &(rootNodes[j]), rootNodes[i].Position.distance(rootNodes[j].Position));
    }
  }
}

template <class T, class R>
LazyTSP<T,R>::~LazyTSP() {
  for (auto ptr : treesToDel) {
    if (ptr != nullptr) {
      delete ptr;
    }
  }
}

template <class T, class R>
void LazyTSP<T, R>::Solve() {
  T prevDist{-1}, newDist{0};
  std::string resultLine;
  std::deque<std::tuple<int, int>> selectedEdges;

  auto startingTime{std::chrono::high_resolution_clock::now()};

  FileStruct tempTsp;
  tempTsp.fileName = TEMP_TSP;
  tempTsp.type = Map;

  bool solved{false};
  int iter{0};
  while (!solved && iter != numRoots * this->problem.maxIterations) {
    selectedEdges.clear();
    prevDist = newDist;

    // run TSP = create file, execute, read output
    std::string id{"id_" + std::to_string(this->problem.iteration) + "_"};
    FileStruct runFile{prefixFileName(tempTsp, id)};
    this->saveTsp(runFile);
    std::string command{this->problem.tspSolver};
    command.append(" --map-type=TSP_FILE --use-path-files-folder=false --use-prm=false --tsp-solver=");
    command.append(this->problem.tspType);
    command.append(" --problem=");
    command.append(runFile.fileName);
    system(command.c_str());

    std::string resultName{TEMP_RESULT};
    std::ifstream resFile{resultName.insert(0, id), std::ios::in};
    if (!resFile.good()) {
      std::cout << "Lazy TSP: result file error";
      return;
    }

    if (resFile.is_open()) {
      getline(resFile, resultLine);
    } else {
      std::cout << "Lazy TSP: result file not opened";
      return;
    }

    processResults(resultLine, selectedEdges, newDist);
    newDist = 0;

    // run RRT for selected edges, recompute new distance
    for (auto &pair : selectedEdges) {
      int first, second;
      std::tie(first, second) = pair;
      DistanceHolder<T, Node<T, R>> &edge{this->neighboringMatrix(first, second)};
      if (edge.plan.empty()) {
        runRRT(&edge, iter);
      } 
      newDist += edge.distance;
    }

    solved = (newDist >= prevDist - TOLERANCE && newDist <= prevDist + TOLERANCE);
  }
  auto stopTime{std::chrono::high_resolution_clock::now()};

  if (SaveRaw <= this->problem.saveOptions) {
    this->savePaths(this->problem.fileNames[SaveRaw], selectedEdges);
  }

  if (this->problem.smoothing) {
    smoothPaths();
  }

  if (SaveParams <= this->problem.saveOptions) {
    this->saveParams(this->problem.fileNames[SaveParams], iter, solved, stopTime - startingTime, selectedEdges);
  }

  if (SaveTSP <= this->problem.saveOptions) {
    this->saveTsp(this->problem.fileNames[SaveTSP]);
  }
}

template <class T, class R>
void LazyTSP<T,R>::getPaths() {

}

template <class T, class R>
void LazyTSP<T, R>::smoothPaths() {

}

template <class T, class R>
void LazyTSP<T,R>::runRRT(DistanceHolder<T, Node<T, R>> *edge, int &iterations) {
  Node<T,R> *goal{edge->node2};
  Tree<T, Node<T, R>> *rrtTree{new Tree<T, Node<T,R>>};
  treesToDel.push_back(rrtTree);
  
  Node<T, R> &start{rrtTree->nodes.emplace_back(edge->node1->Position, rrtTree, nullptr, 0, 0, 0)};
  rrtTree->Root = &start;
  this->allNodes.push_back(&start);
  flann::Matrix<float> rootMat{new float[1 * PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
  for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
    rootMat[0][i] = start.Position[i];
  }
  rrtTree->ptrToDel.push_back(rootMat.ptr());
  rrtTree->flannIndex = new flann::Index<D6Distance<float>>(rootMat, flann::KDTreeIndexParams(4));
  rrtTree->flannIndex->buildIndex();

  bool solved{false};
  int iter{0};
  while (iter < this->problem.maxIterations && !solved) {
    ++iter;
    Point<T> rndPoint, newPoint;
    Node<T,R> *newNode;
    this->rnd.randomPointInSpace(rndPoint, this->problem.dimension);  // NO PRIORITY BIAS!!!

    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
    flann::Matrix<float> rndPointMat{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION}; // just one point to add
    for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
      rndPointMat[0][i] = rndPoint[i];
    }
    rrtTree->flannIndex->knnSearch(rndPointMat, indices, dists, 1, flann::SearchParams(128));
    delete[] rndPointMat.ptr();
    Node<T,R> &neighbor{rrtTree->nodes[indices[0][0]]};
    Node<T,R> *nearest{&neighbor};
    
    // get point in this direction, check for collisions
    newPoint = neighbor.Position.getStateInDistance(rndPoint, Node<T, R>::SamplingDistance);
    if (this->env.Collide(newPoint) || !this->isPathFree(neighbor.Position, newPoint)) {
      continue;    
    }

    // rrt star
    if (this->optimize) {
      T bestDist{newPoint.distance(nearest->Position) + nearest->DistanceToRoot};
      double krrt{2 * M_E * log10(rrtTree->nodes.size() + 1)};
      indices.clear();
      dists.clear();

      flann::Matrix<float> newPointMat{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
      for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
        newPointMat[0][i] = newPoint[i];
      }
      rrtTree->flannIndex->knnSearch(newPointMat, indices, dists, krrt, flann::SearchParams(128));

      std::vector<int> &indRow{indices[0]};
      for (int &ind : indRow) {
        Node<T,R> &neighbor{rrtTree->nodes[ind]};
        T neighDist{newPoint.distance(neighbor.Position) + neighbor.DistanceToRoot};
        if (neighDist < bestDist - TOLERANCE && this->isPathFree(newPoint, neighbor.Position)) {
          bestDist = neighDist;
          nearest = &neighbor;
        }
      }

      newNode = &(rrtTree->nodes.emplace_back(newPoint, rrtTree, nearest, nearest->Position.distance(newPoint), bestDist, iter));
      nearest->Children.push_back(newNode);

      for (int &ind : indRow) {
        Node<T,R> &neighbor{rrtTree->nodes[ind]}; // offset goal node
        T newPointDist{neighbor.Position.distance(newPoint)};
        T proposedDist{bestDist + newPointDist};
        if (proposedDist < neighbor.DistanceToRoot - TOLERANCE && this->isPathFree(neighbor.Position, newPoint)) {
          // rewire
          std::deque<Node<T, R> *> &children{neighbor.Closest->Children};
          auto iter{find(children.begin(), children.end(), &neighbor)};
          if (iter == children.end()) {
            std::cout << "Fatal error: Node not in children\n";
            exit(1);
          }
          neighbor.Closest->Children.erase(iter);
          neighbor.Closest = newNode;
          neighbor.Root = newNode->Root;
          neighbor.ExpandedRoot = newNode->ExpandedRoot;
          neighbor.DistanceToClosest = newPointDist;
          neighbor.DistanceToRoot = proposedDist;
          newNode->Children.push_back(&neighbor);
        }
      }
    } else {
      newNode = &(rrtTree->nodes.emplace_back(newPoint, rrtTree, nearest, Node<T, R>::SamplingDistance, nearest->DistanceToRoot + Node<T, R>::SamplingDistance, iter));
      nearest->Children.push_back(newNode);
    }
    this->allNodes.push_back(newNode);

    // add to flann
    flann::Matrix<float> pointToAdd{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
    for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
      pointToAdd[0][i] = newPoint[i];
    }
    rrtTree->flannIndex->addPoints(pointToAdd);
    rrtTree->ptrToDel.push_back(pointToAdd.ptr());

    // check goal
    T goalDistance{goal->Position.distance(newNode->Position)};
    if (goalDistance < this->treeDistance) {
      solved = true;
      edge->distance = goalDistance + newNode->DistanceToRoot;
      
      // fill-in plan
      edge->plan.push_front(goal);
      Node<T, R> *nodeToPush{newNode};
      edge->plan.push_front(nodeToPush);
      while (!nodeToPush->IsRoot()) {
        nodeToPush = nodeToPush->Closest;
        edge->plan.push_front(nodeToPush);
      }
    }
  }

  if (!solved) {
    edge->distance = std::numeric_limits<T>::max();
  }

  iterations += iter;
}

template <class T, class R>
void LazyTSP<T,R>::processResults(std::string &line, std::deque<std::tuple<int,int>> &edgePairs, T &pathLength) {
  std::string parsedPart;
  parseString(line, parsedPart, line, resultDelimiter);
  pathLength = std::stod(parsedPart);

  parseString(line, parsedPart, line, resultDelimiter);
  int prevPoint{std::stoi(parsedPart)};
  for (int i{0}; i < numRoots; ++i) {
    parseString(line, parsedPart, line, resultDelimiter);
    int actPoint{std::stoi(parsedPart)};
    edgePairs.push_back(std::tuple<int, int>(prevPoint, actPoint));
    prevPoint = actPoint;
  }
}

template <class T, class R>
void LazyTSP<T, R>::saveTsp(const FileStruct file) {
  std::cout << "Saving TSP file\n";
  std::ofstream fileStream{file.fileName.c_str()};
  if (!fileStream.good()) {
    std::cout << "Cannot create file at: " << file.fileName << "\n";
    return;
  }

  if (fileStream.is_open()) {
    fileStream << "NAME: " << this->problem.id << "\n";
    fileStream << "COMMENT:\n";
    fileStream << "TYPE: TSP\n";
    fileStream << "DIMENSION: " << numRoots << "\n";
    fileStream << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    fileStream << "EDGE_WEIGHT_FORMAT : LOWER_DIAG_ROW\n";

    fileStream << "EDGE_WEIGHT_SECTION\n";
    for (int i{0}; i < numRoots; ++i) {
      for (int j{0}; j < i; ++j) {
        fileStream << this->neighboringMatrix(i, j).distance / this->problem.environment.ScaleFactor << TSP_DELIMITER;
      }
      fileStream << "0\n";
    }
  } else {
    std::cout << "Cannot open file at: " << file.fileName << "\n";
  }
}

template <class T, class R>
void LazyTSP<T, R>::savePaths(const FileStruct file, const std::deque<std::tuple<int,int>> &selectedPaths) {
  std::cout << "Saving paths\n";
  std::ofstream fileStream{file.fileName.c_str()};
  if (!fileStream.good()) {
    std::cout << "Cannot create file at: " << file.fileName << "\n";
    return;
  }

  if (fileStream.is_open()) {
    int numRoots{this->problem.GetNumRoots()};
    if (file.type == Obj) {
      fileStream << "o Paths\n";
      for (int i{0}; i < this->allNodes.size(); ++i) {
        Point<T> temp{this->allNodes[i]->Position / this->problem.environment.ScaleFactor};
        fileStream << "v" << DELIMITER_OUT;
        temp.printPosOnly(fileStream);
        fileStream << "\n";
      }
      
      for (auto &pair : selectedPaths) {
        int first, second;
        std::tie(first, second) = pair;
        DistanceHolder<T, Node<T, R>> &holder{this->neighboringMatrix(first, second)};

        std::deque<Node<T, R> *> &plan{holder.plan};
        for (int k{0}; k < plan.size() - 1; ++k) {
          fileStream << "l" << DELIMITER_OUT << plan[k]->GetId() + 1 << DELIMITER_OUT << plan[k+1]->GetId() + 1 << "\n";
        }
      }
    } else if (file.type == Map) {
      for (auto &pair : selectedPaths) {
        int first, second;
        std::tie(first, second) = pair;
        DistanceHolder<T, Node<T, R>> &holder{this->neighboringMatrix(first, second)};

        std::deque<Node<T, R> *> &plan{holder.plan};
        for (int k{0}; k < plan.size() - 1; ++k) {
          fileStream << plan[k]->Position / this->problem.environment.ScaleFactor << DELIMITER_OUT << plan[k+1]->Position / this->problem.environment.ScaleFactor << "\n";
        }
        fileStream << "\n";
    }
    } else {
      throw std::string("Not implemented");
    }

    fileStream.flush();
    fileStream.close();
  } else {
    std::cout << "Cannot open file at: " << file.fileName << "\n";
  }  
}

template<class T, class R>
void LazyTSP<T, R>::saveParams(const FileStruct file, const int iterations, const bool solved, const std::chrono::duration<double> elapsedTime, const std::deque<std::tuple<int,int>> &selectedEdges) {
  std::cout << "Saving parameters\n";
  std::ofstream fileStream{file.fileName.c_str(), std::ios_base::openmode::_S_app};
  if (!fileStream.good()) {
    std::cout << "Cannot create file at: " << file.fileName << "\n";
    return;
  }

  if (fileStream.is_open()) {
    fileStream << this->problem.id << CSV_DELIMITER;
    fileStream << this->problem.iteration << CSV_DELIMITER;
    fileStream << iterations << CSV_DELIMITER;
    fileStream << (solved ? "solved" : "unsolved") << CSV_DELIMITER;
    fileStream << "[";
    int iter{0};
    for (auto &pair : selectedEdges) {
      ++iter;
      int first, second;
      std::tie(first, second) = pair;
      fileStream << first;
      if (iter != numRoots) {
        fileStream << CSV_DELIMITER_2;
      }
    }
    fileStream << "]" << CSV_DELIMITER << "[";
    iter = 0;
    for (auto &pair : selectedEdges) {
      ++iter;
      int first, second;
      std::tie(first, second) = pair;
      fileStream << this->neighboringMatrix(first,second).distance / this->problem.environment.ScaleFactor;
      if (iter != numRoots) {
        fileStream << CSV_DELIMITER_2;
      }
    }
    fileStream << "]" << CSV_DELIMITER;
    fileStream << elapsedTime.count() << "\n";
  } else {
    std::cout << "Cannot open file at: " << file.fileName << "\n";
  }
}

#endif
