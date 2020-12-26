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
#define PATH_TO_TSP   "/home/jarajanos/Documents/obst_tsp/obst_tsp"

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
    
    void runRRT(DistanceHolder<T, Node<T, R>> *edge);
    void processResults(std::string &line, std::deque<std::tuple<int,int>> &edgePairs, T &pathLength);

    void getPaths() override;
    void smoothPaths() override;
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

  auto startingTime{std::chrono::high_resolution_clock::now()};

  FileStruct tempTsp;
  tempTsp.fileName = TEMP_TSP;
  tempTsp.type = Map;

  bool solved{false};
  int iter{0};
  while (!solved && iter != this->problem.maxIterations) {
    std::deque<std::tuple<int, int>> selectedEdges;
    
    ++iter;
    prevDist = newDist;

    // run TSP = create file, execute, read output
    this->saveTsp(tempTsp);
    std::string command{PATH_TO_TSP};
    command.append(" --gui=none --map-type=TSP_FILE --use-path-files-folder=false --use-prm=false --problem=");
    command.append(TEMP_TSP);
    system(command.c_str());

    std::ifstream resFile{TEMP_RESULT, std::ios::in};
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
      if (!edge.plan.empty()) {
        runRRT(&edge);
      } 
      newDist += edge.distance;
    }

    solved = (newDist >= prevDist - TOLERANCE && newDist <= prevDist + TOLERANCE);
  }
  auto stopTime{std::chrono::high_resolution_clock::now()};

    if (SaveRaw <= this->problem.saveOptions) {
    this->savePaths(this->problem.fileNames[SaveRaw]);
  }

  if (this->problem.smoothing) {
    smoothPaths();
  }

  if (SaveParams <= this->problem.saveOptions) {
    this->saveParams(this->problem.fileNames[SaveParams], iter, solved, stopTime - startingTime);
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
void LazyTSP<T,R>::runRRT(DistanceHolder<T, Node<T, R>> *edge) {
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
    Point<T> rndPoint, newPoint;
    Node<T,R> *newNode;
    this->rnd.randomPointInSpace(rndPoint);  // NO PRIORITY BIAS!!!

    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
    flann::Matrix<float> rndPointMat{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION}; // just one point to add
    for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
      rndPointMat[0][i] = rndPoint[i];
    }
    rrtTree->flannIndex->knnSearch(rndPointMat, indices, dists, 1, flann::SearchParams(128));
    delete[] rndPointMat.ptr();
    Node<T,R> &neighbor{rrtTree->nodes[indices[0][0]]};
    
    // get point in this direction, check for collisions
    newPoint = neighbor.Position.getStateInDistance(rndPoint, Node<T, R>::SamplingDistance);
    if (this->env.Collide(newPoint) || !this->isPathFree(neighbor.Position, newPoint)) {
      continue;    
    }

    // add to the tree
    newNode = &(rrtTree->nodes.emplace_back(newPoint, rrtTree, &neighbor, Node<T, R>::SamplingDistance, neighbor.DistanceToRoot + Node<T, R>::SamplingDistance, iter));
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
      edge->distance = goalDistance;
      
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
}

template <class T, class R>
void LazyTSP<T,R>::processResults(std::string &line, std::deque<std::tuple<int,int>> &edgePairs, T &pathLength) {
  std::string parsedPart;
  parseString(line, parsedPart, line, resultDelimiter);
  pathLength = std::stod(parsedPart);

  parseString(line, parsedPart, line, resultDelimiter);
  int prevPoint{std::stoi(parsedPart)};
  for (int i{1}; i < numRoots; ++i) {
    parseString(line, parsedPart, line, resultDelimiter);
    int actPoint{std::stoi(parsedPart)};
    edgePairs.push_back(std::tuple<int, int>(prevPoint, actPoint));
    prevPoint = actPoint;
  }
}

#endif
