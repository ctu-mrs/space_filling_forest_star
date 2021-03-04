/**
 * @file forest.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief 
 * @version 1.0
 * @date 24/06/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __FOREST_H__
#define __FOREST_H__

#include <deque>
#include <map>
#include <flann/flann.hpp>
#include <chrono>

#include "primitives.h"
#include "randGen.h"
#include "environment.h"
#include "problemStruct.h"
#include "heap.h"

#define SQR(A)              ((A) * (A))
#define CLOSED_OBST_MULT    1

using namespace std;

template<class T, class R=Point<T>>
class SpaceForest : public Solver<T, R> {
  public:
    SpaceForest(Problem<T> &problem);

    void Solve() override;
  private:
    int numRoots;

    std::deque<Node<T, R>*> frontier;
    std::deque<Node<T, R>*> closed_list;
    Node<T, R> *goalNode{nullptr};
    Node<T, R> *nodeToExpand{nullptr};
    SymmetricMatrix<std::deque<DistanceHolder<T, Node<T, R>>>> borders;

    bool expandNode(Node<T, R> *expanded, bool &solved, const unsigned int iteration);
    int maxConnected();

    void getPaths() override;
    void smoothPaths() override;

    void saveFrontiers(const FileStruct file);
    void saveIterCheck(const int iter) override;
};

template <class T, class R>
SpaceForest<T, R>::SpaceForest(Problem<T> &problem) : Solver<T,R>(problem), 
  numRoots{this->problem.GetNumRoots()}, borders{numRoots} {
    flann::Matrix<float> globMat{new float[this->problem.roots.size() * PROBLEM_DIMENSION], this->problem.roots.size(), PROBLEM_DIMENSION};
    for (int j{0}; j < this->problem.roots.size(); ++j) {
      Tree<T, Node<T, R>> &tree{this->trees.emplace_back()};
      Node<T, R> &node{tree.nodes.emplace_back(this->problem.roots[j], &tree, nullptr, 0, 0, 0)};
      this->allNodes.push_back(&node);
      tree.Root = &node;
      flann::Matrix<float> rootMat{new float[1 * PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
      frontier.push_back(&node);
      
      for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
        rootMat[0][i] = node.Position[i];
        globMat[j][i] = node.Position[i];
      }
      tree.flannIndex = new flann::Index<D6Distance<float>>(rootMat, flann::KDTreeIndexParams(4));
      tree.flannIndex->buildIndex();
      tree.ptrToDel.push_back(rootMat.ptr());
    }
    delete[] globMat.ptr();

    if (this->usePriority && !this->problem.hasGoal) {
      for (int i{0}; i < this->trees.size(); ++i) {
        Tree<T, Node<T, R>> &tree{this->trees[i]};
        for (int j{0}; j < this->trees.size(); ++j) {
          if (i == j) {
            continue;
          }
          tree.AddFrontier(this->trees[j].Root);
        }
      }
    }

    // add goal - it is not expanded
    if (this->problem.hasGoal) {
      Tree<T, Node<T, R>> &tree{this->trees.emplace_back()};
      Node<T, R> &node{tree.nodes.emplace_back(this->problem.goal, &tree, nullptr, 0, 0, 0)};
      this->allNodes.push_back(&node);
      flann::Matrix<float> rootMat{new float[1 * PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
      for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
        rootMat[0][i] = this->problem.goal[i];
      }
      tree.flannIndex = new flann::Index<D6Distance<float>>(rootMat, flann::KDTreeIndexParams(4));
      tree.flannIndex->buildIndex();
      tree.ptrToDel.emplace_back(rootMat.ptr());
      goalNode = &node;

      if (this->usePriority) {  // must be done after goalNode assignment
        for (int i{0}; i < this->trees.size() - 1; ++i) {
          this->trees[i].AddFrontier(goalNode);
        }
      }
    }
}

template<class T, class R>
void SpaceForest<T, R>::Solve() {
  if (SaveGoals <= this->problem.saveOptions) {
    this->saveCities(this->problem.fileNames[SaveGoals]);
  }
  auto startingTime{std::chrono::high_resolution_clock::now()};

  int iter{0};
  bool solved{false};
  bool emptyFrontier{false};
  while (!(solved || iter >= this->problem.maxIterations)) {
    Tree<T, Node<T, R>> *rndTree{nullptr};
    Heap<T, Node<T, R>> *prior{nullptr};
    int priorPos{-1};
    if (this->usePriority && !emptyFrontier) {
      while (rndTree == nullptr || rndTree->EmptyFrontiers()) {
        rndTree = &(this->trees[this->rnd.randomIntMinMax(0, this->trees.size() - 1)]);
      }
      while(priorPos == -1 || rndTree->frontiers[priorPos].empty()) {
        priorPos = this->rnd.randomIntMinMax(0, rndTree->frontiers.size() - 1);
      }
      prior = &(rndTree->frontiers[priorPos]);
    }
    
    bool fromClosed{false};
    int pos;
    if (!closed_list.empty() && (emptyFrontier)) {
      pos = this->rnd.randomIntMinMax(0, closed_list.size() - 1);
      nodeToExpand = closed_list[pos];
      fromClosed = true;
    } else {
      if (this->usePriority && this->rnd.randomProbability() <= this->problem.priorityBias) {
        nodeToExpand = prior->pop();
      } else if (this->usePriority) {
        pos = this->rnd.randomIntMinMax(0, prior->size() - 1);
        nodeToExpand = prior->pop(pos);
      } else {
        pos = this->rnd.randomIntMinMax(0, frontier.size() - 1);
        nodeToExpand = frontier[pos];
      }
    }

    bool expandResult{true};
    for (int i{0}; i < Node<T,R>::ThresholdMisses && expandResult && iter < this->problem.maxIterations; ++i) {
      ++iter;
      expandResult &= expandNode(nodeToExpand, solved, iter);
      saveIterCheck(iter);
    }
    if (expandResult && !fromClosed) {
      if (!this->usePriority) {
        auto iter{frontier.begin() + pos};
        frontier.erase(iter);
      } else {
        for (int i{(int)(rndTree->frontiers.size()) - 1}; i > -1; --i) {
          Heap<T, Node<T, R>> &p{rndTree->frontiers[i]};
          if (&p == prior) {
            continue;
          }
          for (int j{p.size() - 1}; j > -1; --j) {
            if (p.get(j) == nodeToExpand) {
              p.pop(j);
            }
          }
        }
      }
      nodeToExpand->ForceChildren = true;
      closed_list.push_back(nodeToExpand);
    } else if (this->usePriority && !fromClosed) {
      prior->push(nodeToExpand);
    }

    // check, whether all points are connected of frontiers are empty
    if (!solved && this->usePriority) {
      emptyFrontier = true;
      for (Tree<T, Node<T, R>> &tree : this->trees) {
        emptyFrontier &= tree.EmptyFrontiers();
        if (!emptyFrontier) {
          break;
        }
      }
    } else {
      emptyFrontier = frontier.empty();
    }

    if (!solved) {
      bool connected{maxConnected() == numRoots};
      solved = (!this->problem.hasGoal && emptyFrontier && connected);
    } else {
      maxConnected(); // update the connected trees to correct params output
    }
  }
  auto stopTime{std::chrono::high_resolution_clock::now()};
  if (!solved && !this->problem.hasGoal) {
    solved = maxConnected() == numRoots;
  }

  if (SaveTree <= this->problem.saveOptions) {
    this->saveTrees(this->problem.fileNames[SaveTree]);
  }
  
  getPaths();
  this->getAllPaths();
  if (SaveRaw <= this->problem.saveOptions) {
    this->savePaths(this->problem.fileNames[SaveRaw]);
  }

  if (this->problem.smoothing) {
    smoothPaths();
    if (SaveSmooth <= this->problem.saveOptions) {
      this->savePaths(this->problem.fileNames[SaveSmooth]);
    }
  }

  if (SaveParams <= this->problem.saveOptions) {
    this->saveParams(this->problem.fileNames[SaveParams], iter, solved, stopTime - startingTime);
  }

  if (SaveTSP <= this->problem.saveOptions) {
    this->saveTsp(this->problem.fileNames[SaveTSP]);
  }

  if (SaveFrontiers <= this->problem.saveOptions) {
    this->saveFrontiers(this->problem.fileNames[SaveFrontiers]);
  }
}

// false means success
template<class T, class R>
bool SpaceForest<T, R>::expandNode(Node<T, R> *expanded, bool &solved, const unsigned int iteration) {
  Point<T> newPoint;

  bool result;
  result = this->rnd.randomPointInDistance(expanded->Position, newPoint, Node<T,R>::SamplingDistance, this->problem.dimension);

  if (!result || this->env.Collide(newPoint) || !this->isPathFree(expanded->Position, newPoint)) {     // check limits and collisions
    return true;
  }
  
  T parentDistance{expanded->Position.distance(newPoint)};
  Tree<T, Node<T, R>> *expandedTree{expanded->Root};
  int expandedRootID{expandedTree->GetId()};

  //find nearest neighbors - perform radius search
  std::vector<std::vector<int>> indices;
  std::vector<std::vector<float>> dists;
  flann::Matrix<float> pointToAdd{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION}; // just one point to add
  for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
    pointToAdd[0][i] = newPoint[i];
  }
  T checkDist{this->treeDistance +  2 * Node<T,R>::SamplingDistance}; //expanded->GetSampDist()};
  for (int j{0}; j < this->trees.size(); ++j) {
    Tree<T, Node<T,R>> &tree{this->trees[j]};
    int neighbourRootID{tree.GetId()}; 

    int neighbours{tree.flannIndex->radiusSearch(pointToAdd, indices, dists, SQR(checkDist), 
      flann::SearchParams(128))};
    
    std::vector<int> &indRow{indices[0]}; // just one point
    for (int i{0}; i < neighbours; ++i) {
      int neighID{indRow[i]};
      Node<T, R> *neighbour{&(tree.nodes[neighID])};
      
      T realDist{neighbour->Position.distance(newPoint)};
      
      if (!expanded->ForceChildren && realDist < (parentDistance - TOLERANCE) && neighbourRootID == expandedRootID && this->isPathFree(neighbour->Position, newPoint)) {
        // closer node available in the same tree
        delete[] pointToAdd.ptr();
        return true; // just overcrowded area
      }

      // maybe also only on not ForceChildren ???
      if (neighbourRootID != expandedRootID && realDist < (this->treeDistance - TOLERANCE)) {   // if realDist is bigger than tree distance, it might get expanded later
        // neighbouring trees, add to neighboring matrix and below dTree distance -> invalid point, but save expanded
        // check whether the goal was achieved
        if (this->problem.hasGoal && neighbour->Position == this->problem.goal) {
          solved = this->isPathFree(newPoint, this->problem.goal);  // when true, break the solve loop and after the creation of the new node, add it to the neighbouring matrix
        } else if (!this->problem.hasGoal && this->isPathFree(expanded->Position, neighbour->Position)) {   
          std::deque<DistanceHolder<T, Node<T, R>>> &borderPoints{this->borders(neighbourRootID, expandedRootID)};
          DistanceHolder<T, Node<T, R>> holder{neighbour, expanded};
          if (find(borderPoints.begin(), borderPoints.end(), holder) == borderPoints.end()) {
            this->borders(neighbourRootID, expandedRootID).push_back(holder);
          }
        }

        if (!solved) {  // if solved, the new node lies near the goal and should be created
          delete[] pointToAdd.ptr();
          return true;
        }
      }
    }
  }

  Node<T, R> *newNode; 

  // sff star
  if (this->optimize) {
    T bestDist{newPoint.distance(expanded->Position) + expanded->DistanceToRoot};
    double ksff{2 * M_E * log10(expanded->GetNumNodes())};
    indices.clear();
    dists.clear();

    flann::Matrix<float> newPointMat{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
    for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
      newPointMat[0][i] = newPoint[i];
    }
    expandedTree->flannIndex->knnSearch(newPointMat, indices, dists, ksff, flann::SearchParams(128));

    std::vector<int> &indRow{indices[0]};
    for (int &ind : indRow) {
      Node<T, R> &neighbor{expandedTree->nodes[ind]};
      T neighborDist{newPoint.distance(neighbor.Position) + neighbor.DistanceToRoot};
      if (neighborDist < bestDist - TOLERANCE && this->isPathFree(newPoint, neighbor.Position)) {
        bestDist = neighborDist;
        expanded = &neighbor;
      }
    }

    newNode = &(expandedTree->nodes.emplace_back(newPoint, expandedTree, expanded, newPoint.distance(expanded->Position), bestDist, iteration));
    expanded->Children.push_back(newNode);

    for (int &ind : indRow) {
      Node<T, R> &neighbor{expandedTree->nodes[ind]};
      T newPointDist{neighbor.Position.distance(newPoint)};
      T proposedDist{bestDist + newPointDist};
      if (proposedDist < neighbor.DistanceToRoot - TOLERANCE && this->isPathFree(neighbor.Position, newPoint)) {
        // rewire
        std::deque<Node<T, R> *> &children{neighbor.Closest->Children};
        auto iter{find(children.begin(), children.end(), &neighbor)};
        if (iter == children.end()) {
          std::cout << "Fatal error when rewiring SFF: Node not in children\n";
          exit(1);
        }
        neighbor.Closest->Children.erase(iter);
        neighbor.Closest = newNode;
        neighbor.DistanceToClosest = newPointDist;
        neighbor.DistanceToRoot = proposedDist;
        newNode->Children.push_back(&neighbor);
      }
    }
    delete[] newPointMat.ptr();
  } else {
    newNode = &(expandedTree->nodes.emplace_back(newPoint, expandedTree, expanded, parentDistance, 
      parentDistance + expanded->DistanceToRoot, iteration));
    expanded->Children.push_back(newNode);
  }
  this->allNodes.push_back(newNode);
  
  // finally add the new node to flann and frontiers
  if (this->usePriority) {
    for (Heap<T, Node<T, R>> &prior : expandedTree->frontiers) {
      prior.push(newNode); 
    }
  } else {
    frontier.push_back(newNode);
  }
  expandedTree->flannIndex->addPoints(pointToAdd);

  if (solved) {
    T distance{newPoint.distance(this->problem.goal)};
    this->borders(numRoots - 1, expandedRootID).emplace_back(newNode, goalNode, newNode->DistanceToRoot + distance);
  }
  
  expandedTree->ptrToDel.push_back(pointToAdd.ptr());
  return false;
}

template<class T, class R>
int SpaceForest<T, R>::maxConnected() {
  int maxConn{0};
  int remaining{numRoots};
  bool connMat[numRoots];
  for (int i{1}; i < numRoots; ++i) {
    connMat[i] = false;
  }
  int unconnected{0};
  while (maxConn < remaining) {
    this->connectedTrees.clear();
    std::deque<int> stack{unconnected};
    connMat[unconnected] = true;

    while(!stack.empty()) {
      int root{stack.front()};
      stack.pop_front();
      this->connectedTrees.push_back(&(this->trees[root]));
      for (int i{0}; i < numRoots; ++i) {
        if (root == i) {
          continue;
        }

        if (!this->borders(root, i).empty() && !connMat[i]) {
          connMat[i] = true;
          stack.push_front(i);
        }
      }
    }
    maxConn = this->connectedTrees.size();
    for (int i{0}; i < numRoots; ++i) {
      if (!connMat[i]) {
        unconnected = i;
        break;
      }
    }
    remaining -= maxConn;
  }

  return maxConn;
}

template<class T, class R>
void SpaceForest<T,R>::getPaths() {
  // go through borders, select one with the shortest distance, then update neighboring matrix and create paths
  for (int i{0}; i < numRoots; ++i) {
    for (int j{i + 1}; j < numRoots; ++j) {
      if (this->borders(i, j).empty()) {
        continue;
      }

      std::deque<DistanceHolder<T, Node<T, R>>> &borderPoints{this->borders(i, j)};
      T bestDist{-1};
      for (DistanceHolder<T, Node<T, R>> &dist : borderPoints) {
        dist.UpdateDistance();
        if (bestDist == -1 || dist.distance < bestDist - TOLERANCE) {
          bestDist = dist.distance;
          this->neighboringMatrix(i, j) = dist;
        }
      }

      DistanceHolder<T, Node<T, R>> &holder{this->neighboringMatrix(i, j)};
      std::deque<Node<T, R> *> &plan{holder.plan};
      // one tree
      Node<T, R> *nodeToPush{holder.node1};
      plan.push_front(nodeToPush);
      while (!nodeToPush->IsRoot()) {
        nodeToPush = nodeToPush->Closest;
        plan.push_front(nodeToPush);
      }

      // second tree
      nodeToPush = holder.node2;
      plan.push_back(nodeToPush);
      while(!nodeToPush->IsRoot()) {
        nodeToPush = nodeToPush->Closest;
        plan.push_back(nodeToPush);
      }

      if (this->optimize) {
        // re-compute the distance, which may be different
        holder.distance = holder.node1->DistanceToRoot + holder.node2->DistanceToRoot + holder.node1->Position.distance(holder.node2->Position);
      }
    }
  }
}

template<class T, class R> 
void SpaceForest<T, R>::smoothPaths() {
  for (int i{0}; i < numRoots; ++i) {
    for (int j{i + 1}; j < numRoots; ++j) {
      if (this->neighboringMatrix(i, j).node1 == NULL) {
        continue;
      }

      DistanceHolder<T, Node<T, R>> &holder{this->neighboringMatrix(i, j)};
      std::deque<Node<T, R> *> &plan{holder.plan};

      auto tempGoal{plan.rbegin()};
      T prevDist{holder.distance};
      
      while (tempGoal < plan.rend() - 1) {
        auto prevNode{plan.rend() - 1};
        auto testNode{plan.rend() - 1};
        T cumDist{0};

        bool changed{false};
        while (testNode > tempGoal + 1) {
          if (prevNode != testNode) {
            cumDist += (*testNode)->Position.distance((*prevNode)->Position);
          }

          if (this->isPathFree((*testNode)->Position, (*tempGoal)->Position)) {
            changed = true;
            break;
          }
          prevNode = testNode--;
        }

        if (testNode == tempGoal + 1) {
          cumDist += (*testNode)->Position.distance((*prevNode)->Position);
        }

        if (changed) {
          T difDist{prevDist - cumDist - (*testNode)->Position.distance((*tempGoal)->Position)};
          holder.distance -= difDist;
          plan.erase(std::next(testNode - 1).base(), std::next(tempGoal).base());
        }
        prevDist = cumDist;
        tempGoal = testNode;
      }
    }
  }
}

template<class T, class R>
void SpaceForest<T, R>::saveFrontiers(const FileStruct file) {
  std::cout << "Saving frontiers\n";
  std::ofstream fileStream{file.fileName.c_str()};
  if (!fileStream.good()) {
    std::cout << "Cannot create file at: " << file.fileName << "\n";
    return;
  }

  if (fileStream.is_open()) {
    int numRoots{(int)this->connectedTrees.size()};
    if (file.type == Obj) {
      fileStream << "o Open nodes\n";
      if (this->usePriority) {
        for (int i{0}; i < this->trees.size(); ++i) {
          for (auto heap : this->trees[i].frontiers) {
            for (Node<T, R> *node : *(heap.getHeapVector())) {
              Point<T> temp{node->Position / this->problem.environment.ScaleFactor};
              fileStream << "v" << DELIMITER_OUT;
              temp.printPosOnly(fileStream);
              fileStream << "\n";
            }
            break;  // all nodes are in all heaps, so printing the first heap is sufficient
          }
        }
      } else {
        for (Node <T, R> *node : this->frontier) {
          fileStream << "v" << DELIMITER_OUT << node->Position / this->problem.environment.ScaleFactor << "\n";
        }
      }
    } else if (file.type == Map) {
      if (this->usePriority) {
        for (int i{0}; i < this->trees.size(); ++i) {
          for (auto heap : this->trees[i].frontiers) {
            for (Node<T, R> *node : *(heap.getHeapVector())) {
              fileStream << node->Position / this->problem.environment.ScaleFactor << DELIMITER_OUT << "1\n"; // the one in the end is just for plotting purposes
            }

            break;  // all nodes are in all heaps, so printing out the first heap is sufficient
          }
        }
      } else {
        for (Node<T, R> *node : this->frontier) {
          fileStream << node->Position / this->problem.environment.ScaleFactor << DELIMITER_OUT << "1\n";
        }
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
void SpaceForest<T, R>::saveIterCheck(const int iter) {
  Solver<T, R>::saveIterCheck(iter);

  if (this->problem.saveFrontiersIter != 0 && !(iter % this->problem.saveFrontiersIter)) {
    std::string prefix{"iter_" + to_string(iter) + "_"};
    this->saveFrontiers(prefixFileName(this->problem.fileNames[SaveFrontiers], prefix));
  }
}

#endif
