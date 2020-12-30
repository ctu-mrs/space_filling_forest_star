/**
 * @file rrt.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief 
 * @version 1.0
 * @date 02/07/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __RRT_H__
#define __RRT_H__

#include <deque>
#include <flann/flann.hpp>
#include <algorithm>
#include <chrono>

#include "primitives.h"
#include "randGen.h"
#include "environment.h"
#include "problemStruct.h"

template<class T, class R>
class RapidExpTree : public Solver<T, R> {
  public:
    RapidExpTree(Problem<T> &problem); 

    void Solve() override;
  private:
    int numRoots;
    int numTrees;
    int centralRoot;

    Node<T, R> *goalNode;
    std::deque<Tree<T, Node<T, R>> *> treeFrontier;

    void expandNode(Tree<T, Node<T,R>> *tree, bool &solved, const unsigned int iteration);
    
    void getPaths() override;
    void smoothPaths() override;
    void getConnectedTrees();
};

template<class T, class R>
RapidExpTree<T, R>::RapidExpTree(Problem<T> &problem) : Solver<T, R>(problem), numRoots{this->problem.GetNumRoots()} {
    for (int j{0}; j < this->problem.roots.size(); ++j) {
      Tree<T, Node<T, R>> &tree{this->trees.emplace_back()};
      Node<T, R> &node{tree.nodes.emplace_back(this->problem.roots[j], &tree, nullptr, 0, 0, 0)};
      tree.Root = &node;
      this->allNodes.push_back(&node);
      flann::Matrix<float> rootMat{new float[1 * PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
      for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
        rootMat[0][i] = node.Position[i];
      }
      tree.flannIndex = new flann::Index<D6Distance<float>>(rootMat, flann::KDTreeIndexParams(4));
      tree.flannIndex->buildIndex();

      tree.ptrToDel.push_back(rootMat.ptr());
      treeFrontier.push_back(&tree);
    }    
    numTrees = this->problem.roots.size() - 1;

    // add goal, which is not expanded
    if (this->problem.hasGoal) {
      Tree<T, Node<T, R>> &tree{this->trees.emplace_back()};
      flann::Matrix<float> rootMat{new float[1 * PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
      goalNode = &(tree.nodes.emplace_back(this->problem.goal, &tree, nullptr, 0, 0, 0));
      this->allNodes.push_back(goalNode);
      tree.Root = goalNode;
      for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
        rootMat[0][i] = goalNode->Position[i];
      }

      tree.flannIndex = new flann::Index<D6Distance<float>>(rootMat, flann::KDTreeIndexParams(4));
      tree.flannIndex->buildIndex();

      tree.ptrToDel.push_back(rootMat.ptr());
      treeFrontier.push_back(&tree);
    }
    
} 

template<class T, class R>
void RapidExpTree<T, R>::Solve() {
  if (SaveGoals <= this->problem.saveOptions) {
    this->saveCities(this->problem.fileNames[SaveGoals]);
  }
  auto startingTime{std::chrono::high_resolution_clock::now()};
  int iter{0};
  bool solved{false};
  while (!(solved || iter == this->problem.maxIterations)) {
    ++iter;
    Tree<T, Node<T,R>> *treeToExpand{this->treeFrontier[this->rnd.randomIntMinMax(0, numTrees)]};
    expandNode(treeToExpand, solved, iter);

    this->saveIterCheck(iter);
  }
  auto stopTime{std::chrono::high_resolution_clock::now()};
  getConnectedTrees();
  if (SaveTree <= this->problem.saveOptions) {
    this->saveTrees(this->problem.fileNames[SaveTree]);
  }

  this->getPaths();
  this->getAllPaths();
  if (SaveRaw <= this->problem.saveOptions) {
    this->savePaths(this->problem.fileNames[SaveRaw]);
  }
  if (this->problem.smoothing) {
    this->smoothPaths();
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
}

template<class T, class R>
void RapidExpTree<T, R>::expandNode(Tree<T, Node<T, R>> *treeToExpand, bool &solved, const unsigned int iteration) {
  Point<T> rndPoint, newPoint;
  if (this->usePriority && this->rnd.randomProbability() <= this->problem.priorityBias) {
    rndPoint = goalNode->Position; 
  } else {
    this->rnd.randomPointInSpace(rndPoint, this->problem.dimension);  
  }
  
  // find nearest neighbour
  std::vector<std::vector<int>> indices;
  std::vector<std::vector<float>> dists;
  flann::Matrix<float> rndPointMat{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION}; // just one point to add
  for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
    rndPointMat[0][i] = rndPoint[i];
  }
  treeToExpand->flannIndex->knnSearch(rndPointMat, indices, dists, 1, flann::SearchParams(128));
  delete[] rndPointMat.ptr();
  Node<T,R> &neighbor{treeToExpand->nodes[indices[0][0]]};
  
  // get point in this direction, check for collisions
  newPoint = neighbor.Position.getStateInDistance(rndPoint, Node<T, R>::SamplingDistance);
  if (this->env.Collide(newPoint) || !this->isPathFree(neighbor.Position, newPoint)) {
    return;    
  }

  Node <T, R> *nearest{&neighbor}, *newNode;

  // rrt star
  if (this->optimize) {
    T bestDist{newPoint.distance(nearest->Position) + nearest->DistanceToRoot};
    double krrt{2 * M_E * log10(nearest->GetNumNodes())};
    indices.clear();
    dists.clear();

    flann::Matrix<float> newPointMat{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
    for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
      newPointMat[0][i] = newPoint[i];
    }
    treeToExpand->flannIndex->knnSearch(newPointMat, indices, dists, krrt, flann::SearchParams(128));

    std::vector<int> &indRow{indices[0]};
    for (int &ind : indRow) {
      Node<T,R> &neighbor{treeToExpand->nodes[ind]};
      T neighDist{newPoint.distance(neighbor.Position) + neighbor.DistanceToRoot};
      if (neighDist < bestDist - TOLERANCE && this->isPathFree(newPoint, neighbor.Position)) {
        bestDist = neighDist;
        nearest = &neighbor;
      }
    }

    newNode = &(treeToExpand->nodes.emplace_back(newPoint, nearest->Root, nearest, nearest->Position.distance(newPoint), bestDist, iteration));
    nearest->Children.push_back(newNode);

    for (int &ind : indRow) {
      Node<T,R> &neighbor{treeToExpand->nodes[ind]}; // offset goal node
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
    newNode = &(treeToExpand->nodes.emplace_back(newPoint, nearest->Root, nearest, Node<T, R>::SamplingDistance, nearest->DistanceToRoot + Node<T, R>::SamplingDistance, iteration));
    nearest->Children.push_back(newNode);
  }
  this->allNodes.push_back(newNode);

  // add the point to flann
  flann::Matrix<float> pointToAdd{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
  flann::Matrix<float> refPoint{new float[PROBLEM_DIMENSION], 1, PROBLEM_DIMENSION};
  for (int i{0}; i < PROBLEM_DIMENSION; ++i) {
    pointToAdd[0][i] = newPoint[i];
    refPoint[0][i] = newPoint[i];
  }
  treeToExpand->flannIndex->addPoints(pointToAdd);
  treeToExpand->ptrToDel.push_back(pointToAdd.ptr());

  // check distance to other trees and rewire
  for (int i{0}; i < this->treeFrontier.size(); ++i) {
    Tree<T, Node<T, R>> *tree{this->treeFrontier[i]};
    if (tree->GetId() == treeToExpand->GetId()) {
      continue;
    }

    indices.clear();
    dists.clear();

    tree->flannIndex->knnSearch(refPoint, indices, dists, 1, flann::SearchParams(128));
    Node<T,R> &neighbor{tree->nodes[indices[0][0]]}; // just one-to-one connection
    T neighDist{neighbor.Position.distance(newPoint)};

    if (neighDist < this->treeDistance && this->isPathFree(newPoint, neighbor.Position)) {
      // create link
      treeToExpand->links.emplace_back(newNode, &neighbor);

      // transfer nodes to tree with lower index
      Tree<T, Node<T, R>> *to{treeToExpand->GetId() < neighbor.ExpandedRoot->GetId() ? treeToExpand : neighbor.ExpandedRoot};
      Tree<T, Node<T, R>> *from{treeToExpand->GetId() < neighbor.ExpandedRoot->GetId() ? neighbor.ExpandedRoot : treeToExpand};

      flann::Matrix<float> fromNodes{new float[from->nodes.size() * PROBLEM_DIMENSION], from->nodes.size(), PROBLEM_DIMENSION};
      int rewirePos{static_cast<int>(to->nodes.size())};
      for (int i{0}; i < from->nodes.size(); ++i) {
        for (int j{0}; j < 2; ++j) {
          fromNodes[i][j] = from->nodes[i].Position[j];
        }
        to->nodes.push_back(from->nodes[i]);
      }

      for (int j{rewirePos}; j < to->nodes.size(); ++j) {
        Node<T, R> &node{to->nodes[j]};
        node.ExpandedRoot = to;
        
        if (node.Closest != nullptr) {
          auto subIter{find(to->nodes.begin(), to->nodes.end(), *(node.Closest))};
          if (subIter == to->nodes.end()) {
            std::cout << "Fatal error, rewiring tree multi RRT";
            exit(1);
          }
          node.Closest = &(*subIter);
        }

        std::deque<Node<T, R> *> temp{node.Children};
        node.Children.clear();
        for(Node<T, R> *&child : temp) {
          auto subIter{find(to->nodes.begin(), to->nodes.end(), *child)};
          if (subIter == to->nodes.end()) {
            std::cout << "Fatal error, rewiring tree multi RRT";
            exit(1);
          }
          node.Children.push_back(&(*subIter));
        }
      }

      std::deque<DistanceHolder<T, Node<T,R>>> temp{to->links};
      to->links.clear();
      for (DistanceHolder<T, Node<T, R>> &link : temp) {
        auto iter{find(to->nodes.begin(), to->nodes.end(), *link.node1)};
        if (iter == to->nodes.end()) {
          std::cout << "Fatal error, rewiring tree multi RRT";
          exit(1);
        }

        auto iter2{find(to->nodes.begin(), to->nodes.end(), *link.node2)};
        to->links.emplace_back(&(*iter), &(*iter2));
      }

      for(DistanceHolder<T, Node<T,R>> &link : from->links) {
        auto iter{find(to->nodes.begin(), to->nodes.end(), *link.node1)};
        if (iter == to->nodes.end()) {
          std::cout << "Fatal error, rewiring tree multi RRT";
          exit(1);
        }

        auto iter2{find(to->nodes.begin(), to->nodes.end(), *link.node2)};
        to->links.emplace_back(&(*iter), &(*iter2));
      }

      to->flannIndex->addPoints(fromNodes);
      to->ptrToDel.push_back(fromNodes.ptr());

      // delete tree 
      to->eaten.push_back(from);
      for (auto &tree : from->eaten) {
        to->eaten.push_back(tree);
      }

      auto iter{find(this->treeFrontier.begin(), this->treeFrontier.end(), from)};
      if (iter == this->treeFrontier.end()) {
        std::cout << "Fatal error during tree merging (RRT)";
        exit(1);
      }
      this->treeFrontier.erase(iter);
      treeToExpand = to;
      solved = (this->treeFrontier.size() == 1);
      --numTrees;
      --i;
    }

  }

  delete[] refPoint.ptr();
}

template<class T, class R>
void RapidExpTree<T,R>::getPaths() {
  for (DistanceHolder<T, Node<T, R>> &link : this->trees[centralRoot].links) {
    if (link.distance > 1e100) {
      std::cout << "Fatal error: max distance reached";
      exit(1);
    } 
    std::deque<Node<T, R> *> &plan{link.plan};

    // one tree
    Node<T, R> *nodeToPush{link.node1};
    plan.push_front(nodeToPush);
    while (!nodeToPush->IsRoot()) {
      nodeToPush = nodeToPush->Closest;
      plan.push_front(nodeToPush);
    }

    // second tree
    nodeToPush = link.node2;
    plan.push_back(nodeToPush);
    while(!nodeToPush->IsRoot()) {
      nodeToPush = nodeToPush->Closest;
      plan.push_back(nodeToPush);
    }

    this->neighboringMatrix(link.node1->Root->GetId(), link.node2->Root->GetId()) = link;
  }
}

template<class T, class R>
void RapidExpTree<T,R>::smoothPaths() {
  for (DistanceHolder<T, Node<T, R>> &link : this->trees[this->centralRoot].links) {
    std::deque<Node<T, R> *> &plan{link.plan};

    auto tempGoal{plan.rbegin()};
    
    while (tempGoal < plan.rend() - 1) {
      auto prevNode{plan.rend() - 1};
      auto testNode{plan.rend() - 1};

      bool changed{false};
      while (testNode > tempGoal + 1) {
        if (this->isPathFree((*testNode)->Position, (*tempGoal)->Position)) {
          changed = true;
          break;
        }
        prevNode = testNode--;
      }

      if (changed) {
        plan.erase(std::next(testNode - 1).base(), std::next(tempGoal).base());
      }
      tempGoal = testNode;
    }
  }
}

template <class T, class R>
void RapidExpTree<T, R>::getConnectedTrees() {
  int maxConn{0};
  int numRoots{this->problem.hasGoal ? numTrees + 2 : numTrees + 1};
  for (int i{0}; i < numRoots; ++i) {   //+1 because it is rather the maximum index
    if (this->trees[i].eaten.size() > maxConn) {
      maxConn = this->trees[i].eaten.size();
      this->centralRoot = i;
      this->connectedTrees = this->trees[i].eaten;
      this->connectedTrees.push_back(&this->trees[i]);
    }
  }
}

#endif
