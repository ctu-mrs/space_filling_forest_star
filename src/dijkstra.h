/**
 * @file dijkstra.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief Implementation of the Dijkstra algorithm, as proposed by Robert Penicka
 * @version 1.0
 * @date 23/04/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __DIJKSTRA_H__
#define __DIJKSTRA_H__

#include <vector>
#include <deque>
#include <set>
#include <limits>
#include <memory>
#include "primitives.h"
#include "heap.h"

#define DIJKSTRA_MAX std::numeric_limits<double>::max()

/**
 * @brief Main class for the Dijkstra algorithm
 * 
 * @tparam T Floating type
 * @tparam R Point type
 */
template <class T, class R>
class Dijkstra {
 public:
  Dijkstra(){
	}

	std::deque<DistanceHolder<T, Node<T,R>>> findPath(int start, std::vector<int> &goals, 
			std::deque<Node<T, R>> &connectedPoints);

 private:
	Node<T, R> *expandBest(std::map<Node<T, R> *, PathNode<T>> &pathPoints);
	std::unique_ptr<Heap<T, Node<T, R>>> heap;
};

/**
 * @brief Finds shortest path from the start node to each of the goals nodes
 * 
 * @tparam T Floating type
 * @tparam R Point type
 * @param start Start node
 * @param goals List of goal nodes
 * @param connectedPoints List of all nodes to be explored
 * @return std::deque<plan_with_length<T, R>> List of all paths from start
 */
template <class T, class R>
std::deque<DistanceHolder<T, Node<T,R>>> Dijkstra<T, R>::findPath(int start, std::vector<int> &goals, 
			std::deque<Node<T, R>> &connectedPoints) {
	// 1. initialize nodes and heap
	std::deque<DistanceHolder<T, Node<T,R>>> retVal;

	heap = std::make_unique<Heap<T, Node<T, R>>>(connectedPoints, &(connectedPoints[start]), false);

	heap->pathPoints[&(connectedPoints[start])].distanceFromStart = 0;
	heap->pathPoints[&(connectedPoints[start])].previousPoint = &(connectedPoints[start]);
	heap->sort();

	std::set<int> unvisitedGoals(goals.begin(), goals.end());
	
	// 2. dijkstra
	while(true) {
		Node<T, R> *best{expandBest(heap->pathPoints)};

		if (best == NULL || heap->pathPoints[best].distanceFromStart == DIJKSTRA_MAX) {
			break;
		}

		// verify goals (goals are only reward points, so...)
		if (best->IsRoot()) {
			auto goalIter{unvisitedGoals.find(best->GetId())};
			if (goalIter != unvisitedGoals.end()) {
				unvisitedGoals.erase(best->GetId());
				if (unvisitedGoals.empty()) {
					break;
				}
			}
		}
	}

	// 3. create list of plans
	for (int goal : goals) {
		DistanceHolder<T, Node<T,R>> &plan{retVal.emplace_back(&(connectedPoints[start]), &(connectedPoints[goal]))};

		Node<T, R> *actNode{&(connectedPoints[goal])};
		if (heap->pathPoints[actNode].previousPoint != NULL) {
			plan.distance = heap->pathPoints[actNode].distanceFromStart;
			plan.plan.push_front(actNode);

			Node<T, R> *startNode{&(connectedPoints[start])};
			while (actNode != startNode) {
				actNode = heap->pathPoints[actNode].previousPoint;
				plan.plan.push_front(actNode);
			}
		} else {
			//empty plan
			plan.distance = DIJKSTRA_MAX;
		}
	}

	return retVal;
}

/**
 * @brief Updates cost info for the actually best node in the heap
 * 
 * @tparam T Floating type
 * @tparam R Point type
 * @param pathPoints Map containing info about the best neigbouring nodes
 * @return Node<T, R>* The best node, which is expanded
 */
template <class T, class R>
Node<T, R>* Dijkstra<T, R>::expandBest(std::map<Node<T, R> *, PathNode<T>> &pathPoints) {
	Node<T, R> *actual{heap->pop()};

	if (actual != NULL) {
		for (auto &nodeDistPair : actual->VisibleNodes) {
			T newDist{heap->pathPoints[actual].distanceFromStart + nodeDistPair.second};

			if (newDist < heap->pathPoints[nodeDistPair.first].distanceFromStart) {
				heap->pathPoints[nodeDistPair.first].previousPoint = actual;
				heap->updateCost(nodeDistPair.first, newDist);
			}
		}
	}

	return actual;
}

#endif
