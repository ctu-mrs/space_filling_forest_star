/**
 * @file heap.h
 * @author Robert Penicka (edit by Jaroslav Janos)
 * @brief A custom implementation of the heap
 * @version 1.0
 * @date 23/04/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __HEAP_H__
#define __HEAP_H__
#include <vector>
#include <deque>
#include <set>
#include <map>
#include <algorithm>
#include <math.h>
#include "primitives.h"

#define LCHILD(x) 2 * x + 1
#define RCHILD(x) 2 * x + 2
#define PARENT(x) (x - 1) / 2

#define HEAPCOST_GET(i) getCost(i)
#define HEAPCOST_SET(i,cost) {(pathPoints)[(*heapVector)[i]].distanceFromStart = cost;}
#define HEAP_SET_ELEMENT_POSITION(element,i) {(pathPoints)[element].heapPosition = i; if (calculateCost) (pathPoints)[element].distanceFromStart = (*costFunction)((*element), (*refPoint)); }
#define HEAP_GET_ELEMENT_POSITION(element) ((pathPoints)[element].heapPosition)

template<class T, class R>
class Heap {
public:
	R *refPoint;
	
	Heap();
	Heap(std::deque<R> &data, R* goalNode, bool calculateCost = true, T (*costFunc)(R&, R&) = Distance);
	Heap(std::deque<R*> *data, R *goalNode, bool calculateCost = true, T (*costFunc)(R&, R&) = Distance);
	void push(R *newValue);
	T getCost(int index);
	void sort();
	void updateCost(int position, T cost);
	void updateCost(R *value, T cost);
	R *pop();
	R *pop(int id);
	R *get();
	R *get(int id);
	void replace(int id, R *newValue);
	int size();
	void clear();
	const bool empty();
	bool checkOrdering();
	bool checkIndex(int index);
	bool check();
	std::deque<R*>* getHeapVector();
	std::map<R*, PathNode<T>> pathPoints;
private:
	void BubbleDown(int index);
	void BubbleUp(int index);
	std::deque<R*> heapData;
	std::deque<R*> *heapVector;
	bool calculateCost;
	T (*costFunction)(R&, R&); 
};

template<class T, class R>
Heap<T, R>::Heap() : heapVector{NULL}, costFunction{Distance} {

}

template<class T, class R>
Heap<T, R>::Heap(std::deque<R> &data, R* goalNode, bool calculateCost, T (*costFunc)(R&, R&)) : heapVector{nullptr}, refPoint{goalNode}, costFunction{costFunc}, calculateCost{calculateCost} {
	for (int i{0}; i < data.size(); ++i) {
		heapData.push_back(&(data[i]));
		HEAP_SET_ELEMENT_POSITION(heapData[i], i);
		if (calculateCost) {
			(pathPoints)[heapData[i]].distanceFromStart = (*costFunction)((*heapData[i]), (*refPoint));
		}
	}
	heapVector = &heapData;
	sort();
}

template<class T, class R>
Heap<T, R>::Heap(std::deque<R*> *data, R *goalNode, bool calculateCost, T (*costFunc)(R&, R&)) : heapVector(data), refPoint{goalNode}, costFunction{costFunc}, calculateCost{calculateCost} {
  for (int var = 0; var < data->size(); ++var) {
		HEAP_SET_ELEMENT_POSITION((*heapVector)[var], var);
		if (calculateCost) {
			(pathPoints)[(*heapVector)[var]].distanceFromStart = (*costFunction)((*(*heapVector)[var]), (*refPoint));
		}
	}
	sort();
}

template<class T, class R>
T Heap<T, R>::getCost(int index) {
  auto iter{pathPoints.find((*heapVector)[index])};
	if (iter != pathPoints.end()) {
		return (pathPoints)[(*heapVector)[index]].distanceFromStart;
	} else {
		pathPoints.insert({(*heapVector)[index], PathNode<T>()});
		return std::numeric_limits<T>::max(); 
	}
}

template<class T, class R>
void Heap<T, R>::sort() {
	int size = this->heapVector->size();
	for (int var = size - 1; var >= 0; --var) {
		BubbleDown(var);
	}
}

template<class T, class R>
void Heap<T, R>::BubbleDown(int index) {
	int size = heapVector->size();

	int leftChildIndex = LCHILD(index);
	int rightChildIndex = RCHILD(index);
	if (leftChildIndex >= size) {
		return; //index is a leaf
	}
	int minIndex = index;
	if (HEAPCOST_GET(index) > HEAPCOST_GET(leftChildIndex)) {
		minIndex = leftChildIndex;
	}

	if ((rightChildIndex < size) && (HEAPCOST_GET(minIndex) > HEAPCOST_GET(rightChildIndex))) {
		minIndex = rightChildIndex;
	}

	if (minIndex != index) {
		//need swap
		std::swap((*heapVector)[index], (*heapVector)[minIndex]);
		HEAP_SET_ELEMENT_POSITION((*heapVector)[index], index);
		if (calculateCost) {
			(pathPoints)[(*heapVector)[index]].distanceFromStart = (*costFunction)((*(*heapVector)[index]), (*refPoint));
		}
		HEAP_SET_ELEMENT_POSITION((*heapVector)[minIndex], minIndex);
		if (calculateCost) {
			(pathPoints)[(*heapVector)[minIndex]].distanceFromStart = (*costFunction)((*(*heapVector)[minIndex]), (*refPoint));
		}
		BubbleDown(minIndex);
	}

}

template<class T, class R>
void Heap<T, R>::BubbleUp(int index) {
	if (index == 0)
		return;

	int parentIndex = PARENT(index);		//(index - 1) / 2;
	if (HEAPCOST_GET(parentIndex) > HEAPCOST_GET(index)) {
		std::swap((*heapVector)[parentIndex], (*heapVector)[index]);
		HEAP_SET_ELEMENT_POSITION((*heapVector)[parentIndex], parentIndex);
		if (calculateCost) {
			(pathPoints)[(*heapVector)[parentIndex]].distanceFromStart = (*costFunction)((*(*heapVector)[parentIndex]), (*refPoint));
		}
		HEAP_SET_ELEMENT_POSITION((*heapVector)[index], index);
		if (calculateCost) {
			(pathPoints)[(*heapVector)[index]].distanceFromStart = (*costFunction)((*(*heapVector)[index]), (*refPoint));
		}
		BubbleUp(parentIndex);
	}
}

template<class T, class R>
void Heap<T, R>::push(R *newValue) {
	if (!heapData.empty()) {
		heapVector = &heapData;
	} 
	int newIndex = heapVector->size();
	heapVector->push_back(newValue);
	HEAP_SET_ELEMENT_POSITION(newValue, newIndex);
	if (calculateCost) {
		(pathPoints)[newValue].distanceFromStart = (*costFunction)((*newValue), (*refPoint));
	}
	BubbleUp(newIndex);
}

template<class T, class R>
R* Heap<T, R>::pop() {
	if (!heapData.empty()) {
		heapVector = &heapData;
	} 
	R *min{NULL};
	size_t size{heapVector->size()};
	if (size > 0) {
		min = this->get();
		(*heapVector)[0] = (*heapVector)[size - 1];
		HEAP_SET_ELEMENT_POSITION((*heapVector)[0], 0);
		if (calculateCost) {
			(pathPoints)[(*heapVector)[0]].distanceFromStart = (*costFunction)((*(*heapVector)[0]), (*refPoint));
		}
		heapVector->pop_back();
		BubbleDown(0);
	} 
	return min;
}

template <class T, class R>
R* Heap<T, R>::pop(int id) {
	if (!heapData.empty()) {
		heapVector = &heapData;
	} 
	R *val{nullptr};
	size_t size{heapVector->size()};
	T oldCost{HEAPCOST_GET(id)};
	if (id == size - 1) {
		val = (*heapVector).back();
		(*heapVector).pop_back();
	} else if (size > id) {
		T newCost{HEAPCOST_GET(size - 1)};
		val = (*heapVector)[id];
		(*heapVector)[id] = (*heapVector)[size - 1];
		int position{HEAP_GET_ELEMENT_POSITION(val)};
		HEAP_SET_ELEMENT_POSITION((*heapVector)[id], position);
		if (calculateCost) {
			(pathPoints)[(*heapVector)[id]].distanceFromStart = (*costFunction)((*(*heapVector)[id]), (*refPoint));
		}
		heapVector->pop_back();
		if (newCost < oldCost) {
			BubbleUp(id);
		} else {
			BubbleDown(id);
		}
	}

	return val;
}

template<class T, class R>
R* Heap<T, R>::get() {
	if (!heapData.empty()) {
		heapVector = &heapData;
	} 
	return (*heapVector)[0];
}

template<class T, class R>
R* Heap<T, R>::get(int id) {
	if (!heapData.empty()) {
		heapVector = &heapData;
	} 
	return (*heapVector)[id];
}

template<class T, class R>
void Heap<T, R>::replace(int id, R *newValue) {
	T oldCost = HEAPCOST_GET(id);
	T newCost = newValue->getValue();
	(*heapVector)[id] = newValue;
	HEAP_SET_ELEMENT_POSITION((*heapVector)[id], id);
	if (calculateCost) {
		(pathPoints)[(*heapVector)[id]].distanceFromStart = (*costFunction)((*(*heapVector)[id]), (*refPoint));
	}
	if (newCost < oldCost) {
		//zmenseni hodnoty -- chce to jit nahoru
		BubbleUp(id);
	} else {
		BubbleDown(id);
	}
}

template<class T, class R>
int Heap<T, R>::size() {
	if (!heapData.empty()) {
		heapVector = &heapData;
	} 
	return heapVector->size();
}

template<class T, class R>
void Heap<T, R>::updateCost(int position, T cost) {
	T oldCost = HEAPCOST_GET(position);
	HEAPCOST_SET(position, cost);
	if (oldCost > cost) {
		//zmenseni hodnoty -- chce to jit nahoru
		BubbleUp(position);
	} else {
		BubbleDown(position);
	}
}

template<class T, class R>
void Heap<T, R>::updateCost(R *value, T cost) {
	int size = heapVector->size();
	updateCost((pathPoints)[value].heapPosition, cost);
}

template<class T, class R>
void Heap<T, R>::clear() {
	if (!heapData.empty()) {
		heapVector = &heapData;
	} 
	heapVector->clear();
}

template<class T, class R>
const bool Heap<T, R>::empty() {
	if (!heapData.empty()) {
		heapVector = &heapData;
	} 
	return this->heapVector->empty();
}

template<class T, class R>
std::deque<R*>* Heap<T, R>::getHeapVector() {
	return &heapData;
}

template<class T, class R>
bool Heap<T, R>::checkOrdering() {
	return checkIndex(0);
}

template<class T, class R>
bool Heap<T, R>::checkIndex(int index) {
	if (index < heapVector->size()) {
		int lchild = LCHILD(index);
		int rchild = RCHILD(index);
		if (lchild < heapVector->size()) {
			if ( HEAPCOST_GET(lchild) < HEAPCOST_GET(index)) {
				exit(1);
			}
			checkIndex(lchild);
		}
		if (rchild < heapVector->size()) {
			if ( HEAPCOST_GET(rchild) < HEAPCOST_GET(index)) {
				exit(1);
			}
			checkIndex(rchild);
		}
	}
  return false; // maybe?
}

template<class T, class R>
bool Heap<T, R>::check() {
	return heapVector == &(heapData);
}

#endif