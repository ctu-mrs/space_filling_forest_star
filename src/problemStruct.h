/**
 * @file problemStruct.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief 
 * @version 1.0
 * @date 01/07/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __PROBLEM_STRUCT_H__
#define __PROBLEM_STRUCT_H__

#include <string>
#include <deque>
#include <map>
#include <chrono>

#include "primitives.h"
#include "randGen.h"
#include "environment.h"

enum SaveOptions {
  None = 0,
  SaveConcurrent = 1,
  SaveGoals = 2,
  SaveTree = 4,
  SaveRaw = 8,
  SaveSmooth = 16,
  SaveParams = 32,
  SaveTSP = 64,
  SaveFrontiers = 128,
  Invalid = 256
};

enum SolverType{
  SFF,
  RRT,
  Lazy
};

template<class T>
struct Problem {
  int iteration{0};
  Dimensions dimension{D3};

  SolverType solver;
  bool optimal;
  bool smoothing;

  Environment<T> environment;
  std::deque<Point<T>> roots;
  Point<T> goal;
  bool hasGoal{false};
  
  bool autoRange{false};

  T distTree;
  T collisionDist;
  
  int maxIterations;
  T priorityBias{0};
  int saveTreeIter{0};
  int saveFrontiersIter{0};
  
  SaveOptions saveOptions{None};
  std::map<SaveOptions, FileStruct> fileNames;
  std::string id{ "Solver" };

  std::string tspSolver;
  std::string tspType;

  int GetNumRoots() {
    if (hasGoal) {
      return roots.size() + 1;
    } else {
      return roots.size();
    }
  }
};

/**
 * @brief Addition of a flag to save options
 * 
 * @param a Current save options
 * @param b Flag to be added
 * @return SaveOptions New save options 
 */
inline SaveOptions operator|(SaveOptions a, SaveOptions b) {
  return static_cast<SaveOptions>(static_cast<int>(a) | static_cast<int>(b));
}

/**
 * @brief Returns if a flag in save options is active
 * 
 * @param a Flag to be determined
 * @param b Current save options
 * @return true Flag is active
 * @return false Otherwise
 */
inline bool operator<=(SaveOptions a, SaveOptions b) {
  return  (static_cast<int>(b) & static_cast<int>(a)) == static_cast<int>(a);
}

template<class T, class R=Point<T>>
class Solver {
  public:
    Solver(Problem<T> &problem);

    virtual ~Solver() {
    }

    virtual void Solve() = 0;
  protected:
    Problem<T> &problem;
    RandGen<T> rnd;
    bool optimize;
    bool usePriority;
    T treeDistance;
    T collisionSampleSize{0.1};

    Environment<T> &env;

    std::deque<Node<T, R> *> allNodes;
    std::deque<Tree<T, Node<T, R>>> trees;
    SymmetricMatrix<DistanceHolder<T, Node<T, R>>> neighboringMatrix;
    std::deque<Tree<T, Node<T, R>> *> connectedTrees;

    bool isPathFree(Point<T> &start, Point<T> &finish);

    virtual void getPaths() = 0;
    virtual void getAllPaths();
    virtual void smoothPaths() = 0;
    void checkPlan();
    void checkPlan(std::deque<Node<T, R> *> &plan);
    void checkDistances(std::deque<Node<T, R> *> &plan, T distanceToCheck);
    T computeDistance(std::deque<Node<T, R> *> &plan);

    virtual void saveIterCheck(const int iter);
    virtual void saveTrees(const FileStruct file);
    virtual void saveCities(const FileStruct file);
    virtual void savePaths(const FileStruct file);
    virtual void saveParams(const FileStruct file, const int iterations, const bool solved, const std::chrono::duration<double> elapsedTime);
    virtual void saveTsp(const FileStruct file);
};

template<class T, class R>
Solver<T, R>::Solver(Problem<T> &problem) : problem{problem}, rnd{problem.environment.limits}, neighboringMatrix{problem.GetNumRoots()},
  treeDistance{problem.distTree}, env{problem.environment}, optimize{problem.optimal}, usePriority{problem.priorityBias != 0} {
  }

template<class T, class R>
bool Solver<T, R>::isPathFree(Point<T> &start, Point<T> &finish) {
  T totalDistance{start.distance(finish)};
  T parts{totalDistance / this->collisionSampleSize};
  Point<T> position;
  Vector<T> direction{start, finish};
  bool isFree{true};
  for (unsigned int index{1}; index < parts && isFree; ++index) {
    for (int i{0}; i < 3; ++i) {
      position.setPosition(i, start[i] + index * direction[i] / parts); // create point on path between nodes (line specified by deque direction)
    }

    isFree &= !env.Collide(position);
  }
  return isFree;
}

template <class T, class R>
T Solver<T, R>::computeDistance(std::deque<Node<T, R> *> &plan) {
  Node<T, R> *previous{nullptr};
  T distance{0};
  for (Node<T, R> *node : plan) {
    if (previous != nullptr) {
      distance += previous->Position.distance(node->Position);
    }
    previous = node;
  }
  return distance;
}

template<class T, class R>
void Solver<T, R>::getAllPaths() {
  int numRoots{this->problem.GetNumRoots()};
  for (int k{0}; k < this->connectedTrees.size(); ++k) {
    int id3{connectedTrees[k]->GetId()};
    for (int i{0}; i < this->connectedTrees.size(); ++i) {
      int id1{connectedTrees[i]->GetId()};
      if (i == k || !this->neighboringMatrix.Exists(id1, id3)) {
        continue;
      }
      DistanceHolder<T, Node<T, R>> &holder1{this->neighboringMatrix(id1, id3)};
      for (int j{0}; j < this->connectedTrees.size(); ++j) {
        int id2{connectedTrees[j]->GetId()};
        if (i == j || !this->neighboringMatrix.Exists(id2, id3)) {
          continue;
        }

        DistanceHolder<T, Node<T, R>> &link{this->neighboringMatrix(id1, id2)};
        DistanceHolder<T, Node<T, R>> &holder2{this->neighboringMatrix(id2, id3)};

        Node<T, R> *node1;
        Node<T, R> *node2;
        std::deque<Node<T, R> *> plan1;
        std::deque<Node<T, R> *> plan2;
        std::deque<Node<T, R> *> finalPlan;

        bool reversed1{false};
        bool reversed2{false};
        plan1 = holder1.plan;
        if (holder1.node1->Root->GetId() == id1) {
          node1 = holder1.node1;
        } else {
          node1 = holder1.node2;
          std::reverse(plan1.begin(), plan1.end());
          reversed1 = true;
        }

        plan2 = holder2.plan;
        if (holder2.node1->Root->GetId() == id2) {
          node2 = holder2.node1;
        } else {
          node2 = holder2.node2;
          std::reverse(plan2.begin(), plan2.end());
          reversed2 = true;
        }

        Node<T, R> *last;
        while (!plan1.empty() && !plan2.empty() && plan1.back() == plan2.back()) {
          last = plan1.back();
          plan1.pop_back();
          plan2.pop_back();
        }

        while (plan1.size() > 0) {
          finalPlan.push_back(plan1.front());
          plan1.pop_front();
        }
        finalPlan.push_back(last);
        while (plan2.size() > 0) {
          finalPlan.push_back(plan2.back());
          plan2.pop_back();
        }

        T distance{computeDistance(finalPlan)};
        if (distance < link.distance - TOLERANCE) {     // for nonexisting connections always true (infinite distance at init)
          this->neighboringMatrix(id1, id2) = DistanceHolder<T, Node<T, R>>(node1, node2, distance, finalPlan);
        }
      }
    }
  }
}

template<class T, class R>
void Solver<T, R>::saveIterCheck(const int iter) {
  if (this->problem.saveTreeIter != 0 && !(iter % this->problem.saveTreeIter)) {
    std::string prefix{"iter_" + to_string(iter) + "_"};
    this->saveTrees(prefixFileName(this->problem.fileNames[SaveTree], prefix));
  }
}

template<class T, class R>
void Solver<T, R>::saveCities(const FileStruct file) {
  std::cout << "Saving points\n";
  std::ofstream fileStream{file.fileName.c_str()};
  if (!fileStream.good()) {
    std::cout << "Cannot create file at: " << file.fileName << "\n";
    return;
  }

  if (fileStream.is_open()) {
    if (file.type == Obj) {
      fileStream << "o Points\n";;
      for (int i{0}; i < problem.GetNumRoots(); ++i) { // goal is included
        for (Node<T, R> &node : this->trees[i].nodes){
          fileStream << "v" << DELIMITER_OUT << node.Position / problem.environment.ScaleFactor << "\n";
        }
      }
    } else if (file.type == Map) {
      for (int i{0}; i < problem.GetNumRoots(); ++i) {
        for(Node<T, R> &node : this->trees[i].nodes) {
          fileStream << node.Position / problem.environment.ScaleFactor << "\n";
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
void Solver<T, R>::saveTrees(const FileStruct file) {
  std::cout << "Saving trees\n";
  std::ofstream fileStream{file.fileName.c_str()};
  if (!fileStream.good()) {
    std::cout << "Cannot create file at: " << file.fileName << "\n";
    return;
  }

  if (fileStream.is_open()) {
    if (file.type == Obj) {
      fileStream << "o Trees\n";
      for (int i{0}; i < this->allNodes.size(); ++i) {
        Point<T> temp{this->allNodes[i]->Position / problem.environment.ScaleFactor};
        fileStream << "v" << DELIMITER_OUT;
        temp.printPosOnly(fileStream);
        fileStream << "\n";
      }

      for (int i{0}; i < this->trees.size(); ++i) {
        for (Node<T, R> &node : this->trees[i].nodes) {
          if (node.DistanceToRoot != 0) {
            fileStream << "l" << DELIMITER_OUT << node.GetId() + 1 << DELIMITER_OUT << node.Closest->GetId() + 1 << "\n";
          }
        }
      }
    } else if (file.type == Map) {
      fileStream << "#X1 Y1 Z1 Yaw1 Pitch1 Roll1 X2 Y2 Z2 Yaw2 Pitch2 Roll2 TreeID IterationOfCreation\n";
      for (int i{0}; i < this->trees.size(); ++i) {
        for (Node<T, R> &node : this->trees[i].nodes) {
          if (node.DistanceToRoot != 0) {
            fileStream << node.Position / problem.environment.ScaleFactor << DELIMITER_OUT << node.Closest->Position / problem.environment.ScaleFactor << DELIMITER_OUT << node.Root->GetId() << DELIMITER_OUT << node.GetAge() << "\n";
          }
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
void Solver<T, R>::checkPlan() {
  for (int i{0}; i < this->problem.GetNumRoots(); ++i) {
    for (int j{0}; j < this->problem.GetNumRoots(); ++j) {
      if (this->neighboringMatrix.Exists(i, j)) {
        DistanceHolder<T, Node<T, R>> &holder{this->neighboringMatrix(i, j)};
        for (auto &node : holder.plan) {
          if (node->GetId() == -1) {
            std::cout << "Fatal error!\n";
            exit(3);
          }
        }
      }
    }
  }
}

template<class T, class R>
void Solver<T, R>::checkPlan(std::deque<Node<T, R> *> &plan) {
  for (auto &node : plan) {
    if (node->GetId() == -1) {
      std::cout << "Fatal error!\n";
      exit(3);
    }
  }
}

template <class T, class R>
void Solver<T, R>::checkDistances(std::deque<Node<T, R> *> &plan, T distanceToCheck) {
  Node<T, R> *previous{nullptr};
  T distance{0};
  for (Node<T, R> *node : plan) {
    if (previous != nullptr) {
      if (!isPathFree(node->Position, previous->Position)) {
        std::cout << "Fatal error: path not feasible!\n";
        exit(1);
      }
      distance += previous->Position.distance(node->Position);
    }
    previous = node;
  }

  if (distance < distanceToCheck - TOLERANCE || distance > distanceToCheck + TOLERANCE) {
    std::cout << "Fatal error: Distances mismatch\n";
    exit(1);
  }
}

template<class T, class R>
void Solver<T, R>::saveParams(const FileStruct file, const int iterations, const bool solved, const std::chrono::duration<double> elapsedTime) {
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
    for (int i{0}; i < this->connectedTrees.size(); ++i) {
      fileStream << this->connectedTrees[i]->GetId();
      if (i + 1 != connectedTrees.size()) {
        fileStream << CSV_DELIMITER_2;
      }
    }
    fileStream << "]" << CSV_DELIMITER << "[";
    int numRoots{(int)this->connectedTrees.size()};
    for (int i{0}; i < numRoots; ++i) {
      for (int j{0}; j < i; ++j) {
        int id1{this->connectedTrees[i]->GetId()};
        int id2{this->connectedTrees[j]->GetId()};
        fileStream << neighboringMatrix(id1,id2).distance / problem.environment.ScaleFactor;
        if (i + 1 != numRoots || j + 1 != i) {
          fileStream << CSV_DELIMITER_2;
        }
      }
    }
    fileStream << "]" << CSV_DELIMITER;
    fileStream << elapsedTime.count() << "\n";
  } else {
    std::cout << "Cannot open file at: " << file.fileName << "\n";
  }
}

template <class T, class R>
void Solver<T, R>::saveTsp(const FileStruct file) {
  std::cout << "Saving TSP file\n";
  std::ofstream fileStream{file.fileName.c_str()};
  if (!fileStream.good()) {
    std::cout << "Cannot create file at: " << file.fileName << "\n";
    return;
  }

  if (fileStream.is_open()) {
    fileStream << "NAME: " << this->problem.id << "\n";
    fileStream << "COMMENT: ";
    for (int i{0}; i < this->connectedTrees.size(); ++i) {
      fileStream << this->connectedTrees[i]->GetId();
      if (i + 1 != connectedTrees.size()) {
        fileStream << TSP_DELIMITER;
      }
    }
    fileStream << "\n";
    fileStream << "TYPE: TSP\n";
    fileStream << "DIMENSION: " << this->connectedTrees.size() << "\n";
    fileStream << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    fileStream << "EDGE_WEIGHT_FORMAT : LOWER_DIAG_ROW\n";

    fileStream << "EDGE_WEIGHT_SECTION\n";
    int numRoots{(int)this->connectedTrees.size()};
    for (int i{0}; i < numRoots; ++i) {
      for (int j{0}; j < i; ++j) {
        int id1{this->connectedTrees[i]->GetId()};
        int id2{this->connectedTrees[j]->GetId()};
        fileStream << neighboringMatrix(id1, id2).distance / problem.environment.ScaleFactor << TSP_DELIMITER;
      }
      fileStream << "0\n";
    }
  } else {
    std::cout << "Cannot open file at: " << file.fileName << "\n";
  }
}

template<class T, class R>
void Solver<T, R>::savePaths(const FileStruct file) {
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
        Point<T> temp{this->allNodes[i]->Position / problem.environment.ScaleFactor};
        fileStream << "v" << DELIMITER_OUT;
        temp.printPosOnly(fileStream);
        fileStream << "\n";
      }
      
      for (int i{0}; i < numRoots; ++i) {
        for (int j{i + 1}; j < numRoots; ++j) {
          DistanceHolder<T, Node<T, R>> &holder{this->neighboringMatrix(i, j)};
          if (holder.node1 == NULL) {
            continue;
          }

          std::deque<Node<T, R> *> &plan{holder.plan};
          for (int k{0}; k < plan.size() - 1; ++k) {
            fileStream << "l" << DELIMITER_OUT << plan[k]->GetId() + 1 << DELIMITER_OUT << plan[k+1]->GetId() + 1 << "\n";
          }
        }
      }
    } else if (file.type == Map) {
      for (int i{0}; i < numRoots; ++i) {
        for (int j{i + 1}; j < numRoots; ++j) {
          DistanceHolder<T, Node<T, R>> &holder{this->neighboringMatrix(i, j)};
          if (holder.node1 == NULL) {
            continue;
          }

          std::deque<Node<T, R> *> &plan{holder.plan};
          for (int k{0}; k < plan.size() - 1; ++k) {
            fileStream << plan[k]->Position / problem.environment.ScaleFactor << DELIMITER_OUT << plan[k+1]->Position / problem.environment.ScaleFactor << "\n";
          }
          fileStream << "\n";
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

#endif
