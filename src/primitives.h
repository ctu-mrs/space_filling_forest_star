/**
 * @file primitives.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief 
 * @version 1.0
 * @date 24/06/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __PRIMITIVES_H__
#define __PRIMITIVES_H__

#include <math.h>
#include <set>
#include <vector>
#include <deque>
#include <map>
#include <string>
#include <exception>
#include <regex>
#include <limits.h>
#include <queue>
#include <flann/flann.hpp>

#ifndef DELIMITER_OUT
#define DELIMITER_OUT (" ")
#endif

#ifndef CSV_DELIMITER
#define CSV_DELIMITER (",")
#endif

#ifndef CSV_DELIMITER_2
#define CSV_DELIMITER_2 (";")
#endif

#ifndef TSP_DELIMITER
#define TSP_DELIMITER (" ")
#endif

#define DEFAULT_THRES_MISS 3
#define DEFAULT_OBSTAC_MISS 10
#define TOLERANCE 1e-9

#define SAMPLING_ANGLE_DISP 1 // something about 60 degrees
#define DEFAULT_SAMP_DIST 1
#define EXPANSION_MULTIPLIER 2
#define DEFAULT_DIST_DIV 1
#define ANGLE_MOVE 0.8

#define PROBLEM_DIMENSION   static_cast<size_t>(this->problem.dimension)

#define MIN(X, Y) ((X < Y) ? (X) : (Y))
#define MAX(X, Y) ((X > Y) ? (X) : (Y))

const std::string WHITESPACE = " \n\r\t\f\v";

template<class T> class Point;
template <class T> class Vector;
template<class T, class R=Point<T>> class Node;
template <class T, class R> struct DistanceHolder;
template <class T, class R> class Tree;
struct FileStruct;
template<class T, class R> class Heap;

int parseString(std::string &inp, std::string &outp1, std::string &outp2, std::string &delimiter);
FileStruct prefixFileName(const FileStruct &path, const std::string &insert);
template <class T> T NormalizeAngle(T angle);
template <class T, class R> T AngleDifference(T a1, R a2);

template <class T> T Distance(Node<T> &node1, Node<T> &ref);
template <class T> T StarDistance(Node<T> &node1, Node<T> &ref);

enum Dimensions {
  D2 = 2,
  D3 = 6
};

template <typename T> 
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template<class T>
class Point {
  public:
    T Yaw{0};
    T Pitch{0};
    T Roll{0};
    
    Point<T>() : coords{0, 0, 0} {

    }

    Point<T>(T x, T y, T z) : coords{x, y, z} {

    }

    Point<T>(T x, T y, T z, T yaw, T pitch, T roll) : coords{x, y, z}, Yaw{yaw}, Pitch{pitch}, Roll{roll} {
    }

    Point<T>(const std::string &s, T scale=1) {
      std::regex r("\\[(\\-?[\\d.]+);\\s*(\\-?[\\d.]+);\\s*(\\-?[\\d.]+)\\]");
      std::smatch m;
      std::regex_search(s, m, r);
      if (m.size() != 4) {
        throw std::invalid_argument("Unknown format of point");
      }
      for (int i{0}; i < 3; ++i) {
        coords[i] = std::stod(m[i + 1]) * scale;
      }
    }

    Point<T>& operator=(const Point<T> &p) {
      if(*this != p) {
        for (int i{0}; i < 3; ++i) {
          coords[i] = p[i];
        }
      }
      Yaw = p.Yaw;
      Pitch = p.Pitch;
      Roll = p.Roll;

      return *this;
    }

    T x() const {
      return coords[0];
    }

    T y() const {
      return coords[1];
    }

    T z() const {
      return coords[2];
    }

    void set(T x, T y, T z) {
      coords[0] = x;
      coords[1] = y;
      coords[2] = z;
    }

    void set(T x, T y, T z, T yaw, T pitch, T roll) {
      coords[0] = x;
      coords[1] = y;
      coords[2] = z;
      Yaw = yaw;
      Pitch = pitch;
      Roll = roll;
    }

    const T* get() const {
      return coords;
    }

    void setPosition(unsigned int pos, T val) {
      if (pos > 5) {
        return;
      } else if (pos < 3) {
        coords[pos] = val;
      } else if (pos == 3) {
        Yaw = val;
      } else if (pos == 4) {
        Pitch = val;
      } else if (pos == 5) {
        Roll = val;
      }
    }

    inline const T* operator()() const {
      return coords;
    }

    const T operator[](int i) const {
      if (i < 3) {
        return coords[i];
      } else if (i == 3) {
        return Yaw;
      } else if (i == 4) {
        return Pitch;
      } else if (i == 5) {
        return Roll;
      } else {
        return 1;
      }
    }

    inline void operator+=(const Vector<T> &translate) {
      for (int i{0}; i < 3; ++i) {
        coords[i] += translate[i];
      }
    }

    friend bool operator==(const Point<T> &p1, const Point<T> &p2) {
      return p1.x() == p2.x() && p1.y() == p2.y() && p1.z() == p2.z() && p1.Yaw == p2.Yaw && p1.Pitch == p2.Pitch && p1.Roll == p2.Roll;
    }

    friend bool operator!=(const Point<T> &p1, const Point<T> &p2) {
      return p1.x() != p2.x() || p1.y() != p2.y() || p1.z() != p2.z() || p1.Yaw != p2.Yaw || p1.Pitch != p2.Pitch || p1.Roll != p2.Roll;
    }

    friend bool operator<(const Point<T> &p1, const Point<T> &p2) {
      return p1.x() < p2.x() ||
            (p1.x() == p2.x() && p1.y() < p2.y()) ||
            (p1.x() == p2.x() && p1.y() == p2.y() && p1.z() < p2.z()) ||
            (p1.x() == p2.x() && p1.y() == p2.y() && p1.z() == p2.z() && p1.Yaw < p2.Yaw) ||
            (p1.x() == p2.x() && p1.y() == p2.y() && p1.z() == p2.z() && p1.Yaw == p2.Yaw && p1.Pitch < p2.Pitch) ||
            (p1.x() == p2.x() && p1.y() == p2.y() && p1.z() == p2.z() && p1.Yaw == p2.Yaw && p1.Pitch == p2.Pitch && p1.Roll < p2.Roll);
    }

    // scale position (NOT the rotation)
    friend Point<T> operator/(const Point<T> &p1, const T scale) {
      Point<T> newPoint{p1};
      for (int i{0}; i < 3; ++i) {
        newPoint.coords[i] /= scale;
      }
      return newPoint;
    }

    T distance(const Point<T> &other) const {
      T sum{0};
      for (int i{0}; i < 3; ++i) {
        T diff{(*this)[i] - other[i]};
        sum += diff * diff;
      }
      for (int i{3}; i < 6; ++i) {
        T diff{AngleDifference((*this)[i], other[i])};
        sum += diff * diff;
      }
      return sqrt(sum);
    }

    Point<T> getStateInDistance(Point<T> &other, T dist) const {
      T realDist{distance(other)};
      Vector<T> direction(*this, other);
      Vector<T> angleDir(AngleDifference((*this)[3], other[3]), AngleDifference((*this)[4], other[4]), AngleDifference((*this)[5], other[5]));
      Point <T> retVal;

      for (int i{0}; i < 3; ++i) {
        retVal.setPosition(i, coords[i] + direction[i] * (dist / realDist));
      }
      for (int i{0}; i < 3; ++i) {
        retVal.setPosition(i + 3, (*this)[i + 3] + angleDir[i] * (dist / realDist));
      }
      return retVal;
    }

    void FillRotationMatrix(T (&matrix)[3][3]) {
      matrix[0][0] = cos(Yaw) * cos(Pitch);
      matrix[0][1] = cos(Yaw) * sin(Pitch) * sin(Roll) - sin(Yaw) * cos(Roll);
      matrix[0][2] = cos(Yaw) * sin(Pitch) * cos(Roll) + sin(Yaw) * sin(Roll);
      matrix[1][0] = sin(Yaw) * cos(Pitch);
      matrix[1][1] = sin(Yaw) * sin(Pitch) * sin(Roll) + cos(Yaw) * cos(Roll);
      matrix[1][2] = sin(Yaw) * sin(Pitch) * cos(Roll) - cos(Yaw) * sin(Roll);
      matrix[2][0] = -sin(Pitch);
      matrix[2][1] = cos(Pitch) * sin(Roll);
      matrix[2][2] = cos(Pitch) * cos(Roll);
    }

    void printPosOnly(std::ostream &out) {
      out << this->x() << DELIMITER_OUT << this->y() << DELIMITER_OUT << this->z();
    }

  private:
    T coords[3];
};

template <class T>
std::ostream& operator<<(std::ostream &out, const Point<T> &p) {
  return out << p.x() << DELIMITER_OUT << p.y() << DELIMITER_OUT << p.z() << DELIMITER_OUT << p.Yaw << DELIMITER_OUT << p.Pitch << DELIMITER_OUT << p.Roll;
}

template <class T>
T NormalizeAngle(T angle) {
  if (angle < -M_PI) {
    return angle + 2 * M_PI;
  } else if (angle >= M_PI) {
    return angle - 2 * M_PI;
  } else {
    return angle;
  }
}

template <class T, class R>
T AngleDifference(T a1, R a2) {
  T diff{(T)a2 - a1};
  return NormalizeAngle(diff);
}

template <class T>
class Vector {
 public:
  Vector() : coords{0, 0, 0} {
  }

  Vector(T x, T y, T z) : coords{x, y, z} {
  }

  Vector(Point<T> p) : coords{p.x(), p.y(), p.z()} {
  }

  Vector(Point<T> p1, Point<T> p2) {
    for (int i{0}; i < 3; ++i) {
      coords[i] = p2[i] - p1[i];
    }
  }

  inline T* operator()() {
    return coords;
  }

  inline T operator[](int i) const {
    return coords[i];
  }

  friend T operator*(Vector<T> v1, Vector<T> v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  }

  friend Vector<T> operator|(Vector<T> v1, Vector<T> v2) {
    T  cache[3];
    cache[0] = v1[1] * v2[2] - v1[2] * v2[1];
    cache[1] = v1[2] * v2[0] - v1[0] * v2[2];
    cache[2] = v1[0] * v2[1] - v1[1] * v2[0];

    Vector<T> retVal{cache[0], cache[1], cache[2]};
    return retVal;
  }

  friend Vector<T> operator-(const Vector<T> v1, const Vector<T> v2) {
    return Vector(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]);
  }

  friend Vector<T> operator+(const Vector<T> v1, const Vector<T> v2) {
    return Vector(v2[0] + v1[0], v2[1] + v1[1], v2[2] + v1[2]);
  }

  inline void operator+=(const Vector<T> &v) {
    for (int i{0}; i < 3; ++i) {
      this->coords[i] += v[i];
    }
  }

  inline void operator-=(const Vector<T> &v) {
    for (int i{0}; i < 3; ++i) {
      this->coords[i] -= v[i];
    }
  }

  Vector<T> inverse(const Vector<T> &vec) {
    return Vector(-vec[0], -vec[1], -vec[2]);
  } 

  T size() const {
    T retVal{0};

    for (int i{0}; i < 3; ++i) {
      retVal += coords[i] * coords[i];
    }

    return sqrt(retVal);
  }

  void normalize() {
    T s{this->size()};
    for (int i{0}; i < 3; ++i) {
      coords[i] /= s;
    }
  }

 private:
  T coords[3];
};

template <class T>
class Triangle {
 public:
  Triangle(Point<T> x, Point<T> y, Point<T> z) : vertices{x, y, z} {
  }

  inline Point<T> operator[](int i) {
    return vertices[i];
  }

 protected:
  Point<T> vertices[3];
};

template <class T>
struct Range {
  T minX;
  T maxX;
  T minY;
  T maxY;
  T minZ;
  T maxZ;
};

// FLANN FUNCTOR
template<class T>
struct D6Distance {
  typedef bool is_kdtree_distance;

  typedef T ElementType;
  typedef typename flann::Accumulator<T>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType worst_dist = -1) const {
    ResultType result = ResultType();
    ResultType diff;

    for (int i{0}; i < 3; ++i) {
      diff = (ResultType)(*a++ - *b++);
      result = diff * diff;
    }

    for (int i{3}; i < 6; ++i) {
      diff = (ResultType)AngleDifference(*a++, *b++);
      result = diff * diff;
    }
    return result;
  }

  template<typename U, typename V>
	inline ResultType accum_dist(const U& a, const V& b, int part) const
			{
		if ( part > 2 ) {
			ResultType ang_diff = (ResultType)AngleDifference(a, b);
			return ang_diff * ang_diff;
		} else {
			return (a - b) * (a - b);
		}
	}
};

template <class T, class R>
class Node {
  public:
    inline static char ThresholdMisses = DEFAULT_THRES_MISS;
    
    inline static double SamplingDistance = DEFAULT_SAMP_DIST;

    R Position;
    Tree<T, Node> *Root;
    Tree<T, Node> *ExpandedRoot;
    Node *Closest;
    std::deque<Node *> Children;
    bool ForceChildren{ false };
    T DistanceToClosest;
    T DistanceToRoot;
    std::map<Node*, T> VisibleNodes;

    Node(R position, Tree<T, Node> *root, Node *closest, T distanceToClosest, T distanceToRoot, unsigned int iteration) : Position{position},
      Root{root}, Closest{closest}, DistanceToClosest{distanceToClosest}, DistanceToRoot{distanceToRoot}, generation{iteration} {
        id = globId++;

        if (closest == nullptr) {
          ExpandedRoot = Root;
        } else {
          ExpandedRoot = closest->ExpandedRoot;
        }
      }

    friend bool operator==(const Node &r, const Node &l) {
      return r.GetId() == l.GetId();
    }

    friend bool operator<(const Node &r, const Node &l) {
      return r.GetId() < l.GetId();
    }

    const bool IsRoot() const {
      return DistanceToRoot == 0;
    }

    const int GetId() const {
      return id;
    }

    const int GetNumNodes() const {
      return globId;
    }

    const unsigned int GetAge() const {
      return generation;
    }

  private:
    inline static unsigned int globId = 0;
    unsigned int id;
    unsigned int generation;
    bool nearObstacle { false };

};

template<class T, class R=Node<T>>
class Tree {
  public:
    inline static bool AStar = false;
    std::deque<R> nodes;
    R *Root;
    flann::Index<D6Distance<float>> *flannIndex;
    std::deque<float *> ptrToDel;
    std::deque<DistanceHolder<T, R>> links;
    std::deque<Heap<T, R>> frontiers;
    std::vector<bool> frontierFilter; // true for filtering
    std::deque<Tree *> eaten;

    Tree() {
      id = globId++;
    }

    ~Tree() {
      if (flannIndex != NULL) {
        delete flannIndex;
      }

      for (auto &p : ptrToDel) {
        delete[] p;
      }
    }

    friend bool operator==(const Tree &r, const Tree &l) {
      return r.GetId() == l.GetId();
    }

    void AddFrontier(R *goal) {
      Heap<T, R> prior;
      if (AStar) {
        prior = Heap<T, R>(this->nodes, goal, true, StarDistance);
      } else {
        prior = Heap<T, R>(this->nodes, goal, true, Distance);
      }
      frontiers.push_back(std::move(prior));
      frontierFilter.push_back(false);
    }

    const bool EmptyFrontiers() {
      bool retVal{true};
      for (auto &prior : frontiers) {
        retVal &= prior.empty();
      }

      bool fullFilter{true};
      for (bool filter : frontierFilter) {
        fullFilter &= filter;
      }
      retVal |= fullFilter;
      
      return retVal;
    }

    void EnableFrontier() {
      for (int i{0}; i < frontierFilter.size(); ++i) {
        frontierFilter[i] = false;
      }
    }

    const int GetId() const {
      return id;
    }

  private:
    inline static unsigned int globId = 0;
    unsigned int id;
};

template<class T>
class SymmetricMatrix {
  public:
    SymmetricMatrix(const int size) {
      holder.resize(size * (size+1) / 2);
      this->size = size;
    }

    T& operator()(int i, int j) {
      int index;
      if (i <= j) {
        index = i * size - (i - 1) * i / 2 + j - i;
      } else {
        index = j * size - (j - 1) * j / 2 + i - j;
      }
      return holder[index];
    }

    const bool Exists(int i, int j) {
      return this->operator()(i, j).Exists();
    }
  private:
    std::deque<T> holder;
    int size;
};

template<class T, class R>
struct DistanceHolder {
  R *node1;
  R *node2;
  T distance;
  std::deque<R *> plan;

  DistanceHolder() : node1{NULL}, node2{NULL}, distance{std::numeric_limits<T>::max()} {
  }

  DistanceHolder(R *first, R *second) : node1{first}, node2{second} {
    if (*first < *second) {
      node1 = first;
      node2 = second;
    } else {
      node1 = second;
      node2 = first;
    }
    distance = first->DistanceToRoot + second->DistanceToRoot + first->Position.distance(second->Position);
  }

  DistanceHolder(R *first, R *second, T dist) : distance{dist} {
    if (*first < *second) {
      node1 = first;
      node2 = second;
    } else {
      node1 = second;
      node2 = first;
    }
  }

  DistanceHolder(R *first, R *second, T dist, std::deque<R *> &plan) : distance{dist}, plan{plan} {
    if (*first < *second) {
      node1 = first;
      node2 = second;
    } else {
      node1 = second;
      node2 = first;
      std::reverse(this->plan.begin(), this->plan.end());
    }
  }

  friend bool operator<(const DistanceHolder<T, R> &l, const DistanceHolder<T, R> &r) {
    return l.distance < r.distance;
  }  

  friend bool operator==(const DistanceHolder<T, R> &l, const DistanceHolder<T, R> &r) {
    return l.node1 == r.node1 && l.node2 == r.node2;
  }

  const bool Exists() const {
    return node1 != nullptr;
  }

  void UpdateDistance() {
    distance = node1->DistanceToRoot + node2->DistanceToRoot + node1->Position.distance(node2->Position);
  }
};


enum FileType {
  Map,
  Obj
};

struct FileStruct {
  std::string fileName;
  FileType type;
};

/**
 * @brief Extension of the Node, with ability to hold distance and position data
 * 
 * @tparam T Floating type
 */
template<typename T>
struct PathNode {
	T distanceFromStart{std::numeric_limits<T>::max()};
  int heapPosition{-1};
  Node<T, Point<T>> *previousPoint;
};

int parseString(std::string &inp, std::string &outp1, std::string &outp2, std::string &delimiter) {
  size_t pos = inp.find(delimiter);
  int delimSize{static_cast<int>(delimiter.size())};
  int miss{1};
  if (pos != std::string::npos) {
    while (inp[pos + miss] == delimiter[miss]) {
      ++miss;
    }
    outp1 = inp.substr(0, pos);
    outp2 = inp.substr(pos + miss, inp.length());
    return pos;
  } else {
    outp1 = inp.substr(0, inp.length());
    outp2 = "";
    return -1;
  }
}

FileStruct prefixFileName(const FileStruct &path, const std::string &insert) {
	FileStruct retVal{path};

	auto pos{retVal.fileName.find_last_of("//")};
  if (pos != std::string::npos) {
    retVal.fileName.insert(pos + 1, insert);
  } else {
    retVal.fileName.insert(0, insert);
  }

	return retVal;
}

std::string ltrim(const std::string &s) {
  size_t start{s.find_first_not_of(WHITESPACE)};
  return (start == std::string::npos) ? "" : s.substr(start);
}

std::string rtrim(const std::string &s) {
  size_t end{s.find_last_not_of(WHITESPACE)};
  return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}

std::string trim(const std::string &s) {
  return ltrim(rtrim(s));
}

// COST FUNCTIONS
template <class T>
T Distance(Node<T> &node1, Node<T> &ref) {
  return node1.Position.distance(ref.Position);
}

template <class T>
T StarDistance(Node<T> &node1, Node<T> &ref) {
  return 0.7 * node1.Position.distance(ref.Position) + 0.3 * node1.DistanceToRoot;
}

#endif
