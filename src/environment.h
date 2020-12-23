/**
 * @file environment.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief 
 * @version 1.0
 * @date 24/06/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <limits.h>
#include <cstring>

#include "primitives.h"
#include "RAPID.H"

using namespace std;
template<class T> class Obstacle;

template <class T>
class Environment {
  public:
    std::deque<Obstacle<T>> Obstacles;
    Obstacle<T> *Robot;
    Range<T> limits{__DBL_MAX__, -__DBL_MAX__, __DBL_MAX__, -__DBL_MAX__, __DBL_MAX__, -__DBL_MAX__};
    bool HasMap{true};
    T ScaleFactor;

    Environment() : Robot{nullptr} {
    }

    ~Environment() {
      delete Robot;
    }

    bool Collide(Point<T> position); // whether robot in such position collides with any known obstacle
    
    void processLimits(Range<T> &limits) {
      this->limits.minX = MIN(this->limits.minX, limits.minX);
      this->limits.maxX = MAX(this->limits.maxX, limits.maxX);
      this->limits.minY = MIN(this->limits.minY, limits.minY);
      this->limits.maxY = MAX(this->limits.maxY, limits.maxY);
      this->limits.minZ = MIN(this->limits.minZ, limits.minZ);
      this->limits.maxZ = MAX(this->limits.maxZ, limits.maxZ);
    }

};

// obstacle 
template<class T>
class Obstacle {
  public:
    inline static std::string Delimiter = " ";
    inline static std::string NameDelimiter = "_";

    Point<T> Position;
    Obstacle() {}

    Obstacle(const std::string fileName, const bool isObj, const Point<T> position, const T scaleFactor);
    Obstacle(const std::string fileName, const bool isObj, const T scaleFactor);

    virtual ~Obstacle();
    void ParseOBJFile(const std::string fileName);
    void ParseMapFile(const std::string fileName);
  
    Point<T> getPosition();
    RAPID_model *getRapidModel();
    Range<T> &getRange();

    static bool Collide(Obstacle<T> &object1, Point<T> pos1, Obstacle<T> &object2, Point<T> pos2);
    static bool Collide(Obstacle<T> &object1, Obstacle<T> &object2);
    static bool Collide(Obstacle<T> &object, Obstacle<T> &robot, Point<T> robPos);

  protected:
    std::vector<Point<T>> facePoints;
    std::vector<Triangle<T>> faces;
    RAPID_model *rapidModel = NULL;
    Range<T> localRange{__DBL_MAX__, -__DBL_MAX__, __DBL_MAX__, -__DBL_MAX__, __DBL_MAX__, -__DBL_MAX__};
    T scale{1};
    inline static int rapidId = 0;
    inline static double eyeRotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; 

    virtual void addPoint(int objId, T coords[3]);
    virtual void addFacet(int objId, int offset, int faceInts[3]);
};

template <class T>
Obstacle<T>::Obstacle(const std::string fileName, const bool isObj, const T scaleFactor) : Obstacle<T>(fileName, isObj, Point<T>(), scaleFactor) {

}

template <class T>
Obstacle<T>::Obstacle(const std::string fileName, const bool isObj, const Point<T> position, const T scaleFactor) : Position{position}, scale{scaleFactor} {
  this->rapidModel = new RAPID_model();
  this->rapidModel->BeginModel();

  if (isObj) {
    // OBJ file
    ParseOBJFile(fileName);
  } else {
    // other map file
    ParseMapFile(fileName);
  }
  

  this->rapidModel->EndModel();
}

template <class T>
Obstacle<T>::~Obstacle() {
  if (rapidModel != NULL) {
    delete rapidModel;
  }
}

template <class T>
void Obstacle<T>::ParseOBJFile(const std::string fileName) {
  std::ifstream modelFile(fileName);
  unsigned int elem{0}, offset{0};
  int faceCache[3], objId{0};
  T pointCache[3];
  std::string line, value;

  while (getline(modelFile, line)) {
    parseString(line, value, line, this->Delimiter);
    switch (value[0]) {
      case 'v':
        for (int i{0}; i < 3; ++i) {
          T d;
          parseString(line, value, line, this->Delimiter);
          d = std::stod(value);
          pointCache[i] = d + Position[i];
        }
        addPoint(objId, pointCache);
        ++elem;
        break;
      case 'f':
        for (int i{0}; i < 3; ++i) {
          int k;
          parseString(line, value, line, this->Delimiter);
          k = std::stoi(value);
          faceCache[i] = k;
        }

        addFacet(objId, offset, faceCache);
        break;
      case 'o':
        if (objId != 0) {
          offset += elem;
          elem = 0;
        }

        parseString(line, value, line, this->NameDelimiter);
        break;
    }
  }
  modelFile.close();
}

template <class T>
void Obstacle<T>::ParseMapFile(const std::string fileName) {
  std::ifstream modelFile(fileName);
  int index{1};
  int faceCache[3];
  T pointCache[3] = {0, 0, 0};
  std::string line, value;

  while (getline(modelFile, line)) {
    line = trim(line);
    if (!strcmp(line.c_str(), "")) {
      continue;
    }

    for (int i{0}; i < 3; ++i) {
      for (int j{0}; j < 2; ++j) {
        parseString(line, value, line, this->Delimiter);
        pointCache[j] = std::stod(value) + Position[j];
      }
      addPoint(0, pointCache);
      faceCache[i] = index + i;
    }
    index += 3;

    addFacet(0, 0, faceCache);
  }
  modelFile.close();
}

template <class T>
void Obstacle<T>::addPoint(int objId, T coords[3]) {
  // scale
  for (int i{0}; i < 3; ++i) {
    coords[i] *= scale;
  }

  this->facePoints.emplace_back(coords[0], coords[1], coords[2]);
  localRange.minX = MIN(localRange.minX, coords[0]);
  localRange.maxX = MAX(localRange.maxX, coords[0]);
  localRange.minY = MIN(localRange.minY, coords[1]);
  localRange.maxY = MAX(localRange.maxY, coords[1]);
  localRange.minZ = MIN(localRange.minZ, coords[2]);
  localRange.maxZ = MAX(localRange.maxZ, coords[2]);
}

template <class T>
void Obstacle<T>::addFacet(int objId, int offset, int faceInts[3]) {
  Point<T> faceCache[3];
  for (int i{0}; i < 3; ++i) {
    int pos{faceInts[i] - offset - 1};
    faceCache[i] = this->facePoints[pos];
  }

  this->faces.emplace_back(faceCache[0], faceCache[1], faceCache[2]);
  this->rapidModel->AddTri(faceCache[0](), faceCache[1](), faceCache[2](), this->rapidId++);
}

/**
 * @brief Handler for the RAPID functions for the collision checking
 * 
 * @param object1 First object to be tested for collisions
 * @param pos1 Position of the first object
 * @param object2 Second object to be tested
 * @param pos2 Position of the second object
 * @return true If objects collide
 * @return false Otherwise
 * 
 * BEWARE THAT THE DEFAULT POSITION OF OBJECT IS ALREADY CONSIDERED AND SHOULD NOT BE
 * ADDITIONALLY PASSED TO RAPID
 */

template <class T>
bool Obstacle<T>::Collide(Obstacle<T> &object1, Point<T> pos1, Obstacle<T> &object2, Point<T> pos2) {  
  Vector<T> vecPos1{pos1}, vecPos2{pos2};
  T rotMat1[3][3], rotMat2[3][3];
  pos1.FillRotationMatrix(rotMat1);
  pos2.FillRotationMatrix(rotMat2);

  RAPID_Collide(rotMat1, vecPos1(), object1.getRapidModel(), rotMat2, vecPos2(), object2.getRapidModel());
  return RAPID_num_contacts != 0;
}

/**
 * @brief Handler of the previous function, assuming null positions of the objects
 * 
 * BEWARE THAT THE DEFAULT POSITION OF OBJECT IS ALREADY CONSIDERED AND SHOULD NOT BE
 * ADDITIONALLY PASSED TO RAPID
 */
template <class T>
bool Obstacle<T>::Collide(Obstacle<T> &object1, Obstacle<T> &object2) {
  Point<T> nullPnt;
  return Collide(object1, nullPnt, object2, nullPnt);
}

/**
 * @brief Handler of the base Collide function, tailored for the robot model and its position
 * 
 * BEWARE THAT THE DEFAULT POSITION OF OBJECT IS ALREADY CONSIDERED AND SHOULD NOT BE
 * ADDITIONALLY PASSED TO RAPID
 */
template <class T>
bool Obstacle<T>::Collide(Obstacle<T> &object, Obstacle<T> &robot, Point<T> robPos) {
  Vector<T> vecPos1, vecPos2{robPos};
  T rotMat2[3][3];
  robPos.FillRotationMatrix(rotMat2);

  RAPID_Collide(Obstacle<T>::eyeRotation, vecPos1(), object.getRapidModel(), rotMat2, vecPos2(), robot.getRapidModel());
  return RAPID_num_contacts != 0;
}

/**
 * @brief Simple getter for the position of the particular object
 * 
 * @tparam T Floating type
 * @return Point<T> Position of the object
 */
template <class T>
Point<T> Obstacle<T>::getPosition() {
  return this->position;
}

/**
 * @brief Simple getter for the pointer to the RAPID model of the object
 * 
 * @tparam T Floating type
 * @return RAPID_model*  Pointer to the model
 */
template <class T>
RAPID_model *Obstacle<T>::getRapidModel() {
  return this->rapidModel;
}

template <class T>
Range<T>& Obstacle<T>::getRange() {
  return this->localRange;
}

template <class T>
bool Environment<T>::Collide(Point<T> position) {
  if (!this->HasMap) {
    return false;
  } 
  
  bool retVal{false};
  for (Obstacle<T> &obs : this->Obstacles) {
    retVal |= Obstacle<T>::Collide(obs, *(this->Robot), position);
  }
  return retVal;
}

#endif
