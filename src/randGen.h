/**
 * @file randGen.h
 * @author Jaroslav Janos (janosjar@fel.cvut.cz)
 * @brief 
 * @version 1.0
 * @date 24/06/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __RANDGEN_H__
#define __RANDGEN_H__

#include <random>
#include <math.h>
#include <vector>
#include <chrono>

#include "primitives.h"

typedef std::mt19937_64 randomEngine;   // set used random engine

template<class T>
class RandGen {
  public:
    RandGen(const Range<T> samplingRange);

    bool randomPointInDistance(const Point<T>& center, Point<T>& point, const T distance, const Dimensions dimension);
    bool randomPointInNormDistance(const Point<T>& center, Point<T>& point, const T mean, const T dispersion, const T distance);
    void randomPointInSpace(Point<T> &point, const Dimensions dimension);
    int randomIntMinMax(const int min, const int max);
    T randomProbability();

  private:
    Range<T> limits;
    T Distance;
    randomEngine rndEng;

    std::uniform_real_distribution<T> uniDistAngle;
    std::uniform_real_distribution<T> uniSpaceX;
    std::uniform_real_distribution<T> uniSpaceY;
    std::uniform_real_distribution<T> uniSpaceZ;
    std::uniform_real_distribution<T> uniProb;

    bool isInLimits(Point<T> &p);
};

template<class T>
RandGen<T>::RandGen(const Range<T> samplingRange) : limits{samplingRange} {
  // seed with actual time
  std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> tSeed{std::chrono::high_resolution_clock::now()};
  std::uint_fast64_t uSeed{static_cast<uint_fast64_t>(tSeed.time_since_epoch().count())};

  rndEng = randomEngine(uSeed);

  uniDistAngle = std::uniform_real_distribution<T>(-M_PI, M_PI);
  uniSpaceX = std::uniform_real_distribution<T>(limits.minX, limits.maxX);
  uniSpaceY = std::uniform_real_distribution<T>(limits.minY, limits.maxY);
  uniSpaceZ = std::uniform_real_distribution<T>(limits.minZ, limits.maxZ);
  uniProb = std::uniform_real_distribution<T>(0, 1);
}

/**
 * @brief Random sampling of position, according to https://ri.cmu.edu/pub_files/pub4/kuffner_james_2004_1/kuffner_james_2004_1.pdf 
 * 
 * @return true When the point is valid, i. e. is in limits (does not check nn) 
 */
template<class T>
bool RandGen<T>::randomPointInDistance(const Point<T>& center, Point<T>& point, const T distance, const Dimensions dimension) {
  Point<T> temp;
  
  // translation
  T phi{uniDistAngle(rndEng)};

  if (dimension == D2) {
    temp.setPosition(0, center.x() + cos(phi) * distance);
    temp.setPosition(1, center.y() + sin(phi) * distance);
    temp.setPosition(2, 0);
    temp.setPosition(3, 0);
    temp.setPosition(4, 0);
    temp.setPosition(5, 0);
    point = temp;

  } else if (dimension == D3) {
    T theta{uniDistAngle(rndEng)};
    temp.setPosition(0, center.x() + cos(theta) * sin(phi) * distance);
    temp.setPosition(1, center.y() + sin(theta) * sin(phi)  * distance);
    temp.setPosition(2, center.z() + cos(phi) * distance);
    
    // rotation
    temp.setPosition(3, uniDistAngle(rndEng)); // yaw
    phi = acos(1 - 2 * uniProb(rndEng)) + M_PI_2;
    if (uniProb(rndEng) < 0.5) {
      if (phi < 0) {
        phi += M_PI;
      } else {
        phi -= M_PI;
      }
    }
    temp.setPosition(4, phi);  // pitch
    temp.setPosition(5, uniDistAngle(rndEng)); // roll

    // the distance is not exactly the expected one (might be much greater), therefore a point in the same direction with the correct distance is needed
    point = center.getStateInDistance(temp, distance);
  }

  return isInLimits(point);
}

// NOT CORRECTED - does not make sense in 6DOF
template <class T>
bool RandGen<T>::randomPointInNormDistance(const Point<T>& center, Point<T>& point, const T mean, const T dispersion, const T distance) {
  std::normal_distribution<T> nrmDistPhi{mean, dispersion};
  T angle{nrmDistPhi(rndEng)};

  point.setPosition(0, center.x() + cos(angle) * distance);
  point.setPosition(1, center.y() + sin(angle) * distance);

  return isInLimits(point);
}

template<class T>
void RandGen<T>::randomPointInSpace(Point<T> &point, const Dimensions dimension) {
  T phi;

  point.set(uniSpaceX(rndEng), uniSpaceY(rndEng), 0);
  if (dimension == D2) {
    point.setPosition(3, 0);
    point.setPosition(4, 0);
    point.setPosition(5, 0);
  } else if (dimension == D3) {
    point.setPosition(2, uniSpaceZ(rndEng));  // z
    point.setPosition(3, uniDistAngle(rndEng)); // yaw
    phi = acos(1 - 2 * uniProb(rndEng)) + M_PI_2;
    if (uniProb(rndEng) < 0.5) {
      if (phi < 0) {
        phi += M_PI;
      } else {
        phi -= M_PI;
      }
    }
    point.setPosition(4, phi);  // pitch
    point.setPosition(5, uniDistAngle(rndEng)); // roll
  }
}

template<class T>
int RandGen<T>::randomIntMinMax(const int min, const int max) {
  std::uniform_int_distribution<int> uniInt(min, max);
  return uniInt(rndEng);
}

template <class T>
T RandGen<T>::randomProbability() {
  return uniProb(rndEng);
}

template<class T>
bool RandGen<T>::isInLimits(Point<T> &p) {
  bool valid{true};
  valid &= p.x() >= limits.minX;
  valid &= p.x() <= limits.maxX;
  valid &= p.y() >= limits.minY;
  valid &= p.y() <= limits.maxY;
  valid &= p.z() >= limits.minZ;
  valid &= p.z() <= limits.maxZ;

  return valid;
}

#endif
