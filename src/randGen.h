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

    bool randomPointInDistance(const Point<T>& center, Point<T>& point, const T distance);
    bool randomPointInNormDistance(const Point<T>& center, Point<T>& point, const T mean, const T dispersion, const T distance);
    void randomPointInSpace(Point<T> &point);
    int randomIntMinMax(const int min, const int max);
    T randomProbability();

  private:
    Range<T> limits;
    T Distance;
    randomEngine rndEng;

    std::uniform_real_distribution<T> uniDistPhi;
    //std::uniform_real_distribution<T> uniDistDist;
    std::uniform_real_distribution<T> uniSpaceX;
    std::uniform_real_distribution<T> uniSpaceY;
    std::uniform_real_distribution<T> uniProb;

    bool isInLimits(Point<T> &p);
};

template<class T>
RandGen<T>::RandGen(const Range<T> samplingRange) : limits{samplingRange} {
  // seed with actual time
  std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> tSeed{std::chrono::high_resolution_clock::now()};
  std::uint_fast64_t uSeed{static_cast<uint_fast64_t>(tSeed.time_since_epoch().count())};

  rndEng = randomEngine(uSeed);

  uniDistPhi = std::uniform_real_distribution<T>(-M_PI, M_PI);
  uniSpaceX = std::uniform_real_distribution<T>(limits.minX, limits.maxX);
  uniSpaceY = std::uniform_real_distribution<T>(limits.minY, limits.maxY);
  uniProb = std::uniform_real_distribution<T>(0, 1);
}

/**
 * @return true When the point is valid, i. e. is in limits (does not check nn) 
 */
template<class T>
bool RandGen<T>::randomPointInDistance(const Point<T>& center, Point<T>& point, const T distance) {
  T angle{uniDistPhi(rndEng)};

  point.setPosition(0, center.x() + cos(angle) * distance);
  point.setPosition(1, center.y() + sin(angle) * distance);

  return isInLimits(point);
}

template <class T>
bool RandGen<T>::randomPointInNormDistance(const Point<T>& center, Point<T>& point, const T mean, const T dispersion, const T distance) {
  std::normal_distribution<T> nrmDistPhi{mean, dispersion};
  T angle{nrmDistPhi(rndEng)};

  point.setPosition(0, center.x() + cos(angle) * distance);
  point.setPosition(1, center.y() + sin(angle) * distance);

  return isInLimits(point);
}

template<class T>
void RandGen<T>::randomPointInSpace(Point<T> &point) {
  point.set(uniSpaceX(rndEng), uniSpaceY(rndEng));
}

template<class T>
int RandGen<T>::randomIntMinMax(const int min, const int max) {
  auto uniInt{std::uniform_int_distribution<int>(min, max)};
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

  return valid;
}

#endif
