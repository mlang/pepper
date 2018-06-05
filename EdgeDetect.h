#ifndef EDGEDETECT_H
#define EDGEDETECT_H

#include "EWMA.h"

template<typename T>
class EdgeDetect {
  T previousDifference = T(0);
  T threshold;
  EWMA<T> fastAverage, slowAverage;
public:
  EdgeDetect(T threshold = 0.2)
  : threshold(threshold)
  , fastAverage(0.25), slowAverage(0.0625)
  {}
  bool operator()(T value) {
    auto const difference = fastAverage(value) - slowAverage(value);
    auto const rising = previousDifference < threshold && difference > threshold;
    previousDifference = difference;
    return rising;
  }
};

#endif
