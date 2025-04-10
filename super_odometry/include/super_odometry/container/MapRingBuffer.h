//
// Created by ubuntu on 2020/6/29.
//

#ifndef MAPRINGBUFFER_H
#define MAPRINGBUFFER_H

#include <iostream>
#include <map>

template <typename Meas>
class MapRingBuffer {
public:
  std::map<double, Meas> measMap_;
  typename std::map<double, Meas>::iterator itMeas_;

  int size;
  double maxWaitTime_;
  double minWaitTime_;

  MapRingBuffer() {
    maxWaitTime_ = 0.1;
    minWaitTime_ = 0.0;
  }

  virtual ~MapRingBuffer() {}

  bool allocate(const int sizeBuffer) {
    if (sizeBuffer <= 0) {
      return false;
    } else {
      size = sizeBuffer;
      return true;
    }
  }

  int getSize() { return measMap_.size(); }

  void addMeas(const Meas& meas, const double& t) {
    measMap_.insert(std::make_pair(t, meas));

    // ensure the size of the map, and remove the last element
    if ((int) measMap_.size() > size) {
      measMap_.erase(measMap_.begin());
    }
  }

  void clear() { measMap_.clear(); }

  void clean(double t) {
    while (measMap_.size() >= 1 && measMap_.begin()->first <= t) {
      measMap_.erase(measMap_.begin());
    }
  }

  bool getNextTime(double actualTime, double& nextTime) {
    itMeas_ = measMap_.upper_bound(actualTime);
    if (itMeas_ != measMap_.end()) {
      nextTime = itMeas_->first;
      return true;
    } else {
      return false;
    }
  }
  void waitTime(double actualTime, double& time) {
    double measurementTime = actualTime - maxWaitTime_;
    if (!measMap_.empty() &&
        measMap_.rbegin()->first + minWaitTime_ > measurementTime) {
      measurementTime = measMap_.rbegin()->first + minWaitTime_;
    }
    if (time > measurementTime) {
      time = measurementTime;
    }
  }
  bool getLastTime(double& lastTime) {
    if (!measMap_.empty()) {
      lastTime = measMap_.rbegin()->first;
      return true;
    } else {
      return false;
    }
  }

  bool getFirstTime(double& firstTime) {
    if (!measMap_.empty()) {
      firstTime = measMap_.begin()->first;
      return true;
    } else {
      return false;
    }
  }

  bool getLastMeas(Meas& lastMeas) {
    if (!measMap_.empty()) {
      lastMeas = measMap_.rbegin()->second;
      return true;
    } else {
      return false;
    }
  }

  bool getLastLastMeas(Meas& lastlastMeas) {
    if (measMap_.size() >= 2) {
      auto itr = measMap_.rbegin();
      itr++;
      lastlastMeas = itr->second;
      return true;
    } else {
      return false;
    }
  }

  bool getFirstMeas(Meas& firstMeas) {
    if (!measMap_.empty()) {
      firstMeas = measMap_.begin()->second;
      return true;
    } else {
      return false;
    }
  }

  bool hasMeasurementAt(double t) { return measMap_.count(t) > 0; }

  bool empty() { return measMap_.empty(); }

  void printContainer() {
    itMeas_ = measMap_.begin();
    while (measMap_.size() >= 1 && itMeas_ != measMap_.end()) {
      std::cout << itMeas_->second << " ";
      itMeas_++;
    }
  }
};
#endif // MAPRINGBUFFER_H
