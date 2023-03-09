// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  Abstraction of a Webots sensor
 */

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "Device.hpp"

class Sensor : public Device {
public:
  // Device Manager is responsible to create/destroy devices
  Sensor(WbDeviceTag tag, int index) : Device(tag, index), mSamplingPeriod(0), mLastRefreshTime(0), mSensorRequested(false) {}
  virtual ~Sensor() {}

  int lastRefreshTime() const { return mLastRefreshTime; }
  int samplingPeriod() const { return mSamplingPeriod; }
  bool isEnabled() const { return mSamplingPeriod > 0; }

  void setSamplingPeriod(int samplingPeriod) { mSamplingPeriod = samplingPeriod; }
  void setLastRefreshTime(int lastRefreshTime) { mLastRefreshTime = lastRefreshTime; }

  bool isSensorRequested() const { return mSensorRequested; }
  void resetSensorRequested() { mSensorRequested = false; }
  void setSensorRequested() { mSensorRequested = true; }

private:
  int mSamplingPeriod;   // ms
  int mLastRefreshTime;  // ms
  bool mSensorRequested;
};

#endif
