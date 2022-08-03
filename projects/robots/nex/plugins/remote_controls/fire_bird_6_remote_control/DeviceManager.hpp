// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  Class allowing to create or retrieve devices
 */

#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

#include <webots/types.h>

#include <vector>

class Device;
class Motor;
class SingleValueSensor;
class TripleValuesSensor;

class DeviceManager {
public:
  static DeviceManager *instance();
  static void cleanup();

  const std::vector<Device *> &devices() const { return mDevices; }
  Device *findDeviceFromTag(WbDeviceTag tag) const;

  Motor *motor(int at) const { return mMotors[at]; }
  SingleValueSensor *distanceSensor(int at) const { return mDistanceSensors[at]; }
  SingleValueSensor *sharpDistanceSensor(int at) const { return mSharpDistanceSensors[at]; }
  SingleValueSensor *lightSensor(int at) const { return mLightSensors[at]; }
  SingleValueSensor *positionSensor(int at) const { return mPositionSensors[at]; }
  TripleValuesSensor *accelerometer() const { return mAccelerometer; }
  TripleValuesSensor *gyroscope() const { return mGyro; }
  TripleValuesSensor *magnetometerXY() const { return mMagXY; }
  TripleValuesSensor *magnetometerZ() const { return mMagZ; }

  void apply(int simulationTime);

private:
  static DeviceManager *cInstance;

  DeviceManager();
  DeviceManager(const DeviceManager &);             // non constructor-copyable
  DeviceManager &operator=(const DeviceManager &);  // non copyable
  ~DeviceManager();

  void clear();

  std::vector<Device *> mDevices;
  Motor *mMotors[2];
  SingleValueSensor *mDistanceSensors[8];       // 8 sonar distance sensors
  SingleValueSensor *mSharpDistanceSensors[8];  // 8 sharp distance sensors
  SingleValueSensor *mLightSensors[8];
  SingleValueSensor *mPositionSensors[2];
  TripleValuesSensor *mAccelerometer;
  TripleValuesSensor *mGyro;
  TripleValuesSensor *mMagXY;
  TripleValuesSensor *mMagZ;
};

#endif
