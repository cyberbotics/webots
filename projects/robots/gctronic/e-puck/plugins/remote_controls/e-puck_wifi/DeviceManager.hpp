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
 * Description:  Class allowing to create or retrieve devices
 */

#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

#include <webots/types.h>

#include <vector>

class Device;
class Camera;
class Motor;
class Led;
class SingleValueSensor;
class TripleValuesSensor;

class DeviceManager {
public:
  static DeviceManager *instance();
  static void cleanup();

  const std::vector<Device *> &devices() const { return mDevices; }
  Device *findDeviceFromTag(WbDeviceTag tag) const;

  Camera *camera() const { return mCamera; }
  Led *led(int at) const { return mLeds[at]; }
  Motor *motor(int at) const { return mMotors[at]; }
  SingleValueSensor *distanceSensor(int at) const { return mDistanceSensors[at]; }
  SingleValueSensor *lightSensor(int at) const { return mLightSensors[at]; }
  SingleValueSensor *groundSensor(int at) const { return mGroundSensors[at]; }
  SingleValueSensor *positionSensor(int at) const { return mPositionSensors[at]; }
  TripleValuesSensor *accelerometer() const { return mAccelerometer; }
  SingleValueSensor *tofSensor() const { return mTofSensor; }

  const void apply(int simulationTime) const;

private:
  static DeviceManager *cInstance;

  DeviceManager();
  DeviceManager(const DeviceManager &);             // non constructor-copyable
  DeviceManager &operator=(const DeviceManager &);  // non copyable
  ~DeviceManager();

  void clear();

  std::vector<Device *> mDevices;
  Camera *mCamera;
  Led *mLeds[10];
  Motor *mMotors[2];
  SingleValueSensor *mDistanceSensors[8];
  SingleValueSensor *mLightSensors[8];
  SingleValueSensor *mGroundSensors[3];
  SingleValueSensor *mPositionSensors[2];
  TripleValuesSensor *mAccelerometer;
  SingleValueSensor *mTofSensor;
};

#endif
