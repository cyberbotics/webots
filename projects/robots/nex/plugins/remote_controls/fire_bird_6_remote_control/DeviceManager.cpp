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

#include "DeviceManager.hpp"

#include "Device.hpp"
#include "Motor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <webots/device.h>
#include <webots/robot.h>

#include <cstdio>

using namespace std;

DeviceManager *DeviceManager::cInstance = NULL;

DeviceManager *DeviceManager::instance() {
  if (!cInstance)
    cInstance = new DeviceManager();
  return cInstance;
}

void DeviceManager::cleanup() {
  if (cInstance) {
    delete cInstance;
    cInstance = NULL;
  }
}

DeviceManager::DeviceManager() {
  clear();

  mMagZ = new TripleValuesSensor(wb_robot_get_device("compassZ_01"), 0);
  mDevices.push_back(mMagZ);
  mMagXY = new TripleValuesSensor(wb_robot_get_device("compassXY_01"), 0);
  mDevices.push_back(mMagXY);

  mGyro = new TripleValuesSensor(wb_robot_get_device("gyro_01"), 0);
  mDevices.push_back(mGyro);

  mAccelerometer = new TripleValuesSensor(wb_robot_get_device("accelerometer_01"), 0);
  mDevices.push_back(mAccelerometer);

  for (int i = 0; i < 8; i++) {
    char name[4] = "ps0";
    name[2] += i;
    mDistanceSensors[i] = new SingleValueSensor(wb_robot_get_device(name), i);
    mDevices.push_back(mDistanceSensors[i]);
  }

  for (int i = 0; i < 8; i++) {
    char name[9] = "sharp_00";
    name[7] += i;
    mSharpDistanceSensors[i] = new SingleValueSensor(wb_robot_get_device(name), i);
    mDevices.push_back(mSharpDistanceSensors[i]);
  }

  mMotors[0] = new Motor(wb_robot_get_device("left wheel motor"), 0);
  mDevices.push_back(mMotors[0]);
  mMotors[1] = new Motor(wb_robot_get_device("right wheel motor"), 1);
  mDevices.push_back(mMotors[1]);
  mPositionSensors[0] = new SingleValueSensor(wb_robot_get_device("left wheel sensor"), 0);
  mDevices.push_back(mPositionSensors[0]);
  mPositionSensors[1] = new SingleValueSensor(wb_robot_get_device("right wheel sensor"), 1);
  mDevices.push_back(mPositionSensors[1]);
}

DeviceManager::~DeviceManager() {
  clear();
}

Device *DeviceManager::findDeviceFromTag(WbDeviceTag tag) const {
  vector<Device *>::const_iterator it;
  for (it = mDevices.begin(); it < mDevices.end(); ++it) {
    Device *d = *it;
    if (d->tag() == tag)
      return d;
  }
  return NULL;
}

void DeviceManager::clear() {
  vector<Device *>::const_iterator it;
  for (it = mDevices.begin(); it < mDevices.end(); ++it)
    delete *it;

  mDevices.clear();

  mAccelerometer = NULL;
  mGyro = NULL;
  mMagXY = NULL;
  mMagZ = NULL;

  for (int i = 0; i < 8; i++) {
    mDistanceSensors[i] = NULL;
    mLightSensors[i] = NULL;
  }
  for (int i = 0; i < 2; i++) {
    mMotors[i] = NULL;
    mPositionSensors[i] = NULL;
  }
}

void DeviceManager::apply(int simulationTime) {
  vector<Device *>::const_iterator it;

  // check if some sensors need to be requested
  for (it = mDevices.begin(); it < mDevices.end(); ++it) {
    Device *d = *it;
    Sensor *s = dynamic_cast<Sensor *>(d);

    if (s && s->isEnabled() && s->lastRefreshTime() + s->samplingPeriod() <= simulationTime) {
      s->setLastRefreshTime(simulationTime);
      s->setSensorRequested();
    }
  }
}
