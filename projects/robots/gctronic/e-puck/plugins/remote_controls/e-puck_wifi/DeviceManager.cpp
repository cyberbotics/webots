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

#include "DeviceManager.hpp"

#include "Camera.hpp"
#include "Device.hpp"
#include "Led.hpp"
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

  mCamera = new Camera(wb_robot_get_device("camera"));
  mDevices.push_back(mCamera);

  mAccelerometer = new TripleValuesSensor(wb_robot_get_device("accelerometer"), 0);
  mDevices.push_back(mAccelerometer);

  for (int i = 0; i < 10; i++) {
    char name[5] = "led0";
    name[3] += i;
    mLeds[i] = new Led(wb_robot_get_device(name), i);
    mDevices.push_back(mLeds[i]);
  }

  for (int i = 0; i < 8; i++) {
    char name[4] = "ps0";
    name[2] += i;
    mDistanceSensors[i] = new SingleValueSensor(wb_robot_get_device(name), i);
    mDevices.push_back(mDistanceSensors[i]);
  }

  for (int i = 0; i < 8; i++) {
    char name[4] = "ls0";
    name[2] += i;
    mLightSensors[i] = new SingleValueSensor(wb_robot_get_device(name), i);
    mDevices.push_back(mLightSensors[i]);
  }

  mTofSensor = new SingleValueSensor(wb_robot_get_device("tof"), 0);
  mDevices.push_back(mTofSensor);

  mMotors[0] = new Motor(wb_robot_get_device("left wheel motor"), 0);
  mDevices.push_back(mMotors[0]);
  mMotors[1] = new Motor(wb_robot_get_device("right wheel motor"), 1);
  mDevices.push_back(mMotors[1]);
  mPositionSensors[0] = new SingleValueSensor(wb_robot_get_device("left wheel sensor"), 0);
  mDevices.push_back(mPositionSensors[0]);
  mPositionSensors[1] = new SingleValueSensor(wb_robot_get_device("right wheel sensor"), 1);
  mDevices.push_back(mPositionSensors[1]);

  // init optional devices
  for (int i = 0; i < 3; i++)
    mGroundSensors[i] = NULL;

  WbNodeType deviceType;
  int groundSensorIndex, matchedItems;
  int numberOfDevices = wb_robot_get_number_of_devices();
  for (int index = 0; index < numberOfDevices; index++) {
    WbDeviceTag tag = wb_robot_get_device_by_index(index);
    deviceType = wb_device_get_node_type(tag);

    if (deviceType == WB_NODE_DISTANCE_SENSOR) {
      const char *deviceName = wb_device_get_name(tag);
      matchedItems = sscanf(deviceName, "gs%d", &groundSensorIndex);
      if (matchedItems > 0) {
        // init ground sensors
        if (groundSensorIndex < 3 && !mGroundSensors[groundSensorIndex]) {
          mGroundSensors[groundSensorIndex] = new SingleValueSensor(tag, groundSensorIndex);
          mDevices.push_back(mGroundSensors[groundSensorIndex]);
        }
      }
    }
  }
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

  mCamera = NULL;
  mAccelerometer = NULL;

  for (int i = 0; i < 10; i++)
    mLeds[i] = NULL;
  for (int i = 0; i < 3; i++)
    mGroundSensors[i] = NULL;
  for (int i = 0; i < 8; i++) {
    mDistanceSensors[i] = NULL;
    mLightSensors[i] = NULL;
  }
  for (int i = 0; i < 2; i++) {
    mMotors[i] = NULL;
    mPositionSensors[i] = NULL;
  }
  mTofSensor = NULL;
}

const void DeviceManager::apply(int simulationTime) const {
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
