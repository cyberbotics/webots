// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "Camera.hpp"
#include "Device.hpp"
#include "Led.hpp"
#include "Motor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <webots/robot.h>

#define NMOTORS 20

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

static const char *positionSensorNames[NMOTORS] = {
  "ShoulderRS" /*ID1 */, "ShoulderLS" /*ID2 */, "ArmUpperRS" /*ID3 */, "ArmUpperLS" /*ID4 */, "ArmLowerRS" /*ID5 */,
  "ArmLowerLS" /*ID6 */, "PelvYRS" /*ID7 */,    "PelvYLS" /*ID8 */,    "PelvRS" /*ID9 */,     "PelvLS" /*ID10*/,
  "LegUpperRS" /*ID11*/, "LegUpperLS" /*ID12*/, "LegLowerRS" /*ID13*/, "LegLowerLS" /*ID14*/, "AnkleRS" /*ID15*/,
  "AnkleLS" /*ID16*/,    "FootRS" /*ID17*/,     "FootLS" /*ID18*/,     "NeckS" /*ID19*/,      "HeadS" /*ID20*/
};

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

  mAccelerometer = new TripleValuesSensor(wb_robot_get_device("Accelerometer"), 0);
  mDevices.push_back(mAccelerometer);
  mGyro = new TripleValuesSensor(wb_robot_get_device("Gyro"), 0);
  mDevices.push_back(mGyro);

  mCamera = new CameraR(wb_robot_get_device("Camera"));
  mDevices.push_back(mCamera);

  mLeds[0] = new Led(wb_robot_get_device("EyeLed"), 0);
  mLeds[1] = new Led(wb_robot_get_device("HeadLed"), 1);
  mLeds[2] = new Led(wb_robot_get_device("BackLedRed"), 2);
  mLeds[3] = new Led(wb_robot_get_device("BackLedGreen"), 3);
  mLeds[4] = new Led(wb_robot_get_device("BackLedBlue"), 4);
  for (int i = 0; i < 5; i++) {
    mDevices.push_back(mLeds[i]);
  }

  for (int i = 0; i < 20; i++) {
    mMotors[i] = new MotorR(wb_robot_get_device(motorNames[i]), i);
    mDevices.push_back(mMotors[i]);

    mPositionSensor[i] = new SingleValueSensor(wb_robot_get_device(positionSensorNames[i]), i);
    mDevices.push_back(mPositionSensor[i]);
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
  mGyro = NULL;

  for (int i = 0; i < 5; i++)
    mLeds[i] = NULL;

  for (int i = 0; i < 20; i++) {
    mMotors[i] = NULL;
    mPositionSensor[i] = NULL;
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
