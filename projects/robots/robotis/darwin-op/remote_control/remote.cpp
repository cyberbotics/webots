// Copyright 1996-2023 Cyberbotics Ltd.
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

#include "remote.hpp"
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace std;

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

Remote::Remote() : Robot() {
  mTimeStep = getBasicTimeStep();

  mAccelerometer = getAccelerometer("Accelerometer");
  mGyro = getGyro("Gyro");
  mCamera = getCamera("Camera");
  mCamera->enable(16);

  mEyeLed = getLED("EyeLed");
  mHeadLed = getLED("HeadLed");
  mBackLedRed = getLED("BackLedRed");
  mBackLedGreen = getLED("BackLedGreen");
  mBackLedBlue = getLED("BackLedBlue");

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mPositionSensor[i] = getPositionSensor(positionSensorNames[i]);
  }
}

Remote::~Remote() {
}

void Remote::myStep() {
  int ret = step(1);  // we want the step the shorter as possible (Webots is in charge of the timing management)
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Remote::remoteStep() {
  myStep();
}

const double *Remote::getRemoteAccelerometer() const {
  return mAccelerometer->getValues();
}

const double *Remote::getRemoteGyro() const {
  return mGyro->getValues();
}

const unsigned char *Remote::getRemoteImage() const {
  return mCamera->getImage();
}

void Remote::setRemoteLED(int index, int value) {
  switch (index) {
    case 0:
      mEyeLed->set(value);
      break;
    case 1:
      mHeadLed->set(value);
      break;
    case 2:
      mBackLedRed->set(value);
      break;
    case 3:
      mBackLedGreen->set(value);
      break;
    case 4:
      mBackLedBlue->set(value);
      break;
  }
}

void Remote::setRemoteMotorPosition(int index, int value) {
  double position = (M_PI * value) / 2048;
  mMotors[index]->setPosition(position);
}

void Remote::setRemoteMotorVelocity(int index, int value) {
  double velocity = (value * 0.114 * M_PI) / 30;
  mMotors[index]->setVelocity(velocity);
}

void Remote::setRemoteMotorAcceleration(int index, int value) {
  double acceleration = value / 100000;
  mMotors[index]->setAcceleration(acceleration);
}

void Remote::setRemoteMotorAvailableTorque(int index, int value) {
  double torque = (value * 2.5) / 1023;
  mMotors[index]->setAvailableTorque(torque);
}

void Remote::setRemoteMotorTorque(int index, int value) {
  double torque = (value * 2.5) / 1023;
  mMotors[index]->setTorque(torque);
}

void Remote::setRemoteMotorControlPID(int index, int p, int i, int d) {
  mMotors[index]->setControlPID(0.001 * p, 0.001 * i, 0.001 * d);
}

double Remote::getRemotePositionSensor(int index) {
  return (mPositionSensor[index]->getValue() * 10000);
}

double Remote::getRemoteMotorTorque(int index) {
  return (mMotors[index]->getTorqueFeedback() * 10000);
}

double Remote::getRemoteTime() const {
  return this->getTime();
}
