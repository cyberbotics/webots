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

#include "EPuckOutputPacket.hpp"

#include "Camera.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "Led.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

EPuckOutputPacket::EPuckOutputPacket() :
  Packet(52 * 39 * 2 + 3 + 80),
  mAnswerSize(0),
  mDistanceSensorRequested(false),
  mGroundSensorRequested(false),
  mLightSensorRequested(false),
  mAccelerometerRequested(false),
  mCameraRequested(false),
  mEncoderRequested(false) {
}

EPuckOutputPacket::~EPuckOutputPacket() {
}

void EPuckOutputPacket::clear() {
  Packet::clear();

  mAnswerSize = 0;

  mDistanceSensorRequested = false;
  mGroundSensorRequested = false;
  mLightSensorRequested = false;
  mAccelerometerRequested = false;
  mCameraRequested = false;
  mEncoderRequested = false;
}

void EPuckOutputPacket::apply(int simulationTime) {
  // ---
  // Actuators
  // ---

  // send the motor velocity commands if required
  Motor *leftMotor = DeviceManager::instance()->motor(0);
  Motor *rightMotor = DeviceManager::instance()->motor(1);
  if (leftMotor->isVelocityRequested() || rightMotor->isVelocityRequested()) {
    append(static_cast<char>(-'D'));
    append(static_cast<short>(leftMotor->velocity() / 0.00628));  // 0.00628 = ( 2 * pi) / encoder_resolution
    append(static_cast<short>(rightMotor->velocity() / 0.00628));
    leftMotor->resetVelocityRequested();
    rightMotor->resetVelocityRequested();
  }
  // send the led commands if required
  for (int i = 0; i < 10; i++) {
    Led *led = DeviceManager::instance()->led(i);
    if (led->isLedRequested()) {
      append(static_cast<char>(-'L'));
      append(static_cast<char>(led->index()));
      append(static_cast<char>(led->state()));
      led->resetLedRequested();
    }
  }

  // ---
  // Sensors
  // ---

  // the order of the sensors should match with EPuckInputPacket::decode()

  // accelerometer management
  TripleValuesSensor *accelerometer = DeviceManager::instance()->accelerometer();
  if (accelerometer->isSensorRequested()) {
    mAccelerometerRequested = true;
    append(static_cast<char>(-'a'));
    mAnswerSize += 3 * sizeof(short);
  }

  // manage the optional ground sensors
  // (send the command if at least one ground sensor is required)
  for (int i = 0; i < 3; i++) {
    SingleValueSensor *gs = DeviceManager::instance()->groundSensor(i);
    if (gs && gs->isSensorRequested()) {
      mGroundSensorRequested = true;
      append(static_cast<char>(-'M'));
      mAnswerSize += 3 * sizeof(short);
      break;
    }
  }

  // manage the distance sensors
  // (send the command if at least one distance sensor is required)
  for (int i = 0; i < 8; i++) {
    SingleValueSensor *ds = DeviceManager::instance()->distanceSensor(i);
    if (ds->isSensorRequested()) {
      mDistanceSensorRequested = true;
      append(static_cast<char>(-'N'));
      mAnswerSize += 8 * sizeof(short);
      break;
    }
  }

  // manage the light sensors
  // (send the command if at least one light sensor is required)
  for (int i = 0; i < 8; i++) {
    SingleValueSensor *ls = DeviceManager::instance()->lightSensor(i);
    if (ls->isSensorRequested()) {
      mLightSensorRequested = true;
      append(static_cast<char>(-'O'));
      mAnswerSize += 8 * sizeof(short);
      break;
    }
  }

  // manage the position sensor
  if (DeviceManager::instance()->positionSensor(0)->isSensorRequested() ||
      DeviceManager::instance()->positionSensor(1)->isSensorRequested()) {
    mEncoderRequested = true;
    append(static_cast<char>(-'Q'));
    mAnswerSize += 2 * sizeof(short);
  }

  // camera management
  // it's better to put the camera at the end in case of
  // retrieval after transmission troubles
  Camera *camera = DeviceManager::instance()->camera();
  if (camera->isSensorRequested()) {
    mCameraRequested = true;
    append(static_cast<char>(-'I'));
    mAnswerSize += 3 + 2 * camera->width() * camera->height();
  }

  // This is require to end the packet
  // even if the size is correct
  append('\0');
}
