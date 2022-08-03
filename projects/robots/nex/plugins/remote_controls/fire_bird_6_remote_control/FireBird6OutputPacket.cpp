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

#include "FireBird6OutputPacket.hpp"

#include "Device.hpp"
#include "DeviceManager.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

FireBird6OutputPacket::FireBird6OutputPacket() :
  Packet(64),
  mAnswerSize(0),
  mDistanceSensorRequested(false),
  mSharpDistanceSensorRequested(false),
  mLightSensorRequested(false),
  mAccelerometerRequested(false),
  mGyroRequested(false),
  mMagnetometerRequested(false),
  mEncoderRequested(false) {
}

FireBird6OutputPacket::~FireBird6OutputPacket() {
}

void FireBird6OutputPacket::clear() {
  Packet::clear();

  mAnswerSize = 0;

  mDistanceSensorRequested = false;
  mSharpDistanceSensorRequested = false;
  mLightSensorRequested = false;
  mAccelerometerRequested = false;
  mGyroRequested = false;
  mMagnetometerRequested = false;
  mEncoderRequested = false;
}

void FireBird6OutputPacket::apply(int simulationTime) {
  // ---
  // Actuators
  // ---

  // send the motor velocity commands if required
  Motor *leftMotor = DeviceManager::instance()->motor(0);
  Motor *rightMotor = DeviceManager::instance()->motor(1);
  if (leftMotor->isVelocityRequested() || rightMotor->isVelocityRequested()) {
    int leftSpeed = (char)(-10.66 * leftMotor->velocity() + 128);
    if (leftSpeed > 255)
      leftSpeed = 255;
    int rightSpeed = (char)(-10.66 * rightMotor->velocity() + 128);
    if (rightSpeed > 255)
      rightSpeed = 255;
    append(static_cast<char>('N'));
    append(static_cast<char>('E'));
    append(static_cast<char>('X'));
    append(static_cast<char>(0x95));
    append(static_cast<char>(leftSpeed));
    append(static_cast<char>('N'));
    append(static_cast<char>('E'));
    append(static_cast<char>('X'));
    append(static_cast<char>(0x96));
    append(static_cast<char>(rightSpeed));

    append(static_cast<char>('N'));
    append(static_cast<char>('E'));
    append(static_cast<char>('X'));
    append(static_cast<char>(0x94));
    append(static_cast<char>(0x01));

    leftMotor->resetVelocityRequested();
    rightMotor->resetVelocityRequested();
  }

  // request all sensors
  append(static_cast<char>('N'));
  append(static_cast<char>('E'));
  append(static_cast<char>('X'));
  append(static_cast<char>(0x24));
  append(static_cast<char>(0x00));

  mAnswerSize += 54;

  // manage the position sensor
  if (DeviceManager::instance()->positionSensor(0)->isSensorRequested() ||
      DeviceManager::instance()->positionSensor(1)->isSensorRequested()) {
    mEncoderRequested = true;

    append(static_cast<char>('N'));
    append(static_cast<char>('E'));
    append(static_cast<char>('X'));
    append(static_cast<char>(0x92));
    append(static_cast<char>(0x00));

    append(static_cast<char>('N'));
    append(static_cast<char>('E'));
    append(static_cast<char>('X'));
    append(static_cast<char>(0x93));
    append(static_cast<char>(0x00));

    mAnswerSize += 18 * sizeof(char);
  }

  // print();

  mDistanceSensorRequested = true;
  mSharpDistanceSensorRequested = true;
  mAccelerometerRequested = true;
  mGyroRequested = true;
  mMagnetometerRequested = true;
}
