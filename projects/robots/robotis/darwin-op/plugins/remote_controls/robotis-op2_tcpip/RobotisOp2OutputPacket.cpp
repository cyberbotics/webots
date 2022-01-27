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

#include "RobotisOp2OutputPacket.hpp"

#include "Camera.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "Led.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <webots/camera.h>

using namespace std;

RobotisOp2OutputPacket::RobotisOp2OutputPacket() :
  Packet(50000),
  mAccelerometerRequested(false),
  mGyroRequested(false),
  mCameraRequested(false) {
  for (int c = 0; c < 20; c++) {
    mPositionSensorRequested[c] = false;
    mMotorTorqueFeedback[c] = false;
  }
}

RobotisOp2OutputPacket::~RobotisOp2OutputPacket() {
}

void RobotisOp2OutputPacket::clear() {
  Packet::clear();
  mAccelerometerRequested = false;
  mGyroRequested = false;
  mCameraRequested = false;
  for (int c = 0; c < 20; c++) {
    mPositionSensorRequested[c] = false;
    mMotorTorqueFeedback[c] = false;
  }
}

void RobotisOp2OutputPacket::apply(int simulationTime) {
  append("W", 1);
  short int s = 0;
  append((char *)&s, 2);  // the total size of the packet will be stored here
  // ---
  // Sensors
  // ---

  // the order of the sensors should match with RobotisOp2InputPacket::decode()

  // accelerometer management
  TripleValuesSensor *accelerometer = DeviceManager::instance()->accelerometer();
  if (accelerometer->isSensorRequested()) {
    mAccelerometerRequested = true;
    append("A", 1);
  }

  // gyro management
  TripleValuesSensor *gyro = DeviceManager::instance()->gyro();
  if (gyro->isSensorRequested()) {
    mGyroRequested = true;
    append("G", 1);
  }

  // camera management
  // it's better to put the camera at the end in case of
  // retrieval after transmission troubles
  CameraR *camera = DeviceManager::instance()->camera();
  if (camera->isSensorRequested()) {
    mCameraRequested = true;
    append("C", 1);
  }

  // ---
  // Actuators
  // ---
  unsigned char c;
  // send the led commands if required
  for (int i = 0; i < 5; i++) {
    Led *led = DeviceManager::instance()->led(i);
    if (led->isLedRequested()) {
      append("L", 1);
      c = (led->index()) & 0xFF;
      append(&c, 1);
      c = ((led->state()) >> 16) & 0xFF;
      append(&c, 1);
      c = ((led->state()) >> 8) & 0xFF;
      append(&c, 1);
      c = (led->state()) & 0xFF;
      append(&c, 1);
      led->resetLedRequested();
    }
  }

  // Motors management
  for (int i = 0; i < 20; i++) {
    MotorR *motor = DeviceManager::instance()->motor(i);
    if (motor->isMotorRequested()) {
      append("S", 1);
      c = (motor->index()) & 0xFF;
      append(&c, 1);

      // Position
      if (motor->isPositionRequested()) {
        append("p", 1);
        int value = (int)((motor->position() * 2048) / M_PI);
        appendInt(value);
        motor->resetPositionRequested();
      }
      // Velocity
      if (motor->isVelocityRequested()) {
        append("v", 1);
        int value = (int)((motor->velocity() * 30) / (0.114 * M_PI));
        appendInt(value);
        motor->resetVelocityRequested();
      }
      // Acceleration
      if (motor->isAccelerationRequested()) {
        append("a", 1);
        int value = (int)(motor->acceleration() * 100000);
        appendInt(value);
        motor->resetAccelerationRequested();
      }
      // MotorForce
      if (motor->isMotorForceRequested()) {
        append("m", 1);
        int value = (int)((motor->motorForce() * 1023) / 2.5);
        appendInt(value);
        motor->resetAvailableTorqueRequested();
      }
      // ControlPID
      if (motor->isControlPIDRequested()) {
        append("c", 1);
        int pValue = (int)(motor->controlP() * 1000);
        int iValue = (int)(motor->controlI() * 1000);
        int dValue = (int)(motor->controlD() * 1000);
        appendInt(pValue);
        appendInt(iValue);
        appendInt(dValue);
        // TODO (fabien): why doing this 1000 multiplication (and division at the other side) while
        //                simply creating an appendDouble() function would be much more elegant?
        motor->resetControlPIDRequested();
      }
      // Force
      if (motor->isForceRequested()) {
        append("f", 1);
        int value = (int)((motor->torque() * 1023) / 2.5);
        appendInt(value);
        motor->resetTorqueRequested();
      }
      motor->resetMotorRequested();
    }
  }

  for (int i = 0; i < 20; i++) {
    SingleValueSensor *positionSensor = DeviceManager::instance()->positionSensor(i);
    if (positionSensor->isSensorRequested()) {
      mPositionSensorRequested[i] = true;
      append("P", 1);
      c = (positionSensor->index()) & 0xFF;
      append(&c, 1);
    }
  }

  for (int i = 0; i < 20; i++) {
    MotorR *motor = DeviceManager::instance()->motor(i);
    if (motor->isSensorRequested()) {
      mMotorTorqueFeedback[i] = true;
      append("F", 1);
      c = (motor->index()) & 0xFF;
      append(&c, 1);
    }
  }
  // This is required to end the packet
  // even if the size is correct
  append("", 1);
  s = size();
  char sc[2];
  sc[0] = (unsigned char)(s % 256);
  sc[1] = (unsigned char)(s / 256);
  mData[1] = sc[0];  // write the size
  mData[2] = sc[1];  // of the packet
}
