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

/*
 * Description:  Abstraction of a motor
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "Device.hpp"
#include "SingleValueSensor.hpp"

class MotorR : public SingleValueSensor {
public:
  // Device Manager is responsible to create/destroy devices
  MotorR(WbDeviceTag tag, int index) :
    SingleValueSensor(tag, index),
    mMotorRequested(false),
    mPosition(0.0),
    mVelocity(0.0),
    mAcceleration(0.0),
    mMotorAvailableTorque(0.0),
    mControlP(0.0),
    mControlD(0.0),
    mControlI(0.0),
    mTorque(0.0),
    mPositionRequested(false),
    mVelocityRequested(false),
    mAccelerationRequested(false),
    mMotorAvailableTorqueRequested(false),
    mControlPIDRequested(false),
    mTorqueRequested(false) {}
  virtual ~MotorR() {}

  bool isMotorRequested() const { return mMotorRequested; }
  void resetMotorRequested() { mMotorRequested = false; }
  void setMotorRequested() { mMotorRequested = true; }

  // Actuators part
  void setPosition(double position) { mPosition = position; }
  void setVelocity(double vel) { mVelocity = vel; }
  void setAcceleration(double acceleration) { mAcceleration = acceleration; }
  void setAvailableTorque(double torque) { mMotorAvailableTorque = torque; }
  void setControlPID(double p, double i, double d) {
    mControlP = p;
    mControlI = i;
    mControlD = d;
  }
  void setTorque(double torque) { mTorque = torque; }

  double position() const { return mPosition; }
  double velocity() const { return mVelocity; }
  double acceleration() const { return mAcceleration; }
  double motorForce() const { return mMotorAvailableTorque; }
  double controlP() const { return mControlP; }
  double controlI() const { return mControlI; }
  double controlD() const { return mControlD; }
  double torque() const { return mTorque; }

  bool isPositionRequested() const { return mPositionRequested; }
  void resetPositionRequested() { mPositionRequested = false; }
  void setPositionRequested() { mPositionRequested = true; }

  bool isVelocityRequested() const { return mVelocityRequested; }
  void resetVelocityRequested() { mVelocityRequested = false; }
  void setVelocityRequested() { mVelocityRequested = true; }

  bool isAccelerationRequested() const { return mAccelerationRequested; }
  void resetAccelerationRequested() { mAccelerationRequested = false; }
  void setAccelerationRequested() { mAccelerationRequested = true; }

  bool isMotorForceRequested() const { return mMotorAvailableTorqueRequested; }
  void resetAvailableTorqueRequested() { mMotorAvailableTorqueRequested = false; }
  void setAvailableTorqueRequested() { mMotorAvailableTorqueRequested = true; }

  bool isControlPIDRequested() const { return mControlPIDRequested; }
  void resetControlPIDRequested() { mControlPIDRequested = false; }
  void setControlPIDRequested() { mControlPIDRequested = true; }

  bool isForceRequested() const { return mTorqueRequested; }
  void resetTorqueRequested() { mTorqueRequested = false; }
  void setTorqueRequested() { mTorqueRequested = true; }

private:
  bool mMotorRequested;

  // Actuators part
  double mPosition;
  double mVelocity;
  double mAcceleration;
  double mMotorAvailableTorque;
  double mControlP, mControlD, mControlI;
  double mTorque;

  bool mPositionRequested;
  bool mVelocityRequested;
  bool mAccelerationRequested;
  bool mMotorAvailableTorqueRequested;
  bool mControlPIDRequested;
  bool mTorqueRequested;
};

#endif
