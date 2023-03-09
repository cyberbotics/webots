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

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <CM730.h>
#include <JointData.h>
#include <MX28.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace webots;
using namespace Robot;

std::map<const std::string, int> Motor::mNamesToIDs;
std::map<const std::string, int> Motor::mNamesToLimUp;
std::map<const std::string, int> Motor::mNamesToLimDown;
std::map<const std::string, int> Motor::mNamesToInitPos;

template<typename T> int sgn(T val) {
  if (val >= 0)
    return 1;
  else
    return -1;
}

Motor::Motor(const std::string &name) : Device(name) {
  initStaticMap();
  mAcceleration = -1;
  mMaxVelocity = 10;
  mActualVelocity = 0;
  mGoalPosition = mNamesToInitPos[getName()];
  mTorqueEnable = 1;  // Yes
  mPGain = 32;
  mMovingSpeed = 1023;  // Max speed
  mTorqueLimit = 1023;  // Max torque
  mPresentSpeed = 0;
  mPresentLoad = 0;
  mTorqueFeedback = 0;
}

Motor::~Motor() {
}

void Motor::initStaticMap() {
  static bool firstCall = true;
  if (firstCall) {
    firstCall = false;

    mNamesToIDs["ShoulderR"] = JointData::ID_R_SHOULDER_PITCH;
    mNamesToIDs["ShoulderL"] = JointData::ID_L_SHOULDER_PITCH;
    mNamesToIDs["ArmUpperR"] = JointData::ID_R_SHOULDER_ROLL;
    mNamesToIDs["ArmUpperL"] = JointData::ID_L_SHOULDER_ROLL;
    mNamesToIDs["ArmLowerR"] = JointData::ID_R_ELBOW;
    mNamesToIDs["ArmLowerL"] = JointData::ID_L_ELBOW;
    mNamesToIDs["PelvYR"] = JointData::ID_R_HIP_YAW;
    mNamesToIDs["PelvYL"] = JointData::ID_L_HIP_YAW;
    mNamesToIDs["PelvR"] = JointData::ID_R_HIP_ROLL;
    mNamesToIDs["PelvL"] = JointData::ID_L_HIP_ROLL;
    mNamesToIDs["LegUpperR"] = JointData::ID_R_HIP_PITCH;
    mNamesToIDs["LegUpperL"] = JointData::ID_L_HIP_PITCH;
    mNamesToIDs["LegLowerR"] = JointData::ID_R_KNEE;
    mNamesToIDs["LegLowerL"] = JointData::ID_L_KNEE;
    mNamesToIDs["AnkleR"] = JointData::ID_R_ANKLE_PITCH;
    mNamesToIDs["AnkleL"] = JointData::ID_L_ANKLE_PITCH;
    mNamesToIDs["FootR"] = JointData::ID_R_ANKLE_ROLL;
    mNamesToIDs["FootL"] = JointData::ID_L_ANKLE_ROLL;
    mNamesToIDs["Neck"] = JointData::ID_HEAD_PAN;
    mNamesToIDs["Head"] = JointData::ID_HEAD_TILT;

    mNamesToLimUp["ShoulderR"] = 4095;
    mNamesToLimUp["ShoulderL"] = 3095;
    mNamesToLimUp["ArmUpperR"] = 3548;
    mNamesToLimUp["ArmUpperL"] = 2549;
    mNamesToLimUp["ArmLowerR"] = 2804;
    mNamesToLimUp["ArmLowerL"] = 3109;
    mNamesToLimUp["PelvYR"] = 2480;
    mNamesToLimUp["PelvYL"] = 3680;
    mNamesToLimUp["PelvR"] = 2708;
    mNamesToLimUp["PelvL"] = 2650;
    mNamesToLimUp["LegUpperR"] = 2340;
    mNamesToLimUp["LegUpperL"] = 3145;
    mNamesToLimUp["LegLowerR"] = 3513;
    mNamesToLimUp["LegLowerL"] = 2067;
    mNamesToLimUp["AnkleR"] = 2947;
    mNamesToLimUp["AnkleL"] = 2845;
    mNamesToLimUp["FootR"] = 2728;
    mNamesToLimUp["FootL"] = 2435;
    mNamesToLimUp["Neck"] = 3150;
    mNamesToLimUp["Head"] = 2660;

    mNamesToLimDown["ShoulderR"] = 191;
    mNamesToLimDown["ShoulderL"] = 0;
    mNamesToLimDown["ArmUpperR"] = 1605;
    mNamesToLimDown["ArmUpperL"] = 583;
    mNamesToLimDown["ArmLowerR"] = 970;
    mNamesToLimDown["ArmLowerL"] = 1278;
    mNamesToLimDown["PelvYR"] = 470;
    mNamesToLimDown["PelvYL"] = 1600;
    mNamesToLimDown["PelvR"] = 1390;
    mNamesToLimDown["PelvL"] = 1400;
    mNamesToLimDown["LegUpperR"] = 889;
    mNamesToLimDown["LegUpperL"] = 1724;
    mNamesToLimDown["LegLowerR"] = 2038;
    mNamesToLimDown["LegLowerL"] = 571;
    mNamesToLimDown["AnkleR"] = 1239;
    mNamesToLimDown["AnkleL"] = 1142;
    mNamesToLimDown["FootR"] = 1603;
    mNamesToLimDown["FootL"] = 1385;
    mNamesToLimDown["Neck"] = 790;
    mNamesToLimDown["Head"] = 1815;

    mNamesToInitPos["ShoulderR"] = 1500;
    mNamesToInitPos["ShoulderL"] = 2517;
    mNamesToInitPos["ArmUpperR"] = 1834;
    mNamesToInitPos["ArmUpperL"] = 2283;
    mNamesToInitPos["ArmLowerR"] = 2380;
    mNamesToInitPos["ArmLowerL"] = 1710;
    mNamesToInitPos["PelvYR"] = 2043;
    mNamesToInitPos["PelvYL"] = 2033;
    mNamesToInitPos["PelvR"] = 2057;
    mNamesToInitPos["PelvL"] = 2043;
    mNamesToInitPos["LegUpperR"] = 1277;
    mNamesToInitPos["LegUpperL"] = 2797;
    mNamesToInitPos["LegLowerR"] = 3513;
    mNamesToInitPos["LegLowerL"] = 571;
    mNamesToInitPos["AnkleR"] = 2843;
    mNamesToInitPos["AnkleL"] = 1240;
    mNamesToInitPos["FootR"] = 2077;
    mNamesToInitPos["FootL"] = 2037;
    mNamesToInitPos["Neck"] = 2050;
    mNamesToInitPos["Head"] = 2173;
  }
}

void Motor::setAcceleration(double acceleration) {
  mAcceleration = acceleration;
  if (acceleration == -1)  // No Aceleration limitation -> restore previous Velocity limit
    setVelocity(mMaxVelocity);
}

void Motor::setVelocity(double vel) {
  int value = fabs((vel * 30 / M_PI) / 0.114);  // Need to be verified
  if (value > 1023)
    value = 1023;
  else if (value == 0)  // Because 0 means max Velocity for the dynamixel
    value = 1;
  mMovingSpeed = value;
  if (mAcceleration == -1)
    mMaxVelocity = vel;
}

void Motor::setTorque(double torque) {
  CM730 *cm730 = Robot::getInstance()->getCM730();
  if (torque == 0)
    cm730->WriteWord(mNamesToIDs[getName()], MX28::P_TORQUE_ENABLE, 0, 0);
  else {
    this->setAvailableTorque(fabs(torque));
    int firm_ver = 0;
    if (cm730->ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0) != CM730::SUCCESS)
      cerr << "Can't read firmware version from Dynamixel ID " << JointData::ID_HEAD_PAN << endl;
    else if (27 <= firm_ver) {
      if (torque > 0)
        mGoalPosition = mNamesToLimDown[getName()];
      else
        mGoalPosition = mNamesToLimUp[getName()];
    } else
      cerr << "Motor::setTorque not available for this version of Dynamixel firmware, please update it." << endl;
  }
}

void Motor::setAvailableTorque(double availableTorque) {
  CM730 *cm730 = Robot::getInstance()->getCM730();
  if (availableTorque > 2.5) {
    mTorqueEnable = 1;
    mTorqueLimit = 1023;
  } else if (availableTorque > 0) {
    mTorqueEnable = 1;
    mTorqueLimit = (availableTorque / 2.5) * 1023;
  } else {
    mTorqueLimit = 0;
    mTorqueEnable = 0;
    cm730->WriteWord(mNamesToIDs[getName()], MX28::P_TORQUE_ENABLE, 0, 0);
  }

  // don't override the motor alarm
  if (mTorqueLimit <= 0)
    mTorqueLimit = 1;
}

void Motor::setControlPID(double p, double i, double d) {
  if (p < 3)
    cout << "WARNING : A small value of P can cause differences between simulation and reality." << endl;

  if (p >= 0) {
    int value = p * 8;  // TODO: Seems to be good, but has to be verified
    mPGain = value;
  }
}

void Motor::enableTorqueFeedback(int samplingPeriod) {
  mTorqueFeedback = samplingPeriod;
}

void Motor::disableTorqueFeedback() {
  mTorqueFeedback = 0;
}

int Motor::getTorqueFeedbackSamplingPeriod() const {
  return mTorqueFeedback;
}

double Motor::getTorqueFeedback() const {
  double torque = 0;
  if (mPresentLoad < 1024)
    torque = -2.5 * mPresentLoad / 1023.0;
  else
    torque = 2.5 * (mPresentLoad - 1023) / 1023.0;

  return torque;
}

void Motor::setPosition(double position) {
  int value = MX28::Angle2Value(position * 180.0 / M_PI);

  if (value >= 0 && value <= MX28::MAX_VALUE) {
    //       Self-Collision Avoidance      //
    // Work only with a resolution of 4096 //
    if (value > mNamesToLimUp[getName()])
      value = mNamesToLimUp[getName()];
    else if (value < mNamesToLimDown[getName()])
      value = mNamesToLimDown[getName()];

    mGoalPosition = value;
  }
}

void Motor::updateSpeed(int samplingPeriod) {
  if (mAcceleration != -1) {
    double speed = getSpeed();
    int sens = sgn(speed);

    // Direction of rotation changed //
    //     -> reset of velocity      //
    if (sgn(mActualVelocity) != sens) {
      mActualVelocity = 0;
      setVelocity(0);
    }

    if (speed == 0)
      mActualVelocity = 0;

    mActualVelocity = sens * (fabs(mActualVelocity) + samplingPeriod * mAcceleration / 1000);
    setVelocity(mActualVelocity);
  }
}

double Motor::getSpeed() const {
  int value = 0;
  double speed = 0;

  if (mPresentSpeed > 1023)
    value = -(mPresentSpeed - 1023);
  else
    value = mPresentSpeed;
  speed = (value * M_PI / 30) * 0.114;

  return speed;
}

int Motor::getGoalPosition() const {
  return mGoalPosition;
}

int Motor::getTorqueEnable() const {
  return mTorqueEnable;
}

int Motor::getPGain() const {
  return mPGain;
}

int Motor::getMovingSpeed() const {
  return mMovingSpeed;
}

int Motor::getTorqueLimit() const {
  return mTorqueLimit;
}

void Motor::setPresentSpeed(int speed) {
  mPresentSpeed = speed;
}

void Motor::setPresentLoad(int load) {
  mPresentLoad = load;
}

double Motor::getTargetPosition() const {
  return mGoalPosition;
}

double Motor::getMinPosition() const {
  return (MX28::Value2Angle(mNamesToLimDown[getName()]) * (M_PI / 180.0));
}

double Motor::getMaxPosition() const {
  return (MX28::Value2Angle(mNamesToLimUp[getName()]) * (M_PI / 180.0));
}

int Motor::getType() const {
  return ROTATIONAL;
}
