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

#include <webots/PositionSensor.hpp>
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

std::map<const std::string, int> PositionSensor::mNamesToIDs;
std::map<const std::string, int> PositionSensor::mNamesToInitPos;

PositionSensor::PositionSensor(const std::string &name) : Device(name) {
  initStaticMap();
  mPresentPosition = mNamesToInitPos[getName()];
  mFeedback = 0;
}

PositionSensor::~PositionSensor() {
}

void PositionSensor::initStaticMap() {
  static bool firstCall = true;
  if (firstCall) {
    firstCall = false;

    mNamesToIDs["ShoulderRS"] = JointData::ID_R_SHOULDER_PITCH;
    mNamesToIDs["ShoulderLS"] = JointData::ID_L_SHOULDER_PITCH;
    mNamesToIDs["ArmUpperRS"] = JointData::ID_R_SHOULDER_ROLL;
    mNamesToIDs["ArmUpperLS"] = JointData::ID_L_SHOULDER_ROLL;
    mNamesToIDs["ArmLowerRS"] = JointData::ID_R_ELBOW;
    mNamesToIDs["ArmLowerLS"] = JointData::ID_L_ELBOW;
    mNamesToIDs["PelvYRS"] = JointData::ID_R_HIP_YAW;
    mNamesToIDs["PelvYLS"] = JointData::ID_L_HIP_YAW;
    mNamesToIDs["PelvRS"] = JointData::ID_R_HIP_ROLL;
    mNamesToIDs["PelvLS"] = JointData::ID_L_HIP_ROLL;
    mNamesToIDs["LegUpperRS"] = JointData::ID_R_HIP_PITCH;
    mNamesToIDs["LegUpperLS"] = JointData::ID_L_HIP_PITCH;
    mNamesToIDs["LegLowerRS"] = JointData::ID_R_KNEE;
    mNamesToIDs["LegLowerLS"] = JointData::ID_L_KNEE;
    mNamesToIDs["AnkleRS"] = JointData::ID_R_ANKLE_PITCH;
    mNamesToIDs["AnkleLS"] = JointData::ID_L_ANKLE_PITCH;
    mNamesToIDs["FootRS"] = JointData::ID_R_ANKLE_ROLL;
    mNamesToIDs["FootLS"] = JointData::ID_L_ANKLE_ROLL;
    mNamesToIDs["NeckS"] = JointData::ID_HEAD_PAN;
    mNamesToIDs["HeadS"] = JointData::ID_HEAD_TILT;

    mNamesToInitPos["ShoulderRS"] = 1500;
    mNamesToInitPos["ShoulderLS"] = 2517;
    mNamesToInitPos["ArmUpperRS"] = 1834;
    mNamesToInitPos["ArmUpperLS"] = 2283;
    mNamesToInitPos["ArmLowerRS"] = 2380;
    mNamesToInitPos["ArmLowerLS"] = 1710;
    mNamesToInitPos["PelvYRS"] = 2043;
    mNamesToInitPos["PelvYLS"] = 2033;
    mNamesToInitPos["PelvRS"] = 2057;
    mNamesToInitPos["PelvLS"] = 2043;
    mNamesToInitPos["LegUpperRS"] = 1277;
    mNamesToInitPos["LegUpperLS"] = 2797;
    mNamesToInitPos["LegLowerRS"] = 3513;
    mNamesToInitPos["LegLowerLS"] = 571;
    mNamesToInitPos["AnkleRS"] = 2843;
    mNamesToInitPos["AnkleLS"] = 1240;
    mNamesToInitPos["FootRS"] = 2077;
    mNamesToInitPos["FootLS"] = 2037;
    mNamesToInitPos["NeckS"] = 2050;
    mNamesToInitPos["HeadS"] = 2173;
  }
}

void PositionSensor::enable(int samplingPeriod) {
  mFeedback = samplingPeriod;
}

void PositionSensor::disable() {
  mFeedback = 0;
}

int PositionSensor::getSamplingPeriod() const {
  return mFeedback;
}

double PositionSensor::getValue() const {
  double position = 0;
  position = (MX28::Value2Angle(mPresentPosition) * M_PI) / 180;
  return position;
}

void PositionSensor::setPresentPosition(int position) {
  mPresentPosition = position;
}

int PositionSensor::getType() const {
  return ROTATIONAL;
}
