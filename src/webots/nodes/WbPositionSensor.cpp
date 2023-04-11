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

//
//  WbPositionSensor.cpp
//
#include "WbPositionSensor.hpp"

#include "WbDataStream.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbJoint.hpp"
#include "WbMathsUtilities.hpp"
#include "WbPropeller.hpp"
#include "WbRandom.hpp"
#include "WbSolid.hpp"

#include "../../../include/controller/c/webots/position_sensor.h"  // position sensor types
#include "../../controller/c/messages.h"  // contains the definitions for the macros C_SET_SAMPLING_PERIOD, C_CONFIGURE

#include <QtCore/QDataStream>

#include <cassert>

WbPositionSensor::WbPositionSensor(const QString &modelName, WbTokenizer *tokenizer) : WbJointDevice(modelName, tokenizer) {
  init();
}

WbPositionSensor::WbPositionSensor(WbTokenizer *tokenizer) : WbJointDevice("PositionSensor", tokenizer) {
  init();
}

WbPositionSensor::WbPositionSensor(const WbPositionSensor &other) : WbJointDevice(other) {
  init();
}

WbPositionSensor::WbPositionSensor(const WbNode &other) : WbJointDevice(other) {
  init();
}

void WbPositionSensor::init() {
  mSensor = new WbSensor();
  mValue = 0.0;
  mRequestedDeviceTag = NULL;
  mNoise = findSFDouble("noise");
  mResolution = findSFDouble("resolution");
}

void WbPositionSensor::postFinalize() {
  WbJointDevice::postFinalize();
  connect(mNoise, &WbSFDouble::changed, this, &WbPositionSensor::updateNoise);
  connect(mResolution, &WbSFDouble::changed, this, &WbPositionSensor::updateResolution);
}

void WbPositionSensor::updateNoise() {
  WbFieldChecker::resetDoubleIfNegative(this, mNoise, 0.0);
}

void WbPositionSensor::updateResolution() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void WbPositionSensor::writeConfigure(WbDataStream &stream) {
  mSensor->connectToRobotSignal(robot());

  stream << (unsigned short)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)type();
}

void WbPositionSensor::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;
  if (command & C_SET_SAMPLING_PERIOD) {
    short rate;
    stream >> rate;
    mSensor->setRefreshRate(rate);
  }
  if (command & C_POSITION_SENSOR_GET_ASSOCIATED_DEVICE) {
    short deviceType;
    stream >> deviceType;
    assert(mRequestedDeviceTag == NULL);
    mRequestedDeviceTag = new WbDeviceTag[1];
    WbLogicalDevice *device = getSiblingDeviceByType(deviceType);
    if (!device && deviceType == WB_NODE_ROTATIONAL_MOTOR)
      // check both motor types
      device = getSiblingDeviceByType(WB_NODE_LINEAR_MOTOR);
    mRequestedDeviceTag[0] = device ? device->tag() : 0;
  }
}

double WbPositionSensor::position() const {
  // get exact position
  double pos = WbJointDevice::position();
  // apply noise if needed
  if (mNoise->value() > 0.0)
    pos += mNoise->value() * WbRandom::nextGaussian();
  // apply resolution if needed
  if (mResolution->value() != -1.0)
    pos = WbMathsUtilities::discretize(pos, mResolution->value());
  return pos;
}

void WbPositionSensor::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_POSITION_SENSOR_DATA;
    stream << mValue;
    mSensor->resetPendingValue();
  }
  if (mRequestedDeviceTag != NULL) {
    stream << tag();
    stream << (unsigned char)C_POSITION_SENSOR_GET_ASSOCIATED_DEVICE;
    stream << (unsigned short)mRequestedDeviceTag[0];
    delete[] mRequestedDeviceTag;
    mRequestedDeviceTag = NULL;
  }
}

bool WbPositionSensor::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    mValue = position();
    mSensor->updateTimer();
    return true;
  }
  return false;
}
