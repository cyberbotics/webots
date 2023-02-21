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

#include "WbBrake.hpp"
#include "WbDataStream.hpp"

#include <QtCore/QDataStream>
#include <cassert>
#include "../../controller/c/messages.h"  // contains the definitions for the macros C_BRAKE_SET_DAMPING_CONSTANT and C_CONFIGURE

WbBrake::WbBrake(const QString &modelName, WbTokenizer *tokenizer) : WbJointDevice(modelName, tokenizer) {
  init();
}

WbBrake::WbBrake(WbTokenizer *tokenizer) : WbJointDevice("Brake", tokenizer) {
  init();
}

WbBrake::WbBrake(const WbBrake &other) : WbJointDevice(other) {
  init();
}

WbBrake::WbBrake(const WbNode &other) : WbJointDevice(other) {
  init();
}

void WbBrake::init() {
  mBrakingDampingConstant = 0.0;
  mRequestedDeviceTag = NULL;
}

void WbBrake::reset(const QString &id) {
  WbJointDevice::reset(id);
  mBrakingDampingConstant = 0.0;
  emit brakingChanged();
}

void WbBrake::writeConfigure(WbDataStream &stream) {
  stream << (unsigned short)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)type();
}

void WbBrake::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;
  if (command & C_BRAKE_SET_DAMPING_CONSTANT) {
    double dampingConstant;
    stream >> dampingConstant;
    mBrakingDampingConstant = dampingConstant;
    emit brakingChanged();
  }
  if (command & C_BRAKE_GET_ASSOCIATED_DEVICE) {
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

void WbBrake::writeAnswer(WbDataStream &stream) {
  if (mRequestedDeviceTag != NULL) {
    stream << tag();
    stream << (unsigned char)C_BRAKE_GET_ASSOCIATED_DEVICE;
    stream << (unsigned short)mRequestedDeviceTag[0];
    delete[] mRequestedDeviceTag;
    mRequestedDeviceTag = NULL;
  }
}
