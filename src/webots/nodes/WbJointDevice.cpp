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

#include "WbJointDevice.hpp"

#include "WbBallJoint.hpp"
#include "WbField.hpp"
#include "WbHinge2Joint.hpp"
#include "WbJoint.hpp"
#include "WbPropeller.hpp"
#include "WbSolid.hpp"
#include "WbTrack.hpp"

#include <cassert>

WbJointDevice::WbJointDevice(const QString &modelName, WbTokenizer *tokenizer) : WbLogicalDevice(modelName, tokenizer) {
  init();
}

WbJointDevice::WbJointDevice(WbTokenizer *tokenizer) : WbLogicalDevice("PositionSensor", tokenizer) {
  init();
}

WbJointDevice::WbJointDevice(const WbJointDevice &other) : WbLogicalDevice(other) {
  init();
}

WbJointDevice::WbJointDevice(const WbNode &other) : WbLogicalDevice(other) {
  init();
}

WbJointDevice::~WbJointDevice() {
}

void WbJointDevice::init() {
  mRobotFound = false;
  mPositionIndex = 1;
  mRobot = NULL;
}

void WbJointDevice::preFinalize() {
  WbBaseNode::preFinalize();
  // Cache position index
  const WbField *const f = parentField(true);
  assert(f);
  mPositionIndex = 1;
  if (f->name().endsWith("2"))
    mPositionIndex = 2;
  else if (f->name().endsWith("3"))
    mPositionIndex = 3;
}

WbJoint *WbJointDevice::joint() const {
  return dynamic_cast<WbJoint *>(parentNode());
}

WbPropeller *WbJointDevice::propeller() const {
  return dynamic_cast<WbPropeller *>(parentNode());
}

WbTrack *WbJointDevice::track() const {
  return dynamic_cast<WbTrack *>(parentNode());
}

WbLogicalDevice *WbJointDevice::getSiblingDeviceByType(int nodeType) const {
  WbJoint *j = joint();
  if (j) {
    WbBallJoint *ballJoint = dynamic_cast<WbBallJoint *>(j);
    if (ballJoint) {
      // special case for nodes in devices3 field
      bool isDevice3 = false;
      for (int i = 0; i < ballJoint->devices3Number(); ++i) {
        if (this == ballJoint->device3(i)) {
          isDevice3 = true;
          break;
        }
      }
      if (isDevice3) {
        for (int i = 0; i < ballJoint->devices3Number(); ++i) {
          WbLogicalDevice *device3 = ballJoint->device3(i);
          if (device3 && device3->nodeType() == nodeType)
            return device3;
        }
        return NULL;
      }
    }
    WbHinge2Joint *hinge2 = dynamic_cast<WbHinge2Joint *>(j);
    if (hinge2) {
      // special case for nodes in devices2 field
      bool isDevice2 = false;
      for (int i = 0; i < hinge2->devices2Number(); ++i) {
        if (this == hinge2->device2(i)) {
          isDevice2 = true;
          break;
        }
      }
      if (isDevice2) {
        for (int i = 0; i < hinge2->devices2Number(); ++i) {
          WbLogicalDevice *device2 = hinge2->device2(i);
          if (device2 && device2->nodeType() == nodeType)
            return device2;
        }
        return NULL;
      }
    }

    for (int i = 0; i < j->devicesNumber(); ++i) {
      WbLogicalDevice *device = j->device(i);
      if (device && device->nodeType() == nodeType)
        return device;
    }
    return NULL;
  }

  WbTrack *t = track();
  if (t) {
    foreach (WbLogicalDevice *device, t->devices()) {
      if (device && device->nodeType() == nodeType)
        return device;
    }
  }

  return NULL;
}

int WbJointDevice::type() const {
  const WbJoint *const j = joint();
  if (j)
    return j->nodeType() == WB_NODE_SLIDER_JOINT ? WB_LINEAR : WB_ROTATIONAL;

  if (track())
    return WB_LINEAR;

  // defaults to ROTATIONAL for WbPropeller
  return WB_ROTATIONAL;
}

WbRobot *WbJointDevice::robot() const {
  if (mRobotFound == true)
    return mRobot;

  const WbSolid *const us = upperSolid();
  if (us)
    mRobot = us->robot();

  if (mRobot)
    mRobotFound = true;

  return mRobot;
}

double WbJointDevice::position() const {
  // return exact position
  const WbJoint *const j = joint();
  if (j)
    return j->position(mPositionIndex);

  const WbTrack *const t = track();
  if (t)
    return t->position();

  assert(propeller());
  return propeller()->position();
}
