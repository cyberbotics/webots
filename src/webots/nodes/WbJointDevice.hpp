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

// Abstract node class representing a logical joint device

#ifndef WB_JOINT_DEVICE_HPP
#define WB_JOINT_DEVICE_HPP

#include "WbLogicalDevice.hpp"

class WbJoint;
class WbPropeller;
class WbRobot;
class WbTrack;

class WbJointDevice : public WbLogicalDevice {
  Q_OBJECT

public:
  virtual ~WbJointDevice();

  // inherited from WbBaseNode
  void preFinalize() override;

  WbJoint *joint() const;          // joint attached to the device
  WbPropeller *propeller() const;  // propeller attached to the device
  WbTrack *track() const;          // track attached to the device
  WbRobot *robot() const;          // robot that owns the joint
  double position() const;
  virtual int type() const;
  int positionIndex() const { return mPositionIndex; }

  WbLogicalDevice *getSiblingDeviceByType(int nodeType) const;

protected:
  explicit WbJointDevice(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbJointDevice(WbTokenizer *tokenizer = NULL);
  WbJointDevice(const WbJointDevice &other);
  explicit WbJointDevice(const WbNode &other);

private:
  WbJointDevice &operator=(const WbJointDevice &);  // non copyable
  void init();
  mutable bool mRobotFound;
  mutable WbRobot *mRobot;
  int mPositionIndex;
};

#endif
