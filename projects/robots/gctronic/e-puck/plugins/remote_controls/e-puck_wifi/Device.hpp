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
 * Description:  Abstraction of a Webots device
 */

#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <webots/nodes.h>
#include <webots/types.h>

class Device {
public:
  // Device Manager is responsible to create/destroy devices
  Device(WbDeviceTag tag, int index);
  virtual ~Device() {}

  WbDeviceTag tag() const { return mTag; }

  int index() const { return mIndex; }

private:
  WbDeviceTag mTag;
  WbNodeType mType;
  const char *mName;

  int mIndex;
};

#endif
