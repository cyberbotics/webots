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
 * Description:  Abstraction of a Motor
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "Device.hpp"

class Motor : public Device {
public:
  // Device Manager is responsible to create/destroy devices
  Motor(WbDeviceTag tag, int index) : Device(tag, index), mVelocity(false), mVelocityRequested(false) {}
  virtual ~Motor() {}

  double velocity() const { return mVelocity; }
  void setVelocity(double velocity) { mVelocity = velocity; }

  bool isVelocityRequested() const { return mVelocityRequested; }
  void resetVelocityRequested() { mVelocityRequested = false; }
  void setVelocityRequested() { mVelocityRequested = true; }

private:
  double mVelocity;
  bool mVelocityRequested;
};

#endif
