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

/*
 * Description:  Abstraction of a LED
 */

#ifndef LED_HPP
#define LED_HPP

#include "Device.hpp"

class Led : public Device {
public:
  // Device Manager is responsible to create/destroy devices
  Led(WbDeviceTag tag, int index) : Device(tag, index), mState(false), mLedRequested(false) {}
  virtual ~Led() {}

  bool state() const { return mState; }
  void setState(bool state) { mState = state; }

  bool isLedRequested() const { return mLedRequested; }
  void resetLedRequested() { mLedRequested = false; }
  void setLedRequested() { mLedRequested = true; }

private:
  bool mState;
  bool mLedRequested;
};

#endif
