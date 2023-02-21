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

#include "WbMouse.hpp"

#include "WbSensor.hpp"

#include <cmath>

QList<WbMouse *> WbMouse::mMouses;

WbMouse *WbMouse::create() {
  WbMouse *m = new WbMouse();
  mMouses << m;
  return m;
}

void WbMouse::destroy(WbMouse *mouse) {
  mMouses.removeOne(mouse);
  delete mouse;
}

WbMouse::WbMouse() : mSensor(NULL), mHasMoved(false), mHasClicked(false), mIsTracked(false), mIs3dPositionEnabled(false) {
  reset();
}

WbMouse::~WbMouse() {
  delete mSensor;
}

void WbMouse::setRefreshRate(int rate) {
  if (mSensor == NULL)
    mSensor = new WbSensor();
  mSensor->setRefreshRate(rate);
}

int WbMouse::refreshRate() const {
  if (mSensor)
    return mSensor->refreshRate();
  return 0.0;
}

bool WbMouse::hasPendingValue() {
  return mSensor != NULL ? mSensor->hasPendingValue() : false;
}

void WbMouse::reset() {
  mLeft = false;
  mMiddle = false;
  mRight = false;
  mU = NAN;
  mV = NAN;
  mX = NAN;
  mY = NAN;
  mZ = NAN;
}

bool WbMouse::refreshSensorIfNeeded() {
  if (mSensor->needToRefresh()) {
    mSensor->updateTimer();
    return true;
  }
  return false;
}

bool WbMouse::needToRefresh() const {
  if (mSensor->needToRefresh() || mIsTracked)
    return true;
  return false;
}
