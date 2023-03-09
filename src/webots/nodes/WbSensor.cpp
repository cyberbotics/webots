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

#include "WbSensor.hpp"

#include "WbRobot.hpp"
#include "WbSimulationState.hpp"

#include <cassert>

WbSensor::WbSensor() :
  mRefreshRate(0),  // disabled
  mLastUpdate(-std::numeric_limits<double>::infinity()),
  mIsRemoteMode(false),
  mIsFirstValueReady(false),
  mHasPendingValue(false) {
}

void WbSensor::setRefreshRate(int rate) {
  mRefreshRate = rate;
  // first value available after sampling period elapsed
  mLastUpdate = WbSimulationState::instance()->time();
  if (mRefreshRate == 0)
    emit stateChanged();
  // else state will change when first value is read
}

bool WbSensor::needToRefresh() {
  if (mRefreshRate == 0 || elapsedTime() < mRefreshRate)
    return false;

  if (!mIsFirstValueReady) {
    mIsFirstValueReady = true;
    emit stateChanged();
  }
  return !mIsRemoteMode;
}

bool WbSensor::needToRefreshInMs(int ms) {
  if (mRefreshRate != 0 && !mIsRemoteMode)
    return (elapsedTime() + ms) >= mRefreshRate;
  return false;
}

void WbSensor::updateTimer() {
  mLastUpdate = WbSimulationState::instance()->time();
  mHasPendingValue = true;
}

double WbSensor::elapsedTime() const {
  return WbSimulationState::instance()->time() - mLastUpdate;
}

void WbSensor::toggleRemoteMode(bool enabled) {
  mIsRemoteMode = enabled;
}

void WbSensor::connectToRobotSignal(const WbRobot *robot, bool connectRemoteMode) {
  if (connectRemoteMode)
    connect(robot, &WbRobot::toggleRemoteMode, this, &WbSensor::toggleRemoteMode, Qt::UniqueConnection);
  connect(robot, &WbRobot::wasReset, this, &WbSensor::reset);
}

void WbSensor::reset() {
  mLastUpdate = -std::numeric_limits<double>::infinity();
  mIsFirstValueReady = false;
  mHasPendingValue = false;
}
