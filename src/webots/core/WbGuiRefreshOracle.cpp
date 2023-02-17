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

#include "WbGuiRefreshOracle.hpp"
#include "WbSimulationState.hpp"

#include <QtCore/QCoreApplication>

WbGuiRefreshOracle *WbGuiRefreshOracle::cInstance = NULL;

WbGuiRefreshOracle *WbGuiRefreshOracle::instance() {
  if (!cInstance)
    cInstance = new WbGuiRefreshOracle();
  return cInstance;
}

void WbGuiRefreshOracle::cleanup() {
  if (cInstance) {
    delete cInstance;
    cInstance = NULL;
  }
}

WbGuiRefreshOracle::WbGuiRefreshOracle() : mCanRefreshNow(true) {
  qAddPostRoutine(WbGuiRefreshOracle::cleanup);

  WbSimulationState *state = WbSimulationState::instance();
  connect(state, &WbSimulationState::modeChanged, this, &WbGuiRefreshOracle::updateFlags);
  connect(state, &WbSimulationState::controllerReadRequestsCompleted, this, &WbGuiRefreshOracle::updateFlags);
  connect(&mGlobalRefreshTimer, &QTimer::timeout, this, &WbGuiRefreshOracle::updateFlags);

  // The global refresh timer should not timeout faster than one refresh period
  // so we use the smallest value bigger than the refresh period to check if
  // we should update but haven't yet
  mGlobalRefreshTimer.start(301);
  mLastRefreshTimer.start();
}

WbGuiRefreshOracle::~WbGuiRefreshOracle() {
}

void WbGuiRefreshOracle::updateFlags() {
  // set the mCanRefreshNow to true at frequency 3.33 Hz
  if (!WbSimulationState::instance()->isPaused() && mLastRefreshTimer.elapsed() < 300) {
    mCanRefreshNow = false;
    // restart global refresh timer
    mGlobalRefreshTimer.start(301);
    return;
  }

  bool canRefreshNowHasChanged = !mCanRefreshNow;
  mCanRefreshNow = true;

  if (canRefreshNowHasChanged)
    emit canRefreshActivated();
  emit canRefreshUpdated();
  mLastRefreshTimer.restart();
}
