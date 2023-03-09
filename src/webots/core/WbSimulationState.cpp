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

#include "WbSimulationState.hpp"

#include <cassert>

WbSimulationState *WbSimulationState::cInstance = NULL;
static WbSimulationState::Mode gResumeMode = WbSimulationState::NONE;

WbSimulationState::WbSimulationState() {
  cInstance = this;
  mMode = PAUSE;
  mPreviousMode = PAUSE;
  mEnabled = true;
  mTime = 0.0;
  mRayTracingSubscribersCount = 0;
}

WbSimulationState::~WbSimulationState() {
  cInstance = NULL;
}

void WbSimulationState::setMode(Mode mode) {
  if (mode == mMode)
    return;
  if (mEnabled == false)
    return;
  mPreviousMode = mMode;
  mMode = mode;
  emit modeChanged();
}

void WbSimulationState::setRendering(bool show) {
  mPerformRendering = show;
  emit renderingStateChanged();
}

void WbSimulationState::pauseSimulation() {
  if (gResumeMode != NONE)  // already paused
    return;

  gResumeMode = mMode;
  setMode(PAUSE);
}

void WbSimulationState::resumeSimulation() {
  if (gResumeMode == NONE)  // nothing to resume
    return;

  setMode(gResumeMode);
  gResumeMode = NONE;
}

void WbSimulationState::resetTime() {
  mTime = 0.0;
}

void WbSimulationState::increaseTime(double dt) {
  mTime += dt;
}

void WbSimulationState::setEnabled(bool enabled) {
  if (mEnabled == enabled)
    return;
  mEnabled = enabled;
  emit modeChanged();
  emit enabledChanged(mEnabled);
}

void WbSimulationState::subscribeToRayTracing() {
  assert(mRayTracingSubscribersCount >= 0);
  mRayTracingSubscribersCount++;
  if (mRayTracingSubscribersCount == 1)
    emit rayTracingEnabled();
}

void WbSimulationState::unsubscribeToRayTracing() {
  assert(mRayTracingSubscribersCount > 0);
  mRayTracingSubscribersCount--;
}
