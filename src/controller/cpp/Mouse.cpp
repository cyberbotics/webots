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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/mouse.h>
#include <webots/Mouse.hpp>

using namespace webots;

void Mouse::enable(int sampling_period) {
  wb_mouse_enable(sampling_period);
}

void Mouse::disable() {
  wb_mouse_disable();
}

int Mouse::getSamplingPeriod() const {
  return wb_mouse_get_sampling_period();
}

void Mouse::enable3dPosition() {
  wb_mouse_enable_3d_position();
}

void Mouse::disable3dPosition() {
  wb_mouse_disable_3d_position();
}

bool Mouse::is3dPositionEnabled() const {
  return wb_mouse_is_3d_position_enabled();
}

MouseState Mouse::getState() const {
  return wb_mouse_get_state();
}
