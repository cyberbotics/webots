/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <webots/mouse.h>
#include "messages.h"
#include "mouse_private.h"
#include "robot_private.h"

typedef struct {
  int sampling_period;
  bool sampling_period_changed;
  bool enable_3d_position;
  bool enable_3d_position_changed;
  WbMouseState state;
} WbMouse;

static WbMouse mouse;

void mouse_write_request(WbRequest *req) {
  if (mouse.sampling_period_changed) {
    request_write_uchar(req, C_ROBOT_SET_MOUSE_SAMPLING_PERIOD);
    request_write_uint16(req, mouse.sampling_period);
    mouse.sampling_period_changed = 0;
  }

  if (mouse.enable_3d_position_changed) {
    request_write_uchar(req, C_ROBOT_MOUSE_ENABLE_3D_POSITION);
    request_write_uchar(req, mouse.enable_3d_position ? 1 : 0);
    mouse.enable_3d_position_changed = false;
  }
}

void mouse_set_sampling_period(int sampling_period) {
  mouse.sampling_period = sampling_period;
}

bool mouse_read_answer(int message, WbRequest *r) {
  if (message == C_ROBOT_MOUSE_VALUE) {
    mouse.state.left = request_read_uchar(r);
    mouse.state.middle = request_read_uchar(r);
    mouse.state.right = request_read_uchar(r);
    mouse.state.u = request_read_double(r);
    mouse.state.v = request_read_double(r);
    mouse.state.x = request_read_double(r);
    mouse.state.y = request_read_double(r);
    mouse.state.z = request_read_double(r);
    return true;
  } else
    return false;
}

void wb_mouse_init() {
  mouse.sampling_period = 0;
  mouse.sampling_period_changed = false;
  mouse.enable_3d_position = false;
  mouse.enable_3d_position_changed = false;
  mouse.state.left = false;
  mouse.state.middle = false;
  mouse.state.right = false;
  mouse.state.u = NAN;
  mouse.state.v = NAN;
  mouse.state.x = NAN;
  mouse.state.y = NAN;
  mouse.state.z = NAN;
}

void wb_mouse_enable(int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  mouse.sampling_period = sampling_period;
  mouse.sampling_period_changed = true;
  robot_mutex_unlock();
}

void wb_mouse_disable() {
  wb_mouse_enable(0);
}

int wb_mouse_get_sampling_period() {
  robot_mutex_lock();
  const int sampling_period = mouse.sampling_period;
  robot_mutex_unlock();
  return sampling_period;
}

void wb_mouse_enable_3d_position() {
  robot_mutex_lock();
  if (mouse.sampling_period == 0)
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_mouse_enable().\n", __FUNCTION__);
  else {
    mouse.enable_3d_position = true;
    mouse.enable_3d_position_changed = true;
  }
  robot_mutex_unlock();
}

void wb_mouse_disable_3d_position() {
  robot_mutex_lock();
  mouse.enable_3d_position = false;
  mouse.enable_3d_position_changed = true;
  robot_mutex_unlock();
}

bool wb_mouse_is_3d_position_enabled() {
  robot_mutex_lock();
  const bool enabled = mouse.enable_3d_position;
  robot_mutex_unlock();
  return enabled;
}

WbMouseState wb_mouse_get_state() {
  return mouse.state;
}

WbMouseState *wb_mouse_get_state_pointer() {
  return &(mouse.state);
}
