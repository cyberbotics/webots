/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/keyboard.h>
#include <webots/types.h>
#include "keyboard_private.h"
#include "messages.h"
#include "robot_private.h"

#include <stdio.h>  // fprintf

typedef struct {
  int key[8];
  int sampling_period;
  char pointer;
} WbKeyboard;

static WbKeyboard keyboard;

static void keyboard_read_value(WbRequest *r) {
  int i;
  int max = request_read_uchar(r);
  for (i = 0; i < max; ++i) {
    if (i >= 7)
      request_read_uint32(r);  // empty request
    else
      keyboard.key[i] = request_read_uint32(r);
  }
  if (max > 7)
    max = 7;
  keyboard.key[max] = -1;
}

// Protected funtions available from other files of the client library

void keyboard_write_request(WbRequest *req) {
  if (keyboard.pointer == -1) {  // need to enable or disable
    request_write_uchar(req, C_ROBOT_SET_KEYBOARD_SAMPLING_PERIOD);
    request_write_uint16(req, keyboard.sampling_period);
    keyboard.pointer = 0;
  }
}

void keyboard_set_sampling_period(int sampling_period) {
  keyboard.sampling_period = sampling_period;
  keyboard.pointer = -1;
}

bool keyboard_read_answer(int message, WbRequest *r) {
  if (message == C_ROBOT_KEYBOARD_VALUE) {
    keyboard_read_value(r);
    return true;
  } else
    return false;
}

void keyboard_step_end() {
  if (keyboard.sampling_period) {
    keyboard.key[0] = -1;
    if (keyboard.pointer != -1)
      keyboard.pointer = 0;
  }
}

void wb_keyboard_init() {
  keyboard.sampling_period = 0;  // initially disabled
  keyboard.pointer = 0;
  keyboard.key[0] = -1;
}

// Public functions available from the keyboard API

void wb_keyboard_enable(int sampling_period) {
  robot_mutex_lock();
  keyboard.key[0] = -1;
  keyboard.pointer = -1;  // need to enable or disable
  keyboard.sampling_period = sampling_period;
  robot_mutex_unlock();
}

void wb_keyboard_disable() {
  wb_keyboard_enable(0);
}

int wb_keyboard_get_sampling_period() {
  int sampling_period = 0;
  robot_mutex_lock();
  sampling_period = keyboard.sampling_period;
  robot_mutex_unlock();
  return sampling_period;
}

int wb_keyboard_get_key() {
  if (keyboard.sampling_period <= 0)
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_keyboard_enable().\n", __FUNCTION__);

  int r = -1;
  robot_mutex_lock();
  if (keyboard.pointer != -1) {
    r = keyboard.key[(int)keyboard.pointer];
    if (r >= 0)
      keyboard.pointer++;
  }
  robot_mutex_unlock();
  return r;
}
