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

#include <webots/joystick.h>
#include <webots/types.h>
#include "joystick_private.h"
#include "messages.h"
#include "robot_private.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct {
  int sampling_period;
  int connected;
  int button_pointer;
  int number_of_axes;
  int number_of_povs;
  int number_of_buttons;
  int *pressed_button;
  int *axis_value;
  int *pov_value;
  char *model;
  // force feedback
  int force_feedback_level;
  double force_feedback_duration;
  double auto_centering_gain;
  double resistance_gain;
  int force_axis;
  bool force_feedback_level_request;
  bool force_feedback_duration_request;
  bool auto_centering_gain_request;
  bool resistance_gain_request;
  bool force_axis_request;
} WbJoystick;

static WbJoystick joystick;

static void joystick_read_value(WbRequest *r) {
  int i;
  // read buttons
  int number_of_pressed_button = request_read_uint32(r);
  for (i = 0; i < number_of_pressed_button; ++i) {
    if (i >= joystick.number_of_buttons || joystick.pressed_button == NULL)
      request_read_uint32(r); /* empty request */
    else
      joystick.pressed_button[i] = request_read_uint32(r);
    // fprintf(stdout, "Pressed button %d / %d.\n", joystick.pressed_button[i], number_of_pressed_button);
  }
  if (number_of_pressed_button > joystick.number_of_buttons)
    number_of_pressed_button = joystick.number_of_buttons;
  if (joystick.pressed_button) {
    joystick.pressed_button[number_of_pressed_button] = -1;
    if (joystick.button_pointer != -1)
      joystick.button_pointer = 0;
  }
  // read axes
  joystick.number_of_axes = request_read_uint32(r);
  for (i = 0; i < joystick.number_of_axes; ++i) {
    int value = request_read_uint32(r);
    // fprintf(stdout, "Received %d for axis %d.\n", value, i);
    if (i < joystick.number_of_axes && joystick.axis_value)
      joystick.axis_value[i] = value;
  }
  // read povs
  joystick.number_of_povs = request_read_uint32(r);
  for (i = 0; i < joystick.number_of_povs; ++i) {
    int value = request_read_uint32(r);
    // fprintf(stdout, "Received %d for pov %d.\n", value, i);
    if (i < joystick.number_of_povs && joystick.pov_value)
      joystick.pov_value[i] = value;
  }
}

// Protected funtions available from other files of the client library

void joystick_write_request(WbRequest *req) {
  if (joystick.button_pointer == -1) {  // need to enable or disable
    request_write_uchar(req, C_ROBOT_SET_JOYSTICK_SAMPLING_PERIOD);
    request_write_uint16(req, joystick.sampling_period);
    joystick.button_pointer = 0;
  }

  if (joystick.force_feedback_level_request) {
    request_write_uchar(req, C_ROBOT_SET_JOYSTICK_FORCE_FEEDBACK);
    request_write_uint16(req, joystick.force_feedback_level);
    joystick.force_feedback_level_request = false;
  }

  if (joystick.force_feedback_duration_request) {
    request_write_uchar(req, C_ROBOT_SET_JOYSTICK_FORCE_FEEDBACK_DURATION);
    request_write_double(req, joystick.force_feedback_duration);
    joystick.force_feedback_duration_request = false;
  }

  if (joystick.auto_centering_gain_request) {
    request_write_uchar(req, C_ROBOT_SET_JOYSTICK_AUTO_CENTERING_GAIN);
    request_write_double(req, joystick.auto_centering_gain);
    joystick.auto_centering_gain_request = false;
  }

  if (joystick.resistance_gain_request) {
    request_write_uchar(req, C_ROBOT_SET_JOYSTICK_RESISTANCE_GAIN);
    request_write_double(req, joystick.resistance_gain);
    joystick.resistance_gain_request = false;
  }

  if (joystick.force_axis_request) {
    request_write_uchar(req, C_ROBOT_SET_JOYSTICK_FORCE_AXIS);
    request_write_uint32(req, joystick.force_axis);
    joystick.force_axis_request = false;
  }
}

void joystick_set_sampling_period(int sampling_period) {
  joystick.sampling_period = sampling_period;
  joystick.button_pointer = -1;
}

bool joystick_read_answer(int message, WbRequest *r) {
  if (message == C_ROBOT_JOYSTICK_VALUE) {
    joystick_read_value(r);
    return true;
  } else if (message == C_ROBOT_JOYSTICK_CONFIG) {
    free(joystick.pressed_button);
    joystick.pressed_button = NULL;
    free(joystick.axis_value);
    joystick.axis_value = NULL;
    free(joystick.pov_value);
    joystick.pov_value = NULL;
    free(joystick.model);
    joystick.model = NULL;
    joystick.number_of_axes = request_read_uint32(r);
    joystick.number_of_buttons = request_read_uint32(r);
    joystick.number_of_povs = request_read_uint32(r);
    if (joystick.number_of_axes < 0 && joystick.number_of_buttons < 0 && joystick.number_of_povs < 0) {
      joystick.number_of_axes = 0;
      joystick.number_of_buttons = 0;
      joystick.number_of_povs = 0;
      joystick.connected = false;
    } else {
      joystick.model = request_read_string(r);
      joystick.axis_value = (int *)malloc(joystick.number_of_axes * sizeof(int));
      joystick.pov_value = (int *)malloc(joystick.number_of_povs * sizeof(int));
      joystick.pressed_button = (int *)malloc((joystick.number_of_buttons + 1) * sizeof(int));
      joystick.connected = true;
      int i;
      for (i = 0; i < joystick.number_of_axes; ++i)
        joystick.axis_value[i] = 0;
      for (i = 0; i < joystick.number_of_povs; ++i)
        joystick.pov_value[i] = 0;
      for (i = 0; i < (joystick.number_of_buttons + 1); ++i)
        joystick.pressed_button[i] = -1;
    }
    return true;
  } else
    return false;
}

void joystick_step_end() {
  if (joystick.sampling_period) {
    if (joystick.pressed_button)
      joystick.pressed_button[0] = -1;
    if (joystick.button_pointer != -1)
      joystick.button_pointer = 0;
  }
}

void wb_joystick_init() {
  joystick.sampling_period = 0;  // initially disabled
  joystick.connected = false;
  joystick.button_pointer = -1;
  joystick.number_of_axes = 0;
  joystick.number_of_povs = 0;
  joystick.number_of_buttons = 0;
  joystick.pressed_button = NULL;
  joystick.axis_value = NULL;
  joystick.pov_value = NULL;
  joystick.model = NULL;
  joystick.force_feedback_level = 0;
  joystick.force_feedback_duration = 0;
  joystick.auto_centering_gain = 0.0;
  joystick.resistance_gain = 0.0;
  joystick.force_axis = 0;
  joystick.force_feedback_level_request = false;
  joystick.force_feedback_duration_request = false;
  joystick.auto_centering_gain_request = false;
  joystick.resistance_gain_request = false;
  joystick.force_axis_request = false;
}

// Public functions available from the keyboard API

void wb_joystick_enable(int sampling_period) {
  robot_mutex_lock();
  joystick.button_pointer = -1;  // need to enable or disable
  joystick.sampling_period = sampling_period;
  robot_mutex_unlock();
}

void wb_joystick_disable() {
  wb_joystick_enable(0);
}

int wb_joystick_get_sampling_period() {
  int sampling_period = 0;
  robot_mutex_lock();
  sampling_period = joystick.sampling_period;
  robot_mutex_unlock();
  return sampling_period;
}

int wb_joystick_get_number_of_axes() {
  if (joystick.sampling_period <= 0)
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);

  return joystick.number_of_axes;
}

int wb_joystick_get_axis_value(int axis) {
  if (joystick.sampling_period <= 0)
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);

  if (axis >= joystick.number_of_axes)
    fprintf(stderr, "Error: %s() called with an 'axis' argument (%d) bigger than or equal to the number of axes (%d).\n",
            __FUNCTION__, axis, joystick.number_of_axes);

  if (joystick.axis_value)
    return joystick.axis_value[axis];
  else
    return 0;
}

int wb_joystick_get_number_of_povs() {
  if (joystick.sampling_period <= 0)
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);

  return joystick.number_of_povs;
}

int wb_joystick_get_pov_value(int pov) {
  if (joystick.sampling_period <= 0)
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);

  if (pov >= joystick.number_of_povs)
    fprintf(stderr, "Error: %s() called with a 'pov' argument (%d) bigger than or equal to the number of axes (%d).\n",
            __FUNCTION__, pov, joystick.number_of_povs);

  if (joystick.pov_value)
    return joystick.pov_value[pov];
  else
    return 0;
}

int wb_joystick_get_pressed_button() {
  if (joystick.sampling_period <= 0)
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);

  if (joystick.button_pointer == -1 || joystick.pressed_button == NULL)
    return -1;
  int button = joystick.pressed_button[(int)joystick.button_pointer];
  if (button >= 0)
    joystick.button_pointer++;
  return button;
}

void wb_joystick_set_constant_force(int level) {
  if (joystick.sampling_period <= 0) {
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);
    return;
  }

  joystick.force_feedback_level = level;
  joystick.force_feedback_level_request = true;
}

void wb_joystick_set_constant_force_duration(double duration) {
  if (joystick.sampling_period <= 0) {
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);
    return;
  }

  if (duration < 0) {
    fprintf(stderr, "Error: %s() called with a negative 'duration' argument.\n", __FUNCTION__);
    return;
  }

  joystick.force_feedback_duration = duration;
  joystick.force_feedback_duration_request = true;
}

void wb_joystick_set_auto_centering_gain(double gain) {
  if (joystick.sampling_period <= 0) {
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);
    return;
  }

  joystick.auto_centering_gain = gain;
  joystick.auto_centering_gain_request = true;
}

void wb_joystick_set_resistance_gain(double gain) {
  if (joystick.sampling_period <= 0) {
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);
    return;
  }

  joystick.resistance_gain = gain;
  joystick.resistance_gain_request = true;
}

void wb_joystick_set_force_axis(int axis) {
  if (joystick.sampling_period <= 0) {
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);
    return;
  }

  if (axis >= joystick.number_of_axes) {
    fprintf(stderr, "Error: %s() called with an 'axis' argument (%d) bigger than or equal to the number of axes (%d).\n",
            __FUNCTION__, axis, joystick.number_of_axes);
    return;
  }

  joystick.force_axis = axis;
  joystick.force_axis_request = true;
}

bool wb_joystick_is_connected() {
  if (joystick.sampling_period <= 0) {
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_joystick_enable().\n", __FUNCTION__);
    return false;
  }

  return joystick.connected;
}

const char *wb_joystick_get_model() {
  return joystick.model;
}
