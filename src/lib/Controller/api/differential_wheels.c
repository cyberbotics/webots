/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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

// This module describes a mobile robot basis with two independant motor wheels
// that can be controlled in speed and have incremental encoders for controlling
// position of each wheel.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/differential_wheels.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include "device_private.h"
#include "differential_wheels_private.h"
#include "messages.h"
#include "robot_private.h"

// Private functions

typedef struct {
  bool set_encoders;
  bool set_speed;
  bool enable_encoders;
  int encoders_sampling_period;
  double left_encoder;
  double right_encoder;
  double left_speed;
  double right_speed;
  double max_speed;
  double speed_unit;
} DifferentialWheels;

static DifferentialWheels *differential_wheels_create() {
  DifferentialWheels *dw = malloc(sizeof(DifferentialWheels));
  dw->set_encoders = false;
  dw->set_speed = false;
  dw->enable_encoders = false;
  dw->encoders_sampling_period = 0;
  dw->left_speed = NAN;
  dw->right_speed = NAN;
  dw->left_encoder = 0.0;
  dw->right_encoder = 0.0;
  dw->max_speed = 0.0;
  dw->speed_unit = 0.0;
  return dw;
}

static DifferentialWheels *differential_wheels_get_struct() {
  WbDevice *d = robot_get_robot_device();
  return d ? d->pdata : NULL;
}

static void differential_wheels_write_request(WbDevice *d, WbRequest *r) {
  // chain with base class
  robot_write_request(d, r);

  DifferentialWheels *dw = d->pdata;
  if (dw->set_speed) {
    request_write_uchar(r, C_DIFFERENTIAL_WHEELS_SET_SPEED);
    request_write_double(r, dw->left_speed);
    request_write_double(r, dw->right_speed);
    dw->set_speed = false;
  }
  if (dw->enable_encoders) {
    request_write_uchar(r, C_DIFFERENTIAL_WHEELS_ENCODERS_SET_SAMPLING_PERIOD);
    request_write_uint16(r, dw->encoders_sampling_period);
    dw->enable_encoders = false;
  }
  if (dw->set_encoders) {
    request_write_uchar(r, C_DIFFERENTIAL_WHEELS_ENCODERS_SET_VALUE);
    request_write_double(r, dw->left_encoder);
    request_write_double(r, dw->right_encoder);
    dw->set_encoders = false;
  }
}

static void differential_wheels_read_answer(WbDevice *d, WbRequest *r) {
  DifferentialWheels *dw = d->pdata;
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:
      dw->max_speed = request_read_double(r);
      dw->speed_unit = request_read_double(r);
      break;
    case C_DIFFERENTIAL_WHEELS_GET_ENCODERS:
      dw->left_encoder = request_read_double(r);
      dw->right_encoder = request_read_double(r);
      break;
    default:
      r->pointer--;  // unread last value
      robot_read_answer(d, r);
      break;
  }
}

static void differential_wheels_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void differential_wheels_toggle_remote(WbDevice *d, WbRequest *r) {
  // chain with base class
  robot_toggle_remote(d, r);

  // stop motor in the old mode
  request_write_uchar(r, C_DIFFERENTIAL_WHEELS_SET_SPEED);
  request_write_double(r, 0.0);
  request_write_double(r, 0.0);

  // scheduled resend of state to the new mode
  DifferentialWheels *dw = d->pdata;
  dw->set_speed = true;
  if (dw->encoders_sampling_period > 0) {
    dw->set_encoders = true;
    dw->enable_encoders = true;
  }
}

void wbr_differential_wheels_set_encoders(double left, double right) {
  if (!robot_check_differential_wheels("wbr_differential_wheels_set_encoders"))
    return;

  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw) {
    // note that contrary to the controller version : wb_differential_wheels_set_encoders
    // this function will not send a request
    dw->left_encoder = left;
    dw->right_encoder = right;
  }
}

// Protected functions

void wb_differential_wheels_init(WbDevice *d) {
  d->write_request = differential_wheels_write_request;
  d->read_answer = differential_wheels_read_answer;
  d->cleanup = differential_wheels_cleanup;
  d->toggle_remote = differential_wheels_toggle_remote;
  d->pdata = differential_wheels_create();
}

// Public functions available from the user API

void wb_differential_wheels_set_speed(double left, double right) {
  if (!robot_check_differential_wheels("wb_differential_wheels_set_speed"))
    return;

  if (isnan(left) || isnan(right)) {
    fprintf(stderr, "Error: wb_differential_wheels_set_speed(): invalid NaN value passed as argument.\n");
    return;
  }

  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw) {
    if (left * dw->speed_unit * 0.99999 > dw->max_speed || right * dw->speed_unit * 0.99999 > dw->max_speed)
      fprintf(stderr, "Error: wb_differential_wheels_set_speed(%g,%g) overflows maxSpeed/speedUnit: %g/%g=%g.\n", left, right,
              dw->max_speed, dw->speed_unit, dw->max_speed / dw->speed_unit);
    dw->left_speed = left;
    dw->right_speed = right;
    dw->set_speed = true;
  }
  robot_mutex_unlock_step();
}

void wb_differential_wheels_enable_encoders(int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: wb_differential_wheels_enable_encoders() called with negative sampling period.\n");
    return;
  }

  if (!robot_check_differential_wheels("wb_differential_wheels_enable_encoders"))
    return;

  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw) {
    dw->encoders_sampling_period = sampling_period;
    dw->enable_encoders = true;
  }
  robot_mutex_unlock_step();
}

void wb_differential_wheels_disable_encoders() {
  if (!robot_check_differential_wheels("wb_differential_wheels_disable_encoders"))
    return;

  wb_differential_wheels_enable_encoders(0);
}

int wb_differential_wheels_get_encoders_sampling_period() {
  int sampling_period = 0;
  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw)
    sampling_period = dw->encoders_sampling_period;
  robot_mutex_unlock_step();
  return sampling_period;
}

double wb_differential_wheels_get_left_speed() {
  double speed = 0.0;
  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw)
    speed = dw->left_speed;
  robot_mutex_unlock_step();
  return speed;
}

double wb_differential_wheels_get_right_speed() {
  double speed = 0.0;
  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw)
    speed = dw->right_speed;
  robot_mutex_unlock_step();
  return speed;
}

double wb_differential_wheels_get_max_speed() {
  double speed = NAN;
  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw)
    speed = dw->max_speed;
  robot_mutex_unlock_step();
  return speed;
}

double wb_differential_wheels_get_speed_unit() {
  double speed_unit = NAN;
  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw)
    speed_unit = dw->speed_unit;
  robot_mutex_unlock_step();
  return speed_unit;
}

double wb_differential_wheels_get_left_encoder() {
  if (!robot_check_differential_wheels("wb_differential_wheels_get_left_encoder"))
    return NAN;

  double result = NAN;
  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw) {
    if (dw->encoders_sampling_period <= 0)
      fprintf(stderr, "Error: wb_differential_wheels_get_left_encoder() called for a disabled device! Please use: "
                      "wb_differential_wheels_enable_encoders().\n");
    result = dw->left_encoder;
  }
  robot_mutex_unlock_step();
  return result;
}

double wb_differential_wheels_get_right_encoder() {
  if (!robot_check_differential_wheels("wb_differential_wheels_get_right_encoder"))
    return NAN;

  double result = NAN;
  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw) {
    if (dw->encoders_sampling_period <= 0)
      fprintf(stderr, "Error: wb_differential_wheels_get_right_encoder() called for a disabled device! Please use: "
                      "wb_differential_wheels_enable_encoders().\n");
    result = dw->right_encoder;
  }
  robot_mutex_unlock_step();
  return result;
}

void wb_differential_wheels_set_encoders(double left, double right) {
  if (!robot_check_differential_wheels("wb_differential_wheels_set_encoders"))
    return;

  if (isnan(left) || isnan(right)) {
    fprintf(stderr, "Error: wb_differential_wheels_set_encoders(): invalid NaN value passed as argument.\n");
    return;
  }

  robot_mutex_lock_step();
  DifferentialWheels *dw = differential_wheels_get_struct();
  if (dw) {
    dw->set_encoders = true;
    dw->left_encoder = left;
    dw->right_encoder = right;
  }
  robot_mutex_unlock_step();
}
