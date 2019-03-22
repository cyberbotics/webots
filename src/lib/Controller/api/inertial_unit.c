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

#include <stdio.h>
#include <stdlib.h>
#include <webots/inertial_unit.h>
#include <webots/nodes.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

typedef struct {
  int enable;           // need to enable device ?
  int sampling_period;  // milliseconds
  double rpy[3];        // roll/pitch/yaw
} InertialUnit;

static InertialUnit *inertial_unit_create() {
  InertialUnit *inertial_unit = malloc(sizeof(InertialUnit));
  inertial_unit->enable = false;
  inertial_unit->sampling_period = 0;
  inertial_unit->rpy[0] = NAN;
  inertial_unit->rpy[1] = NAN;
  inertial_unit->rpy[2] = NAN;
  return inertial_unit;
}

// Static functions

static InertialUnit *inertial_unit_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_INERTIAL_UNIT, true);
  return d ? d->pdata : NULL;
}

static void inertial_unit_read_answer(WbDevice *d, WbRequest *r) {
  InertialUnit *s = d->pdata;
  s->rpy[0] = request_read_double(r);
  s->rpy[1] = request_read_double(r);
  s->rpy[2] = request_read_double(r);
}

static void inertial_unit_write_request(WbDevice *d, WbRequest *r) {
  InertialUnit *inertial_unit = d->pdata;
  if (inertial_unit->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, inertial_unit->sampling_period);
    inertial_unit->enable = false;  // done
  }
}

static void inertial_unit_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void inertial_unit_toggle_remote(WbDevice *d, WbRequest *r) {
  InertialUnit *inertial_unit = d->pdata;
  if (inertial_unit->sampling_period != 0)
    inertial_unit->enable = true;
}

void wbr_inertial_unit_set_values(WbDeviceTag t, const double *values) {
  InertialUnit *inertial_unit = inertial_unit_get_struct(t);
  if (inertial_unit) {
    inertial_unit->rpy[0] = values[0];
    inertial_unit->rpy[1] = values[1];
    inertial_unit->rpy[2] = values[2];
  } else
    fprintf(stderr, "Error: wbr_inertial_unit_set_values(): invalid device tag.\n");
}

void wb_inertial_unit_init(WbDevice *);

void wb_inertial_unit_init(WbDevice *d) {
  d->write_request = inertial_unit_write_request;
  d->read_answer = inertial_unit_read_answer;
  d->cleanup = inertial_unit_cleanup;
  d->pdata = inertial_unit_create();
  d->toggle_remote = inertial_unit_toggle_remote;
}

// Public function available from the user API

void wb_inertial_unit_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: wb_inertial_unit_enable() called with negative sampling period.\n");
    return;
  }

  robot_mutex_lock_step();
  InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit) {
    inertial_unit->enable = true;
    inertial_unit->sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: wb_inertial_unit_enable(): invalid device tag.\n");
  robot_mutex_unlock_step();
}

void wb_inertial_unit_disable(WbDeviceTag tag) {
  InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit)
    wb_inertial_unit_enable(tag, 0);
  else
    fprintf(stderr, "Error: wb_inertial_unit_disable(): invalid device tag.\n");
}

int wb_inertial_unit_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock_step();
  InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit)
    sampling_period = inertial_unit->sampling_period;
  else
    fprintf(stderr, "Error: wb_inertial_unit_get_sampling_period(): invalid device tag.\n");
  robot_mutex_unlock_step();
  return sampling_period;
}

const double *wb_inertial_unit_get_roll_pitch_yaw(WbDeviceTag tag) {
  const double *result = NULL;
  robot_mutex_lock_step();
  InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit) {
    if (inertial_unit->sampling_period <= 0)
      fprintf(
        stderr,
        "Error: wb_inertial_unit_get_roll_pitch_yaw() called for a disabled device! Please use: wb_inertial_unit_enable().\n");
    result = inertial_unit->rpy;
  } else
    fprintf(stderr, "Error: wb_inertial_unit_get_roll_pitch_yaw(): invalid device tag.\n");
  robot_mutex_unlock_step();
  return result;
}
