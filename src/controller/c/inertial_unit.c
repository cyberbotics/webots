/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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
#include <stdlib.h>
#include <string.h>
#include <webots/inertial_unit.h>
#include <webots/nodes.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

typedef struct {
  int enable;           // need to enable device ?
  int sampling_period;  // milliseconds
  double quaternion[4];
  double noise;
  char *coordinate_system;
} InertialUnit;

static InertialUnit *inertial_unit_create() {
  InertialUnit *inertial_unit = malloc(sizeof(InertialUnit));
  inertial_unit->enable = false;
  inertial_unit->sampling_period = 0;
  inertial_unit->coordinate_system = NULL;
  return inertial_unit;
}

// Static functions

static InertialUnit *inertial_unit_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_INERTIAL_UNIT, true);
  return d ? d->pdata : NULL;
}

static void inertial_unit_read_answer(WbDevice *d, WbRequest *r) {
  InertialUnit *s = d->pdata;

  switch (request_read_uchar(r)) {
    case C_INERTIAL_UNIT_DATA:
      for (int i = 0; i < 4; i++)
        s->quaternion[i] = request_read_double(r);
      break;
    case C_CONFIGURE:
      s->noise = request_read_double(r);
      s->coordinate_system = request_read_string(r);
      break;
    default:
      ROBOT_ASSERT(0);  // should never be reached
      break;
  }
}

double wb_inertial_unit_get_noise(WbDeviceTag tag) {
  double result = 0;
  robot_mutex_lock();
  const InertialUnit *dev = inertial_unit_get_struct(tag);
  if (dev)
    result = dev->noise;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
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
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit) {
    inertial_unit->enable = true;
    inertial_unit->sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_inertial_unit_disable(WbDeviceTag tag) {
  const InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit)
    wb_inertial_unit_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_inertial_unit_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  const InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit)
    sampling_period = inertial_unit->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

const double *wb_inertial_unit_get_roll_pitch_yaw(WbDeviceTag tag) {
  static double result[3];
  robot_mutex_lock();
  const InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit) {
    if (inertial_unit->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_inertial_unit_enable().\n", __FUNCTION__);

    const double *q = inertial_unit->quaternion;

    if (strcmp(inertial_unit->coordinate_system, "NUE") == 0) {
      // NUE: extrensic rotation matrix e = Y(yaw) Z(pitch) X(roll)
      result[2] = atan2(2 * q[1] * q[3] - 2 * q[0] * q[2], 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]);
      result[0] = atan2(2 * q[0] * q[3] - 2 * q[1] * q[2], 1 - 2 * q[0] * q[0] - 2 * q[2] * q[2]);
      result[1] = asin(2 * q[0] * q[1] + 2 * q[2] * q[3]);
    } else {
      // ENU: extrensic rotation matrix e = Z(yaw) Y(pitch) X(roll)
      const double t0 = 2.0 * (q[3] * q[0] + q[1] * q[2]);
      const double t1 = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
      const double roll = atan2(t0, t1);
      double t2 = 2.0 * (q[3] * q[1] - q[2] * q[0]);
      t2 = (t2 > 1.0) ? 1.0 : t2;
      t2 = (t2 < -1.0) ? -1.0 : t2;
      const double pitch = asin(t2);
      const double t3 = 2.0 * (q[3] * q[2] + q[0] * q[1]);
      const double t4 = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
      const double yaw = atan2(t3, t4);
      result[0] = roll;
      result[1] = pitch;
      result[2] = yaw;
    }
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const double *wb_inertial_unit_get_quaternion(WbDeviceTag tag) {
  const double *result = NULL;
  robot_mutex_lock();
  const InertialUnit *inertial_unit = inertial_unit_get_struct(tag);
  if (inertial_unit) {
    if (inertial_unit->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_inertial_unit_enable().\n", __FUNCTION__);
    result = inertial_unit->quaternion;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}
