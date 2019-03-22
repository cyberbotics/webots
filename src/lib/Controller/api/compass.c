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
#include <webots/compass.h>
#include <webots/nodes.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

typedef struct {
  bool enable;          // need to enable device ?
  int sampling_period;  // milliseconds
  double north[3];      // north
} Compass;

static Compass *compass_create() {
  Compass *compass = malloc(sizeof(Compass));
  compass->enable = false;
  compass->sampling_period = 0;
  compass->north[0] = NAN;
  compass->north[1] = NAN;
  compass->north[2] = NAN;
  return compass;
}

// Static functions

static Compass *compass_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_COMPASS, true);
  return d ? d->pdata : NULL;
}

static void compass_read_answer(WbDevice *d, WbRequest *r) {
  Compass *compass = d->pdata;
  compass->north[0] = request_read_double(r);
  compass->north[1] = request_read_double(r);
  compass->north[2] = request_read_double(r);
}

static void compass_write_request(WbDevice *d, WbRequest *r) {
  Compass *compass = d->pdata;
  if (compass->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, compass->sampling_period);
    compass->enable = false;  // done
  }
}

static void compass_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void compass_toggle_remote(WbDevice *d, WbRequest *r) {
  Compass *compass = d->pdata;
  if (compass->sampling_period != 0)
    compass->enable = true;
}

void wbr_compass_set_values(WbDeviceTag tag, const double *values) {
  Compass *compass = compass_get_struct(tag);
  if (compass) {
    compass->north[0] = values[0];
    compass->north[1] = values[1];
    compass->north[2] = values[2];
  } else
    fprintf(stderr, "Error: wbr_compass_set_values(): invalid device tag.\n");
}

void wb_compass_init(WbDevice *);

void wb_compass_init(WbDevice *d) {
  d->write_request = compass_write_request;
  d->read_answer = compass_read_answer;
  d->cleanup = compass_cleanup;
  d->pdata = compass_create();
  d->toggle_remote = compass_toggle_remote;
}

// Public function available from the user API

void wb_compass_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: wb_compass_enable() called with negative sampling period.\n");
    return;
  }

  robot_mutex_lock_step();
  Compass *compass = compass_get_struct(tag);
  if (compass) {
    compass->sampling_period = sampling_period;
    compass->enable = true;
  } else
    fprintf(stderr, "Error: wb_compass_enable(): invalid device tag.\n");
  robot_mutex_unlock_step();
}

void wb_compass_disable(WbDeviceTag tag) {
  Compass *compass = compass_get_struct(tag);
  if (compass)
    wb_compass_enable(tag, 0);
  else
    fprintf(stderr, "Error: wb_compass_disable(): invalid device tag.\n");
}

int wb_compass_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock_step();
  Compass *compass = compass_get_struct(tag);
  if (compass)
    sampling_period = compass->sampling_period;
  else
    fprintf(stderr, "Error: wb_compass_get_sampling_period(): invalid device tag.\n");
  robot_mutex_unlock_step();
  return sampling_period;
}

const double *wb_compass_get_values(WbDeviceTag tag) {
  const double *result = NULL;
  robot_mutex_lock_step();
  Compass *compass = compass_get_struct(tag);
  if (compass) {
    if (compass->sampling_period <= 0)
      fprintf(stderr, "Error: wb_compass_get_values() called for a disabled device! Please use: wb_compass_enable().\n");
    result = compass->north;
  } else
    fprintf(stderr, "Error: wb_compass_get_values(): invalid device tag.\n");
  robot_mutex_unlock_step();
  return result;
}
