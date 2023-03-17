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
#include <stdlib.h>
#include <webots/altimeter.h>
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool enable;
  int sampling_period;
  double altitude;
} Altimeter;

static Altimeter *altimeter_create() {
  Altimeter *altimeter = malloc(sizeof(Altimeter));
  altimeter->enable = false;
  altimeter->sampling_period = 0;
  altimeter->altitude = NAN;
  return altimeter;
}

static Altimeter *altimeter_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_ALTIMETER, true);
  return d ? d->pdata : NULL;
}

static void altimeter_read_answer(WbDevice *d, WbRequest *r) {
  Altimeter *altimeter = d->pdata;

  switch (request_read_uchar(r)) {
    case C_ALTIMETER_DATA:
      altimeter->altitude = request_read_double(r);
      break;
    default:
      ROBOT_ASSERT(0);  // should never be reached
      break;
  }
}

static void altimeter_write_request(WbDevice *d, WbRequest *r) {
  Altimeter *altimeter = d->pdata;
  if (altimeter->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, altimeter->sampling_period);
    altimeter->enable = false;
  }
}

static void altimeter_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void altimeter_toggle_remote(WbDevice *d, WbRequest *r) {
  Altimeter *altimeter = d->pdata;
  if (altimeter->sampling_period != 0)
    altimeter->enable = true;
}

void wbr_altimeter_set_value(WbDeviceTag t, const double value) {
  Altimeter *altimeter = altimeter_get_struct(t);
  if (altimeter) {
    altimeter->altitude = value;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag. \n", __FUNCTION__);
}

// Protected functions (exported to device.cc)

void wb_altimeter_init(WbDevice *d) {
  d->write_request = altimeter_write_request;
  d->read_answer = altimeter_read_answer;
  d->cleanup = altimeter_cleanup;
  d->pdata = altimeter_create();
  d->toggle_remote = altimeter_toggle_remote;
}

// Public function available from the user API

void wb_altimeter_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period. \n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  Altimeter *altimeter = altimeter_get_struct(tag);
  if (altimeter) {
    altimeter->enable = true;
    altimeter->sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_altimeter_disable(WbDeviceTag tag) {
  Altimeter *altimeter = altimeter_get_struct(tag);
  if (altimeter)
    wb_altimeter_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_altimeter_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  Altimeter *altimeter = altimeter_get_struct(tag);
  if (altimeter)
    sampling_period = altimeter->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

double wb_altimeter_get_value(WbDeviceTag tag) {
  double result = NAN;
  robot_mutex_lock();
  Altimeter *altimeter = altimeter_get_struct(tag);
  if (altimeter) {
    if (altimeter->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_altimeter_enable().\n", __FUNCTION__);
    result = altimeter->altitude;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}
