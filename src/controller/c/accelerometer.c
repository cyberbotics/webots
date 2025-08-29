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
#include <stdlib.h>  // malloc and free
#include <webots/accelerometer.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool enable;          // need to enable device ?
  int sampling_period;  // milliseconds
  double values[3];     // acceleration
  int lookup_table_size;
  double *lookup_table;
} Accelerometer;

static Accelerometer *accelerometer_create() {
  Accelerometer *acc = malloc(sizeof(Accelerometer));
  acc->enable = false;
  acc->sampling_period = 0;
  acc->values[0] = NAN;
  acc->values[1] = NAN;
  acc->values[2] = NAN;
  acc->lookup_table = NULL;
  acc->lookup_table_size = 0;
  return acc;
}

static Accelerometer *accelerometer_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_ACCELEROMETER, true);
  return d ? d->pdata : NULL;
}

static void accelerometer_read_answer(WbDevice *d, WbRequest *r) {
  Accelerometer *acc = d->pdata;
  switch (request_read_uchar(r)) {
    case C_ACCELEROMETER_DATA:
      acc->values[0] = request_read_double(r);
      acc->values[1] = request_read_double(r);
      acc->values[2] = request_read_double(r);
      break;
    case C_CONFIGURE:
      acc->lookup_table_size = request_read_int32(r);
      free(acc->lookup_table);
      acc->lookup_table = NULL;
      if (acc->lookup_table_size > 0) {
        acc->lookup_table = (double *)malloc(sizeof(double) * acc->lookup_table_size * 3);
        for (int i = 0; i < acc->lookup_table_size * 3; i++)
          acc->lookup_table[i] = request_read_double(r);
      }
      break;
    default:
      ROBOT_ASSERT(0);  // should never be reached
      break;
  }
}

int wb_accelerometer_get_lookup_table_size(WbDeviceTag tag) {
  int result = 0;
  robot_mutex_lock();
  const Accelerometer *dev = accelerometer_get_struct(tag);
  if (dev)
    result = dev->lookup_table_size;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const double *wb_accelerometer_get_lookup_table(WbDeviceTag tag) {
  const double *result = NULL;
  robot_mutex_lock();
  const Accelerometer *dev = accelerometer_get_struct(tag);
  if (dev)
    result = dev->lookup_table;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

static void accelerometer_write_request(WbDevice *d, WbRequest *r) {
  Accelerometer *acc = d->pdata;
  if (acc->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, acc->sampling_period);
    acc->enable = false;  // done
  }
}

static void accelerometer_cleanup(WbDevice *d) {
  Accelerometer *acc = (Accelerometer *)d->pdata;
  free(acc->lookup_table);
  free(d->pdata);
}

static void accelerometer_toggle_remote(WbDevice *d, WbRequest *r) {
  Accelerometer *acc = d->pdata;
  if (acc->sampling_period != 0)
    acc->enable = true;
}

void wbr_accelerometer_set_values(WbDeviceTag tag, const double *values) {
  Accelerometer *acc = accelerometer_get_struct(tag);
  if (acc) {
    acc->values[0] = values[0];
    acc->values[1] = values[1];
    acc->values[2] = values[2];
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

// Protected functions (exported to device.cc)

void wb_accelerometer_init(WbDevice *d) {
  d->pdata = accelerometer_create();
  d->write_request = accelerometer_write_request;
  d->read_answer = accelerometer_read_answer;
  d->cleanup = accelerometer_cleanup;
  d->toggle_remote = accelerometer_toggle_remote;
}

// Public function available from the user API

void wb_accelerometer_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  Accelerometer *acc = accelerometer_get_struct(tag);
  if (acc) {
    acc->sampling_period = sampling_period;
    acc->enable = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_accelerometer_disable(WbDeviceTag tag) {
  const Accelerometer *acc = accelerometer_get_struct(tag);
  if (acc)
    wb_accelerometer_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_accelerometer_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  const Accelerometer *acc = accelerometer_get_struct(tag);
  if (acc)
    sampling_period = acc->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

const double *wb_accelerometer_get_values(WbDeviceTag tag) {
  const double *result = NULL;
  robot_mutex_lock();
  const Accelerometer *acc = accelerometer_get_struct(tag);
  if (acc) {
    if (acc->sampling_period == 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_accelerometer_enable().\n", __FUNCTION__);
    result = acc->values;
  }
  robot_mutex_unlock();
  return result;
}
