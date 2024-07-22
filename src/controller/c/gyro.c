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
#include <webots/gyro.h>
#include <webots/nodes.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

typedef struct {
  int enable;           // need to enable device ?
  int sampling_period;  // milliseconds
  double velocity[3];   // angular velocity
  int lookup_table_size;
  double *lookup_table;
} Gyro;

static Gyro *gyro_create() {
  Gyro *gyro = malloc(sizeof(Gyro));
  gyro->enable = false;
  gyro->sampling_period = 0;
  gyro->velocity[0] = NAN;
  gyro->velocity[1] = NAN;
  gyro->velocity[2] = NAN;
  gyro->lookup_table = NULL;
  gyro->lookup_table_size = 0;
  return gyro;
}

// Static functions

static Gyro *gyro_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_GYRO, true);
  return d ? d->pdata : NULL;
}

static void gyro_read_answer(WbDevice *d, WbRequest *r) {
  Gyro *gyro = d->pdata;
  switch (request_read_uchar(r)) {
    case C_GYRO_DATA:
      gyro->velocity[0] = request_read_double(r);
      gyro->velocity[1] = request_read_double(r);
      gyro->velocity[2] = request_read_double(r);
      break;
    case C_CONFIGURE:
      gyro->lookup_table_size = request_read_int32(r);
      free(gyro->lookup_table);
      gyro->lookup_table = NULL;
      if (gyro->lookup_table_size > 0) {
        gyro->lookup_table = (double *)malloc(sizeof(double) * gyro->lookup_table_size * 3);
        for (int i = 0; i < gyro->lookup_table_size * 3; i++)
          gyro->lookup_table[i] = request_read_double(r);
      }
      break;
    default:
      ROBOT_ASSERT(0);  // should never be reached
      break;
  }
}

int wb_gyro_get_lookup_table_size(WbDeviceTag tag) {
  int result = 0;
  robot_mutex_lock();
  const Gyro *dev = gyro_get_struct(tag);
  if (dev)
    result = dev->lookup_table_size;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const double *wb_gyro_get_lookup_table(WbDeviceTag tag) {
  double *result = NULL;
  robot_mutex_lock();
  Gyro *dev = gyro_get_struct(tag);
  if (dev)
    result = dev->lookup_table;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

static void gyro_write_request(WbDevice *d, WbRequest *r) {
  Gyro *gyro = d->pdata;
  if (gyro->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, gyro->sampling_period);
    gyro->enable = false;  // done
  }
}

static void gyro_cleanup(WbDevice *d) {
  Gyro *gyro = (Gyro *)d->pdata;
  free(gyro->lookup_table);
  free(d->pdata);
}

static void gyro_toggle_remote(WbDevice *d, WbRequest *r) {
  Gyro *gyro = d->pdata;
  if (gyro->sampling_period != 0)
    gyro->enable = true;
}

void wbr_gyro_set_values(WbDeviceTag t, const double *values) {
  Gyro *gyro = gyro_get_struct(t);
  if (gyro) {
    gyro->velocity[0] = values[0];
    gyro->velocity[1] = values[1];
    gyro->velocity[2] = values[2];
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

void wb_gyro_init(WbDevice *);

void wb_gyro_init(WbDevice *d) {
  d->write_request = gyro_write_request;
  d->read_answer = gyro_read_answer;
  d->cleanup = gyro_cleanup;
  d->pdata = gyro_create();
  d->toggle_remote = gyro_toggle_remote;
}

// Public function available from the user API

void wb_gyro_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  Gyro *gyro = gyro_get_struct(tag);
  if (gyro) {
    gyro->enable = true;
    gyro->sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_gyro_disable(WbDeviceTag tag) {
  const Gyro *gyro = gyro_get_struct(tag);
  if (gyro)
    wb_gyro_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_gyro_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  const Gyro *gyro = gyro_get_struct(tag);
  if (gyro)
    sampling_period = gyro->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

const double *wb_gyro_get_values(WbDeviceTag tag) {
  const double *result = NULL;
  robot_mutex_lock();
  const Gyro *gyro = gyro_get_struct(tag);
  if (gyro) {
    if (gyro->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_gyro_enable().\n", __FUNCTION__);
    result = gyro->velocity;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}
