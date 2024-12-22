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
#include <webots/light_sensor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool enable;          // need to enable device ?
  int sampling_period;  // milliseconds
  double value;
  int lookup_table_size;
  double *lookup_table;
} LightSensor;

static LightSensor *light_sensor_create() {
  LightSensor *ls = malloc(sizeof(LightSensor));
  ls->enable = false;
  ls->sampling_period = 0;
  ls->value = NAN;
  ls->lookup_table = NULL;
  ls->lookup_table_size = 0;
  return ls;
}

static LightSensor *light_sensor_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_LIGHT_SENSOR, true);
  return d ? d->pdata : NULL;
}

static void light_sensor_read_answer(WbDevice *d, WbRequest *r) {
  LightSensor *ls = (LightSensor *)d->pdata;
  switch (request_read_uchar(r)) {
    case C_LIGHT_SENSOR_DATA:
      ls->value = request_read_double(r);
      break;
    case C_CONFIGURE:
      ls->lookup_table_size = request_read_int32(r);
      free(ls->lookup_table);
      ls->lookup_table = NULL;
      if (ls->lookup_table_size > 0) {
        ls->lookup_table = (double *)malloc(sizeof(double) * ls->lookup_table_size * 3);
        for (int i = 0; i < ls->lookup_table_size * 3; i++)
          ls->lookup_table[i] = request_read_double(r);
      }
      break;
    default:
      ROBOT_ASSERT(0);  // should never be reached
      break;
  }
}

int wb_light_sensor_get_lookup_table_size(WbDeviceTag tag) {
  int result = 0;
  robot_mutex_lock();
  const LightSensor *dev = light_sensor_get_struct(tag);
  if (dev)
    result = dev->lookup_table_size;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const double *wb_light_sensor_get_lookup_table(WbDeviceTag tag) {
  double *result = NULL;
  robot_mutex_lock();
  LightSensor *dev = light_sensor_get_struct(tag);
  if (dev)
    result = dev->lookup_table;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

static void light_sensor_write_request(WbDevice *d, WbRequest *r) {
  LightSensor *ls = (LightSensor *)d->pdata;
  if (ls->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, ls->sampling_period);
    ls->enable = false;  // done
  }
}

static void light_sensor_cleanup(WbDevice *d) {
  LightSensor *ls = (LightSensor *)d->pdata;
  free(ls->lookup_table);
  free(d->pdata);
}

static void light_sensor_toggle_remote(WbDevice *d, WbRequest *r) {
  LightSensor *ls = (LightSensor *)d->pdata;
  if (ls->sampling_period != 0)
    ls->enable = true;
}

void wbr_light_sensor_set_value(WbDeviceTag t, double value) {
  LightSensor *ls = light_sensor_get_struct(t);
  if (ls) {
    ls->value = value;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

// Protected functions (exported to device.cc)

void wb_light_sensor_init(WbDevice *d) {
  d->pdata = light_sensor_create();
  d->write_request = light_sensor_write_request;
  d->read_answer = light_sensor_read_answer;
  d->cleanup = light_sensor_cleanup;
  d->toggle_remote = light_sensor_toggle_remote;
}

// Public function available from the user API

void wb_light_sensor_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  LightSensor *ls = light_sensor_get_struct(tag);
  if (ls) {
    ls->sampling_period = sampling_period;
    ls->enable = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_light_sensor_disable(WbDeviceTag tag) {
  const LightSensor *ls = light_sensor_get_struct(tag);
  if (ls)
    wb_light_sensor_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_light_sensor_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  const LightSensor *ls = light_sensor_get_struct(tag);
  if (ls)
    sampling_period = ls->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

double wb_light_sensor_get_value(WbDeviceTag tag) {
  double value = NAN;
  robot_mutex_lock();
  const LightSensor *ls = light_sensor_get_struct(tag);
  if (ls) {
    if (ls->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_light_sensor_enable().\n", __FUNCTION__);
    value = ls->value;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return value;
}
