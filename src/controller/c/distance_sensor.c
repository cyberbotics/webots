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
#include <webots/distance_sensor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/types.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool enable;          // need to enable device ?
  int sampling_period;  // milliseconds
  double value;
  WbDistanceSensorType type;
  double max_value;
  double min_value;
  double aperture;
  int lookup_table_size;
  double *lookup_table;
} DistanceSensor;

static DistanceSensor *distance_sensor_create() {
  DistanceSensor *ds = malloc(sizeof(DistanceSensor));
  ds->enable = false;
  ds->sampling_period = 0;
  ds->value = NAN;
  ds->type = WB_DISTANCE_SENSOR_GENERIC;
  ds->max_value = 0;
  ds->min_value = 0;
  ds->aperture = 0;
  ds->lookup_table = NULL;
  ds->lookup_table_size = 0;
  return ds;
}

static DistanceSensor *distance_sensor_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_DISTANCE_SENSOR, true);
  return d ? d->pdata : NULL;
}

static void distance_sensor_read_answer(WbDevice *d, WbRequest *r) {
  DistanceSensor *ds = (DistanceSensor *)d->pdata;
  switch (request_read_uchar(r)) {
    case C_DISTANCE_SENSOR_DATA:
      ds->value = request_read_double(r);
      break;
    case C_CONFIGURE:
      ds->type = request_read_int32(r);
      ds->min_value = request_read_double(r);
      ds->max_value = request_read_double(r);
      ds->aperture = request_read_double(r);
      ds->lookup_table_size = request_read_int32(r);
      free(ds->lookup_table);
      ds->lookup_table = NULL;
      if (ds->lookup_table_size > 0) {
        ds->lookup_table = (double *)malloc(sizeof(double) * ds->lookup_table_size * 3);
        for (int i = 0; i < ds->lookup_table_size * 3; i++)
          ds->lookup_table[i] = request_read_double(r);
      }
      break;
    default:
      ROBOT_ASSERT(0);  // should never be reached
      break;
  }
}

int wb_distance_sensor_get_lookup_table_size(WbDeviceTag tag) {
  int result = 0;
  robot_mutex_lock();
  const DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds)
    result = ds->lookup_table_size;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const double *wb_distance_sensor_get_lookup_table(WbDeviceTag tag) {
  double *result = NULL;
  robot_mutex_lock();
  DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds)
    result = ds->lookup_table;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

static void distance_sensor_write_request(WbDevice *d, WbRequest *r) {
  DistanceSensor *ds = (DistanceSensor *)d->pdata;
  if (ds->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, ds->sampling_period);
    ds->enable = false;  // done
  }
}

static void distance_sensor_cleanup(WbDevice *d) {
  DistanceSensor *ds = (DistanceSensor *)d->pdata;
  free(ds->lookup_table);
  free(d->pdata);
}

static void distance_sensor_toggle_remote(WbDevice *d, WbRequest *r) {
  DistanceSensor *ds = (DistanceSensor *)d->pdata;
  if (ds->sampling_period != 0)
    ds->enable = true;
}

void wbr_distance_sensor_set_value(WbDeviceTag t, double value) {
  DistanceSensor *ds = distance_sensor_get_struct(t);
  if (ds)
    ds->value = value;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

// Protected functions (exported to WbDevice.cc)

void wb_distance_sensor_init(WbDevice *d) {
  d->pdata = distance_sensor_create();
  d->write_request = distance_sensor_write_request;
  d->read_answer = distance_sensor_read_answer;
  d->cleanup = distance_sensor_cleanup;
  d->toggle_remote = distance_sensor_toggle_remote;
}

// Public function available from the user API

void wb_distance_sensor_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds) {
    ds->sampling_period = sampling_period;
    ds->enable = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

int wb_distance_sensor_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  const DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds)
    sampling_period = ds->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

void wb_distance_sensor_disable(WbDeviceTag tag) {
  const DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds)
    wb_distance_sensor_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

double wb_distance_sensor_get_value(WbDeviceTag tag) {
  double value = NAN;
  robot_mutex_lock();
  const DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds) {
    if (ds->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_distance_sensor_enable().\n", __FUNCTION__);
    value = ds->value;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return value;
}

double wb_distance_sensor_get_max_value(WbDeviceTag tag) {
  double result = NAN;
  robot_mutex_lock();
  const DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds)
    result = ds->max_value;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

double wb_distance_sensor_get_min_value(WbDeviceTag tag) {
  double result = NAN;
  robot_mutex_lock();
  const DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds)
    result = ds->min_value;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

double wb_distance_sensor_get_aperture(WbDeviceTag tag) {
  double result = NAN;
  robot_mutex_lock();
  const DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds)
    result = ds->aperture;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

WbDistanceSensorType wb_distance_sensor_get_type(WbDeviceTag tag) {
  WbDistanceSensorType result = WB_DISTANCE_SENSOR_GENERIC;
  robot_mutex_lock();
  const DistanceSensor *ds = distance_sensor_get_struct(tag);
  if (ds)
    result = ds->type;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}
