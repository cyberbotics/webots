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
#include <webots/gps.h>
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool enable;
  int sampling_period;
  WbGpsCoordinateSystem coordinate_system;
  double position[3];
  double speed;
  double motion[3];
} GPS;

static GPS *gps_create() {
  GPS *gps = malloc(sizeof(GPS));
  gps->enable = false;
  gps->sampling_period = 0;
  gps->coordinate_system = WB_GPS_LOCAL_COORDINATE;
  gps->position[0] = NAN;
  gps->position[1] = NAN;
  gps->position[2] = NAN;
  gps->speed = NAN;
  gps->motion[0] = NAN;
  gps->motion[1] = NAN;
  gps->motion[2] = NAN;
  return gps;
}

static GPS *gps_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_GPS, true);
  return d ? d->pdata : NULL;
}

static void gps_read_answer(WbDevice *d, WbRequest *r) {
  GPS *gps = d->pdata;
  int i;

  switch (request_read_uchar(r)) {
    case C_GPS_DATA:
      for (i = 0; i < 3; i++)
        gps->position[i] = request_read_double(r);
      gps->speed = request_read_double(r);
      for (i = 0; i < 3; i++)
        gps->motion[i] = request_read_double(r);
      break;
    case C_CONFIGURE:
      gps->coordinate_system = request_read_int32(r);
      break;
    default:
      ROBOT_ASSERT(0);  // should never be reached
      break;
  }
}

static void gps_write_request(WbDevice *d, WbRequest *r) {
  GPS *gps = d->pdata;
  if (gps->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, gps->sampling_period);
    gps->enable = false;
  }
}

static void gps_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void gps_toggle_remote(WbDevice *d, WbRequest *r) {
  GPS *gps = d->pdata;
  if (gps->sampling_period != 0)
    gps->enable = true;
}

void wbr_gps_set_values(WbDeviceTag t, const double *values) {
  GPS *gps = gps_get_struct(t);
  if (gps) {
    gps->position[0] = values[0];
    gps->position[1] = values[1];
    gps->position[2] = values[2];
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

void wbr_gps_set_velocity_vector(WbDeviceTag t, const double *values) {
  GPS *gps = gps_get_struct(t);
  if (gps) {
    gps->motion[0] = values[0];
    gps->motion[1] = values[1];
    gps->motion[2] = values[2];
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

void wbr_gps_set_speed(WbDeviceTag t, const double speed) {
  GPS *gps = gps_get_struct(t);
  if (gps)
    gps->speed = speed;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

// Protected functions (exported to device.cc)

void wb_gps_init(WbDevice *d) {
  d->write_request = gps_write_request;
  d->read_answer = gps_read_answer;
  d->cleanup = gps_cleanup;
  d->pdata = gps_create();
  d->toggle_remote = gps_toggle_remote;
}

// Public function available from the user API

void wb_gps_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  GPS *gps = gps_get_struct(tag);
  if (gps) {
    gps->enable = true;
    gps->sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_gps_disable(WbDeviceTag tag) {
  const GPS *gps = gps_get_struct(tag);
  if (gps)
    wb_gps_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_gps_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  const GPS *gps = gps_get_struct(tag);
  if (gps)
    sampling_period = gps->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

const double *wb_gps_get_values(WbDeviceTag tag) {
  const double *result = NULL;
  robot_mutex_lock();
  const GPS *gps = gps_get_struct(tag);
  if (gps) {
    if (gps->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_gps_enable().\n", __FUNCTION__);
    result = gps->position;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

double wb_gps_get_speed(WbDeviceTag tag) {
  double result = NAN;
  robot_mutex_lock();
  const GPS *gps = gps_get_struct(tag);
  if (gps) {
    if (gps->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_gps_enable().\n", __FUNCTION__);
    result = gps->speed;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const double *wb_gps_get_speed_vector(WbDeviceTag tag) {
  const double *result = NULL;
  robot_mutex_lock();
  const GPS *gps = gps_get_struct(tag);
  if (gps) {
    if (gps->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_gps_enable().\n", __FUNCTION__);
    result = gps->motion;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const char *wb_gps_convert_to_degrees_minutes_seconds(double decimal_degrees) {
  int degrees = decimal_degrees;
  int minutes = (decimal_degrees - degrees) * 60;
  int seconds = (((decimal_degrees - degrees) * 60) - minutes) * 60;
  char *buffer = (char *)malloc(32 * sizeof(char));
  sprintf(buffer, "%d° %d′ %d″", degrees, minutes, seconds);
  return buffer;
}

WbGpsCoordinateSystem wb_gps_get_coordinate_system(WbDeviceTag tag) {
  WbGpsCoordinateSystem result = WB_GPS_LOCAL_COORDINATE;
  robot_mutex_lock();
  const GPS *gps = gps_get_struct(tag);
  if (gps)
    result = gps->coordinate_system;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}
