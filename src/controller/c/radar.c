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
#include <stdlib.h>  // malloc and free
#include <webots/nodes.h>
#include <webots/radar.h>
#include <webots/robot.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool enable;             // need to enable device ?
  int sampling_period;     // milliseconds
  int target_number;       // number of targets currrently seen by the radar
  WbRadarTarget *targets;  // list of targets
  double min_range;
  double max_range;
  double horizontal_fov;
  double vertical_fov;
} Radar;

static Radar *radar_create() {
  Radar *radar = malloc(sizeof(Radar));
  radar->enable = false;
  radar->sampling_period = 0;
  radar->target_number = 0;
  radar->targets = NULL;
  radar->min_range = 0;
  radar->max_range = 0;
  radar->horizontal_fov = 0;
  radar->vertical_fov = 0;
  return radar;
}

static Radar *radar_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_RADAR, true);
  return d ? d->pdata : NULL;
}

static void radar_read_answer(WbDevice *d, WbRequest *r) {
  Radar *radar = d->pdata;
  int i = 0;
  switch (request_read_uchar(r)) {
    case C_RADAR_DATA:  // read target list
      radar->target_number = request_read_int32(r);
      free(radar->targets);
      radar->targets = (WbRadarTarget *)malloc(radar->target_number * sizeof(WbRadarTarget));
      for (i = 0; i < radar->target_number; ++i) {
        radar->targets[i].distance = request_read_double(r);
        radar->targets[i].received_power = request_read_double(r);
        radar->targets[i].speed = request_read_double(r);
        radar->targets[i].azimuth = request_read_double(r);
      }
      break;
    case C_CONFIGURE:
      radar->min_range = request_read_double(r);
      radar->max_range = request_read_double(r);
      radar->horizontal_fov = request_read_double(r);
      radar->vertical_fov = request_read_double(r);
      break;
    default:
      ROBOT_ASSERT(0);  // should not be reached
      break;
  }
}

static void radar_write_request(WbDevice *d, WbRequest *r) {
  Radar *radar = d->pdata;
  if (radar->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, radar->sampling_period);
    radar->enable = false;  // done
  }
}

static void radar_cleanup(WbDevice *d) {
  Radar *radar = d->pdata;
  free(radar->targets);
  free(d->pdata);
}

static void radar_toggle_remote(WbDevice *d, WbRequest *r) {
  Radar *radar = d->pdata;
  if (radar->sampling_period != 0)
    radar->enable = true;
}

void wbr_radar_set_targets(WbDeviceTag tag, const WbRadarTarget *targets, int target_number) {
  Radar *radar = radar_get_struct(tag);
  if (radar) {
    radar->target_number = target_number;
    free(radar->targets);
    radar->targets = (WbRadarTarget *)malloc(radar->target_number * sizeof(WbRadarTarget));
    int i = 0;
    for (i = 0; i < radar->target_number; ++i) {
      radar->targets[i].distance = targets[i].distance;
      radar->targets[i].received_power = targets[i].received_power;
      radar->targets[i].speed = targets[i].speed;
      radar->targets[i].azimuth = targets[i].azimuth;
    }
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

// Protected functions (exported to device.cc)

void wb_radar_init(WbDevice *d) {
  d->pdata = radar_create();
  d->write_request = radar_write_request;
  d->read_answer = radar_read_answer;
  d->cleanup = radar_cleanup;
  d->toggle_remote = radar_toggle_remote;
}

// Public function available from the user API

void wb_radar_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  Radar *radar = radar_get_struct(tag);
  if (radar) {
    radar->sampling_period = sampling_period;
    radar->enable = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_radar_disable(WbDeviceTag tag) {
  Radar *radar = radar_get_struct(tag);
  if (radar)
    wb_radar_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_radar_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  Radar *radar = radar_get_struct(tag);
  if (radar)
    sampling_period = radar->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

int wb_radar_get_number_of_targets(WbDeviceTag tag) {
  int result = 0;
  robot_mutex_lock();
  Radar *radar = radar_get_struct(tag);
  if (radar) {
    if (radar->sampling_period == 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_radar_enable().\n", __FUNCTION__);
    result = radar->target_number;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const WbRadarTarget *wb_radar_get_targets(WbDeviceTag tag) {
  const WbRadarTarget *result = 0;
  robot_mutex_lock();
  Radar *radar = radar_get_struct(tag);
  if (radar) {
    if (radar->sampling_period == 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_radar_enable().\n", __FUNCTION__);
    result = radar->targets;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

double wb_radar_get_min_range(WbDeviceTag tag) {
  double result = 0;
  robot_mutex_lock();
  Radar *radar = radar_get_struct(tag);
  if (radar)
    result = radar->min_range;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

double wb_radar_get_max_range(WbDeviceTag tag) {
  double result = 0;
  robot_mutex_lock();
  Radar *radar = radar_get_struct(tag);
  if (radar)
    result = radar->max_range;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

double wb_radar_get_horizontal_fov(WbDeviceTag tag) {
  double result = 0;
  robot_mutex_lock();
  Radar *radar = radar_get_struct(tag);
  if (radar)
    result = radar->horizontal_fov;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

double wb_radar_get_vertical_fov(WbDeviceTag tag) {
  double result = 0;
  robot_mutex_lock();
  Radar *radar = radar_get_struct(tag);
  if (radar)
    result = radar->vertical_fov;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const WbRadarTarget *wb_radar_get_target(WbDeviceTag tag, int index) {
  Radar *radar = radar_get_struct(tag);
  if (radar)
    return (wb_radar_get_targets(tag) + index);

  fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  return NULL;
}
