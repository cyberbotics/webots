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

// ***************************************************************************
// this file contains the API interface for the PositionSensor device
// ***************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/nodes.h>
#include <webots/position_sensor.h>
#include "messages.h"
#include "robot_private.h"

typedef struct {
  bool enable;  // need to enable device ?
  int sampling_period;
  double position;
  WbJointType type;
  int requested_device_type;
  int requested_device_tag;
} PositionSensor;

static PositionSensor *position_sensor_create() {
  PositionSensor *position_sensor = malloc(sizeof(PositionSensor));
  position_sensor->enable = false;
  position_sensor->sampling_period = 0;
  position_sensor->position = NAN;
  position_sensor->type = WB_ROTATIONAL;
  position_sensor->requested_device_type = 0;
  position_sensor->requested_device_tag = 0;
  return position_sensor;
}

// Static functions
static PositionSensor *position_sensor_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_POSITION_SENSOR, true);
  return d ? d->pdata : NULL;
}

static void position_sensor_write_request(WbDevice *d, WbRequest *r) {
  PositionSensor *p = (PositionSensor *)d->pdata;
  if (p->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, p->sampling_period);
    p->enable = false;  // clear it for next time
  }
  if (p->requested_device_type != 0) {
    request_write_uchar(r, C_POSITION_SENSOR_GET_ASSOCIATED_DEVICE);
    request_write_uint16(r, p->requested_device_type);
    p->requested_device_type = 0;
  }
}

static void position_sensor_read_answer(WbDevice *d, WbRequest *r) {
  PositionSensor *p = (PositionSensor *)d->pdata;
  switch (request_read_uchar(r)) {
    case C_POSITION_SENSOR_DATA:  // read position feedback
      p->position = request_read_double(r);
      break;
    case C_POSITION_SENSOR_GET_ASSOCIATED_DEVICE:
      p->requested_device_tag = request_read_uint16(r);
      break;
    case C_CONFIGURE:
      p->type = request_read_int32(r);
      break;
    default:
      ROBOT_ASSERT(0);  // should not be reached
      break;
  }
}

static void position_sensor_cleanup(WbDevice *d) {
  PositionSensor *p = (PositionSensor *)d->pdata;
  free(p);
}

static void position_sensor_toggle_remote(WbDevice *d, WbRequest *r) {
  PositionSensor *p = (PositionSensor *)d->pdata;
  if (p->sampling_period != 0)
    p->enable = true;
}

void wbr_position_sensor_set_value(WbDeviceTag tag, const double value) {
  PositionSensor *p = position_sensor_get_struct(tag);
  if (p) {
    p->position = value;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

// Exported functions

void wb_position_sensor_init(WbDevice *d) {
  d->read_answer = position_sensor_read_answer;
  d->write_request = position_sensor_write_request;
  d->cleanup = position_sensor_cleanup;
  d->toggle_remote = position_sensor_toggle_remote;
  d->pdata = position_sensor_create();
}

// Public functions (available from the user API)

void wb_position_sensor_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  PositionSensor *p = position_sensor_get_struct(tag);
  if (p) {
    p->enable = true;
    p->sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_position_sensor_disable(WbDeviceTag tag) {
  const PositionSensor *p = position_sensor_get_struct(tag);
  if (p)
    wb_position_sensor_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

double wb_position_sensor_get_value(WbDeviceTag tag) {
  double result = NAN;
  robot_mutex_lock();
  const PositionSensor *p = position_sensor_get_struct(tag);
  if (p) {
    if (p->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_position_sensor_enable().\n", __FUNCTION__);
    result = p->position;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

int wb_position_sensor_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  const PositionSensor *p = position_sensor_get_struct(tag);
  if (p) {
    sampling_period = p->sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

WbJointType wb_position_sensor_get_type(WbDeviceTag tag) {
  WbJointType type = WB_ROTATIONAL;
  robot_mutex_lock();
  const PositionSensor *p = position_sensor_get_struct(tag);
  if (p) {
    type = p->type;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return type;
}

static WbDeviceTag position_sensor_get_associated_device(WbDeviceTag tag, int device_type, const char *function_name) {
  PositionSensor *p = position_sensor_get_struct(tag);
  if (!p) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", function_name);
    return 0;
  }

  robot_mutex_lock();
  p->requested_device_type = device_type;
  wb_robot_flush_unlocked(function_name);
  WbDeviceTag result = p->requested_device_tag;
  robot_mutex_unlock();
  return result;
}

WbDeviceTag wb_position_sensor_get_motor(WbDeviceTag tag) {
  // this function works for both linear and rotational motors
  return position_sensor_get_associated_device(tag, WB_NODE_ROTATIONAL_MOTOR, __FUNCTION__);
}

WbDeviceTag wb_position_sensor_get_brake(WbDeviceTag tag) {
  return position_sensor_get_associated_device(tag, WB_NODE_BRAKE, __FUNCTION__);
}
