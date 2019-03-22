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
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

typedef struct {
  bool enable;          // need to enable device ?
  int sampling_period;  // milliseconds
  WbTouchSensorType type;
  double values[3];
} TouchSensor;

static TouchSensor *touch_sensor_create() {
  TouchSensor *ts = malloc(sizeof(TouchSensor));
  ts->enable = false;
  ts->type = WB_TOUCH_SENSOR_BUMPER;
  ts->sampling_period = 0;
  ts->values[0] = NAN;
  ts->values[1] = NAN;
  ts->values[2] = NAN;
  return ts;
}

static TouchSensor *touch_sensor_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_TOUCH_SENSOR, true);
  return d ? d->pdata : NULL;
}

static void touch_sensor_read_answer(WbDevice *d, WbRequest *r) {
  TouchSensor *ts = (TouchSensor *)d->pdata;
  switch (request_read_uchar(r)) {
    case C_TOUCH_SENSOR_DATA:
      ts->values[0] = request_read_double(r);
      break;
    case C_TOUCH_SENSOR_DATA_3D:
      ts->values[0] = request_read_double(r);
      ts->values[1] = request_read_double(r);
      ts->values[2] = request_read_double(r);
      break;
    case C_CONFIGURE:
      ts->type = request_read_int32(r);
      break;
    default:
      ROBOT_ASSERT(0);  // should never be reached
      break;
  }
}

static void touch_sensor_write_request(WbDevice *d, WbRequest *r) {
  TouchSensor *ts = (TouchSensor *)d->pdata;
  if (ts->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, ts->sampling_period);
    ts->enable = false;  // done
  }
}

static void touch_sensor_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void touch_sensor_toggle_remote(WbDevice *d, WbRequest *r) {
  TouchSensor *ts = (TouchSensor *)d->pdata;
  if (ts->sampling_period != 0)
    ts->enable = true;
}

void wbr_touch_sensor_set_value(WbDeviceTag tag, double value) {
  TouchSensor *ts = touch_sensor_get_struct(tag);
  if (ts) {
    if (ts->type != WB_TOUCH_SENSOR_BUMPER && ts->type != WB_TOUCH_SENSOR_FORCE) {
      fprintf(stderr, "Error: wbr_touch_sensor_set_value() must be used with a TouchSensor of type \"bumper\" or \"force\"\n");
      fprintf(stderr, "Error: you should use wbr_touch_sensor_set_values() instead.\n");
      return;
    }
    ts->values[0] = value;
  } else
    fprintf(stderr, "Error: wbr_touch_sensor_set_value(): invalid device tag.\n");
}

void wbr_touch_sensor_set_values(WbDeviceTag tag, const double *values) {
  TouchSensor *ts = touch_sensor_get_struct(tag);
  if (ts) {
    if (ts->type != WB_TOUCH_SENSOR_FORCE3D) {
      fprintf(stderr, "Error: wbr_touch_sensor_set_values() must be used with a TouchSensor of type \"force-3d\"\n");
      fprintf(stderr, "Error: you should use wbr_touch_sensor_set_value() instead.\n");
      return;
    }
    ts->values[0] = values[0];
    ts->values[1] = values[1];
    ts->values[2] = values[2];
  } else
    fprintf(stderr, "Error: wbr_touch_sensor_set_values(): invalid device tag.\n");
}

// Protected functions (exported to WbDevice.cc)

void wb_touch_sensor_init(WbDevice *d) {
  d->pdata = touch_sensor_create();
  d->write_request = touch_sensor_write_request;
  d->read_answer = touch_sensor_read_answer;
  d->cleanup = touch_sensor_cleanup;
  d->toggle_remote = touch_sensor_toggle_remote;
}

// Public function available from the user API

void wb_touch_sensor_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: wb_touch_sensor_enable() called with negative sampling period.\n");
    return;
  }

  robot_mutex_lock_step();
  TouchSensor *ts = touch_sensor_get_struct(tag);
  if (ts) {
    ts->sampling_period = sampling_period;
    ts->enable = true;
  } else
    fprintf(stderr, "Error: wb_touch_sensor_enable(): invalid device tag.\n");
  robot_mutex_unlock_step();
}

void wb_touch_sensor_disable(WbDeviceTag tag) {
  TouchSensor *ts = touch_sensor_get_struct(tag);
  if (ts)
    wb_touch_sensor_enable(tag, 0);
  else
    fprintf(stderr, "Error: wb_touch_sensor_disable(): invalid device tag.\n");
}

int wb_touch_sensor_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock_step();
  TouchSensor *ts = touch_sensor_get_struct(tag);
  if (ts)
    sampling_period = ts->sampling_period;
  else
    fprintf(stderr, "Error: wb_touch_sensor_get_sampling_period(): invalid device tag.\n");
  robot_mutex_unlock_step();
  return sampling_period;
}

double wb_touch_sensor_get_value(WbDeviceTag tag) {
  TouchSensor *ts = touch_sensor_get_struct(tag);

  double value = NAN;
  robot_mutex_lock_step();
  if (ts) {
    if (ts->type != WB_TOUCH_SENSOR_BUMPER && ts->type != WB_TOUCH_SENSOR_FORCE) {
      fprintf(stderr, "Error: wb_touch_sensor_get_value() must be used with a TouchSensor of type \"bumper\" or \"force\"\n");
      fprintf(stderr, "Error: you should use wb_touch_sensor_get_values() instead.\n");
    } else {
      if (ts->sampling_period <= 0)
        fprintf(stderr,
                "Error: wb_touch_sensor_get_value() called for a disabled device! Please use: wb_touch_sensor_enable().\n");
      value = ts->values[0];
    }
  } else
    fprintf(stderr, "Error: wb_touch_sensor_get_value(): invalid device tag.\n");
  robot_mutex_unlock_step();
  return value;
}

const double *wb_touch_sensor_get_values(WbDeviceTag tag) {
  TouchSensor *ts = touch_sensor_get_struct(tag);
  double *values = NULL;
  robot_mutex_lock_step();
  if (ts) {
    if (ts->type != WB_TOUCH_SENSOR_FORCE3D) {
      fprintf(stderr, "Error: wb_touch_sensor_get_values() must be used with a TouchSensor of type \"force-3d\"\n");
      fprintf(stderr, "Error: you should use wb_touch_sensor_get_value() instead.\n");
    } else {
      if (ts->sampling_period <= 0)
        fprintf(stderr,
                "Error: wb_touch_sensor_get_values() called for a disabled device! Please use: wb_touch_sensor_enable().\n");
      values = ts->values;
    }
  } else
    fprintf(stderr, "Error: wb_touch_sensor_get_values(): invalid device tag.\n");
  robot_mutex_unlock_step();
  return values;
}

WbTouchSensorType wb_touch_sensor_get_type(WbDeviceTag tag) {
  WbTouchSensorType result = WB_TOUCH_SENSOR_BUMPER;
  robot_mutex_lock_step();
  TouchSensor *ts = touch_sensor_get_struct(tag);
  if (ts)
    result = ts->type;
  else
    fprintf(stderr, "Error: wb_touch_sensor_get_type(): invalid device tag.\n");
  robot_mutex_unlock_step();
  return result;
}
