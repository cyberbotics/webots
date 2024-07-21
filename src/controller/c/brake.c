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

//***************************************************************************
// this file contains the API interface for the Brake device
//***************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/brake.h>
#include <webots/nodes.h>
#include "messages.h"
#include "robot_private.h"

typedef struct {
  int state;
  WbJointType type;
  double damping_constant;
  int requested_device_type;
  int requested_device_tag;
} Brake;

static Brake *brake_create() {
  Brake *brake = malloc(sizeof(Brake));
  brake->state = 0;
  brake->type = WB_ROTATIONAL;
  brake->damping_constant = 0.0;
  brake->requested_device_type = 0;
  brake->requested_device_tag = 0;
  return brake;
}

// Static functions
static Brake *brake_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_BRAKE, true);
  return d ? d->pdata : NULL;
}

static void brake_write_request(WbDevice *d, WbRequest *r) {
  Brake *b = (Brake *)d->pdata;
  if (b->state) {
    request_write_uchar(r, b->state);
    if (b->state & C_BRAKE_SET_DAMPING_CONSTANT)
      request_write_double(r, b->damping_constant);
    if (b->state & C_BRAKE_GET_ASSOCIATED_DEVICE) {
      request_write_uint16(r, b->requested_device_type);
      b->requested_device_type = 0;
    }
    b->state = 0;  // clear it for next time
  }
}

static void brake_read_answer(WbDevice *d, WbRequest *r) {
  Brake *b = (Brake *)d->pdata;
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:
      b->type = request_read_int32(r);
      break;
    case C_BRAKE_GET_ASSOCIATED_DEVICE:
      b->requested_device_tag = request_read_uint16(r);
      break;
    default:
      ROBOT_ASSERT(0);  // should not be reached
      break;
  }
}

static void brake_cleanup(WbDevice *d) {
  Brake *b = (Brake *)d->pdata;
  free(b);
}

static void brake_toggle_remote(WbDevice *d, WbRequest *r) {
  // nop: reseting the position_sensor is often problematic
}

// Exported functions

void wb_brake_init(WbDevice *d) {
  d->read_answer = brake_read_answer;
  d->write_request = brake_write_request;
  d->cleanup = brake_cleanup;
  d->toggle_remote = brake_toggle_remote;
  d->pdata = brake_create();
}

// Public functions (available from the user API)

void wb_brake_set_damping_constant_no_mutex(WbDeviceTag tag, double damping_constant) {
  Brake *b = brake_get_struct(tag);
  if (b) {
    b->state |= C_BRAKE_SET_DAMPING_CONSTANT;
    b->damping_constant = damping_constant;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

void wb_brake_set_damping_constant(WbDeviceTag tag, double damping_constant) {
  if (isnan(damping_constant)) {
    fprintf(stderr, "Error: %s() called with an invalid 'damping_constant' argument (NaN).\n", __FUNCTION__);
    return;
  }

  const Brake *b = brake_get_struct(tag);
  if (!b) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  wb_brake_set_damping_constant_no_mutex(tag, damping_constant);
  robot_mutex_unlock();
}

WbJointType wb_brake_get_type(WbDeviceTag tag) {
  WbJointType type = WB_ROTATIONAL;
  robot_mutex_lock();
  const Brake *b = brake_get_struct(tag);
  if (b)
    type = b->type;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return type;
}

static WbDeviceTag brake_get_associated_device(WbDeviceTag t, int device_type, const char *function_name) {
  Brake *b = brake_get_struct(t);
  if (!b) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", function_name);
    return 0;
  }

  robot_mutex_lock();
  b->state |= C_BRAKE_GET_ASSOCIATED_DEVICE;
  b->requested_device_type = device_type;
  wb_robot_flush_unlocked(function_name);
  WbDeviceTag result = b->requested_device_tag;
  robot_mutex_unlock();
  return result;
}

WbDeviceTag wb_brake_get_motor(WbDeviceTag tag) {
  // this function works for both linear and rotational motors
  return brake_get_associated_device(tag, WB_NODE_ROTATIONAL_MOTOR, __FUNCTION__);
}

WbDeviceTag wb_brake_get_position_sensor(WbDeviceTag tag) {
  return brake_get_associated_device(tag, WB_NODE_POSITION_SENSOR, __FUNCTION__);
}
