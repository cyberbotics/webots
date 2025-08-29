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

//***************************************************************************
// this file is the API code for the VacuumGripper device
//***************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <webots/nodes.h>
#include <webots/vacuum_gripper.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool toggle;
  bool is_on;
  bool enable_presence;
  int presence_sampling_period;
  bool presence;
} VacuumGripper;

static VacuumGripper *vacuum_gripper_create() {
  VacuumGripper *vc = malloc(sizeof(VacuumGripper));
  vc->toggle = false;
  vc->is_on = false;
  vc->enable_presence = false;
  vc->presence_sampling_period = 0;
  return vc;
}

static VacuumGripper *vacuum_gripper_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_VACUUM_GRIPPER, true);
  return d ? d->pdata : NULL;
}

static void vacuum_gripper_write_request(WbDevice *d, WbRequest *r) {
  VacuumGripper *vc = d->pdata;
  if (vc->enable_presence) {
    request_write_uchar(r, C_VACUUM_GRIPPER_GET_PRESENCE);
    request_write_uint16(r, vc->presence_sampling_period);
    vc->enable_presence = false;
  }
  if (vc->toggle) {
    request_write_uchar(r, vc->is_on ? C_VACUUM_GRIPPER_TURN_ON : C_VACUUM_GRIPPER_TURN_OFF);
    vc->toggle = false;
  }
}

static void vacuum_gripper_read_answer(WbDevice *d, WbRequest *r) {
  VacuumGripper *vc = d->pdata;
  switch (request_read_uchar(r)) {
    case C_VACUUM_GRIPPER_GET_PRESENCE:
      vc->presence = request_read_uchar(r) == 1;
      break;
    case C_CONFIGURE:
      vc->is_on = request_read_uchar(r);
      break;
    default:
      ROBOT_ASSERT(0);
  }
}

static void vacuum_gripper_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void vacuum_gripper_toggle_remote(WbDevice *d, WbRequest *r) {
  VacuumGripper *vc = d->pdata;
  if (vc->presence_sampling_period != 0)
    vc->enable_presence = true;
  vc->toggle = true;
}

// Exported functions

void wb_vacuum_gripper_init(WbDevice *d) {
  d->read_answer = vacuum_gripper_read_answer;
  d->write_request = vacuum_gripper_write_request;
  d->cleanup = vacuum_gripper_cleanup;
  d->pdata = vacuum_gripper_create();
  d->toggle_remote = vacuum_gripper_toggle_remote;
}

// Public functions (available from the user API)

void wb_vacuum_gripper_enable_presence(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  VacuumGripper *vc = vacuum_gripper_get_struct(tag);
  if (vc) {
    vc->enable_presence = true;
    vc->presence_sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_vacuum_gripper_disable_presence(WbDeviceTag tag) {
  const VacuumGripper *vc = vacuum_gripper_get_struct(tag);
  if (vc)
    wb_vacuum_gripper_enable_presence(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_vacuum_gripper_get_presence_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  const VacuumGripper *vc = vacuum_gripper_get_struct(tag);
  if (vc)
    sampling_period = vc->presence_sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

void wb_vacuum_gripper_turn_on(WbDeviceTag tag) {
  robot_mutex_lock();
  VacuumGripper *vc = vacuum_gripper_get_struct(tag);
  if (vc) {
    vc->toggle = true;
    vc->is_on = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_vacuum_gripper_turn_off(WbDeviceTag tag) {
  robot_mutex_lock();
  VacuumGripper *vc = vacuum_gripper_get_struct(tag);
  if (vc) {
    vc->toggle = true;
    vc->is_on = false;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

bool wb_vacuum_gripper_get_presence(WbDeviceTag tag) {
  bool result = false;
  robot_mutex_lock();
  const VacuumGripper *vc = vacuum_gripper_get_struct(tag);
  if (vc) {
    if (vc->presence_sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_vacuum_gripper_enable_presence().\n",
              __FUNCTION__);
    result = vc->presence;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

bool wb_vacuum_gripper_is_on(WbDeviceTag tag) {
  bool result;
  robot_mutex_lock();
  const VacuumGripper *vc = vacuum_gripper_get_struct(tag);
  if (vc)
    result = vc->is_on;
  else {
    result = false;
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  }
  robot_mutex_unlock();
  return result;
}
