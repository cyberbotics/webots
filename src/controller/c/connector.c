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
// this file is the API code for the Connector device
//***************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <webots/connector.h>
#include <webots/nodes.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool set_locked_state;
  bool is_locked;
  bool enable_presence;
  int presence_sampling_period;
  int presence;
} Connector;

static Connector *connector_create() {
  Connector *con = malloc(sizeof(Connector));
  con->set_locked_state = false;
  con->is_locked = false;
  con->enable_presence = false;
  con->presence_sampling_period = 0;
  return con;
}

static Connector *connector_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_CONNECTOR, true);
  return d ? d->pdata : NULL;
}

static void connector_write_request(WbDevice *d, WbRequest *r) {
  Connector *con = d->pdata;
  if (con->enable_presence) {
    request_write_uchar(r, C_CONNECTOR_GET_PRESENCE);
    request_write_uint16(r, con->presence_sampling_period);
    con->enable_presence = false;
  }
  if (con->set_locked_state) {
    request_write_uchar(r, con->is_locked ? C_CONNECTOR_LOCK : C_CONNECTOR_UNLOCK);
    con->set_locked_state = false;
  }
}

static void connector_read_answer(WbDevice *d, WbRequest *r) {
  Connector *con = d->pdata;
  switch (request_read_uchar(r)) {
    case C_CONNECTOR_GET_PRESENCE:
      con->presence = request_read_int16(r);
      break;
    case C_CONFIGURE:
      con->is_locked = request_read_uchar(r);
      break;
    default:
      ROBOT_ASSERT(0);
  }
}

static void connector_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void connector_toggle_remote(WbDevice *d, WbRequest *r) {
  Connector *con = d->pdata;
  if (con->presence_sampling_period != 0)
    con->enable_presence = true;
  con->set_locked_state = true;
}

// Exported functions

void wb_connector_init(WbDevice *d) {
  d->read_answer = connector_read_answer;
  d->write_request = connector_write_request;
  d->cleanup = connector_cleanup;
  d->pdata = connector_create();
  d->toggle_remote = connector_toggle_remote;
}

// Public functions (available from the user API)

void wb_connector_enable_presence(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  Connector *con = connector_get_struct(tag);
  if (con) {
    con->enable_presence = true;
    con->presence_sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_connector_disable_presence(WbDeviceTag tag) {
  Connector *con = connector_get_struct(tag);
  if (con)
    wb_connector_enable_presence(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_connector_get_presence_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  Connector *con = connector_get_struct(tag);
  if (con)
    sampling_period = con->presence_sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

void wb_connector_lock(WbDeviceTag tag) {
  robot_mutex_lock();
  Connector *con = connector_get_struct(tag);
  if (con) {
    con->set_locked_state = true;
    con->is_locked = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_connector_unlock(WbDeviceTag tag) {
  robot_mutex_lock();
  Connector *con = connector_get_struct(tag);
  if (con) {
    con->set_locked_state = true;
    con->is_locked = false;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

int wb_connector_get_presence(WbDeviceTag tag) {
  int result = -1;
  robot_mutex_lock();
  Connector *con = connector_get_struct(tag);
  if (con) {
    if (con->presence_sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_connector_enable_presence().\n", __FUNCTION__);
    result = con->presence;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

bool wb_connector_is_locked(WbDeviceTag tag) {
  bool result;
  robot_mutex_lock();
  Connector *con = connector_get_struct(tag);
  if (con)
    result = con->is_locked;
  else {
    result = false;
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  }
  robot_mutex_unlock();
  return result;
}
