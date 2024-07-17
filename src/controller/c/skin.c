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

#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/skin.h>
#include <webots/types.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct BoneRequest {
  int type;
  int bone_index;
  double values[4];
  bool absolute;
  struct BoneRequest *next;
} BoneRequest;

typedef struct {
  BoneRequest *root_request;
  BoneRequest *last_request;
  int bone_count;
  char **bone_names;
  double *bone_position;
  double *bone_orientation;
} Skin;

static Skin *skin_create() {
  Skin *skin = malloc(sizeof(Skin));
  skin->root_request = NULL;
  skin->last_request = NULL;
  skin->bone_count = 0;
  skin->bone_position = malloc(3 * sizeof(double));
  skin->bone_orientation = malloc(4 * sizeof(double));
  return skin;
}

static Skin *skin_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_SKIN, true);
  return d ? d->pdata : NULL;
}

static void skin_read_answer(WbDevice *d, WbRequest *r) {
  int bone_count;
  Skin *skin = (Skin *)d->pdata;
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:
      bone_count = request_read_uint32(r);
      skin->bone_count = bone_count;
      skin->bone_names = malloc((bone_count) * sizeof(char *));
      int i = 0;
      for (i = 0; i < bone_count; ++i)
        skin->bone_names[i] = request_read_string(r);
      break;
    case C_SKIN_GET_BONE_POSITION:
      skin->bone_position[0] = request_read_double(r);
      skin->bone_position[1] = request_read_double(r);
      skin->bone_position[2] = request_read_double(r);
      break;
    case C_SKIN_GET_BONE_ORIENTATION:
      skin->bone_orientation[0] = request_read_double(r);
      skin->bone_orientation[1] = request_read_double(r);
      skin->bone_orientation[2] = request_read_double(r);
      skin->bone_orientation[3] = request_read_double(r);
      break;
    default:
      ROBOT_ASSERT(0);
      break;
  }
}

static void skin_write_request(WbDevice *d, WbRequest *r) {
  Skin *skin = (Skin *)d->pdata;

  BoneRequest *request = skin->root_request;
  while (request != NULL) {
    request_write_uchar(r, request->type);
    request_write_uint16(r, (unsigned short int)(request->bone_index));
    if (request->type == C_SKIN_SET_BONE_ORIENTATION || request->type == C_SKIN_SET_BONE_POSITION) {
      request_write_double(r, request->values[0]);
      request_write_double(r, request->values[1]);
      request_write_double(r, request->values[2]);
      if (request->type == C_SKIN_SET_BONE_ORIENTATION)
        request_write_double(r, request->values[3]);  // angle
    }
    request_write_uchar(r, request->absolute);
    BoneRequest *previous = request;
    request = request->next;
    free(previous);
  }
  skin->last_request = NULL;
  skin->root_request = NULL;
}

static void skin_cleanup(WbDevice *d) {
  Skin *skin = (Skin *)d->pdata;
  if (skin == NULL)
    return;
  BoneRequest *request = skin->root_request;
  while (request != NULL) {
    BoneRequest *previous = request;
    request = request->next;
    free(previous);
  }
  skin->root_request = NULL;
  skin->last_request = NULL;
  int i;
  for (i = 0; i < skin->bone_count; ++i)
    free(skin->bone_names[i]);
  free(skin->bone_names);
  free(skin->bone_position);
  free(skin->bone_orientation);
  free(skin);
  d->pdata = NULL;
}

static void add_request(Skin *skin, BoneRequest *request) {
  if (skin->root_request == NULL)
    skin->root_request = request;
  if (skin->last_request != NULL)
    skin->last_request->next = request;
  skin->last_request = request;
}

// Protected functions (exported to WbDevice.cc)

void wb_skin_init(WbDevice *d) {
  d->pdata = skin_create();
  d->write_request = skin_write_request;
  d->read_answer = skin_read_answer;
  d->cleanup = skin_cleanup;
}

// Public functions available from the user API

void wb_skin_set_bone_orientation(WbDeviceTag tag, int index, const double orientation[4], bool absolute) {
  int i;
  for (i = 0; i < 4; i++) {
    if (isnan(orientation[i])) {
      fprintf(stderr, "Error: %s() called with a NaN orientation value.\n", __FUNCTION__);
      return;
    }
  }
  // Check if axis is valid
  if ((orientation[0] == 0.0 && orientation[1] == 0.0 && orientation[2] == 0.0)) {
    fprintf(stderr, "Error: %s() called with invalid values for the [x y z] orientation axis.\n", __FUNCTION__);
    return;
  }
  robot_mutex_lock();
  Skin *skin = skin_get_struct(tag);
  // Check if joint index is valid
  if (skin) {
    if (index < 0 || index >= skin->bone_count) {
      fprintf(stderr, "Error: The index of %s() is out of the bounds.\n", __FUNCTION__);
      robot_mutex_unlock();
      return;
    }
    BoneRequest *request = malloc(sizeof(BoneRequest));
    request->type = C_SKIN_SET_BONE_ORIENTATION;
    request->bone_index = index;
    request->values[0] = orientation[0];
    request->values[1] = orientation[1];
    request->values[2] = orientation[2];
    request->values[3] = orientation[3];
    request->absolute = absolute;
    request->next = NULL;
    add_request(skin, request);
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  wb_robot_flush_unlocked(__FUNCTION__);
  robot_mutex_unlock();
}

void wb_skin_set_bone_position(WbDeviceTag tag, int index, const double position[3], bool absolute) {
  int i;
  for (i = 0; i < 3; i++) {
    if (isnan(position[i])) {
      fprintf(stderr, "Error: %s() called with a NaN value.\n", __FUNCTION__);
      return;
    }
  }

  robot_mutex_lock();
  Skin *skin = skin_get_struct(tag);
  // Check if joint index is valid
  if (skin) {
    if (index < 0 || index >= skin->bone_count) {
      fprintf(stderr, "Error: The index of %s() is out of the bounds.\n", __FUNCTION__);
      robot_mutex_unlock();
      return;
    }
    BoneRequest *request = malloc(sizeof(BoneRequest));
    request->type = C_SKIN_SET_BONE_POSITION;
    request->bone_index = index;
    request->values[0] = position[0];
    request->values[1] = position[1];
    request->values[2] = position[2];
    request->absolute = absolute;
    request->next = NULL;
    add_request(skin, request);
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  wb_robot_flush_unlocked(__FUNCTION__);
  robot_mutex_unlock();
}

int wb_skin_get_bone_count(WbDeviceTag tag) {
  int result = 0;
  robot_mutex_lock();
  const Skin *skin = skin_get_struct(tag);
  if (skin)
    result = skin->bone_count;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const char *wb_skin_get_bone_name(WbDeviceTag tag, int index) {
  char *result = NULL;
  robot_mutex_lock();
  Skin *skin = skin_get_struct(tag);
  // Check if joint index is valid
  if (skin) {
    if (index < 0 || index >= skin->bone_count) {
      robot_mutex_unlock();
      fprintf(stderr, "Error: The index of %s() is out of the bounds.\n", __FUNCTION__);
      return result;
    }
    result = skin->bone_names[index];
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

const double *wb_skin_get_bone_position(WbDeviceTag tag, int index, bool absolute) {
  Skin *skin = skin_get_struct(tag);
  if (!skin) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return NULL;
  }

  robot_mutex_lock();
  // Check if joint index is valid
  if (index < 0 || index >= skin->bone_count) {
    robot_mutex_unlock();
    fprintf(stderr, "Error: The index of %s() is out of the bounds.\n", __FUNCTION__);
    return NULL;
  }

  BoneRequest *request = malloc(sizeof(BoneRequest));
  request->type = C_SKIN_GET_BONE_POSITION;
  request->bone_index = index;
  request->absolute = absolute;
  request->next = NULL;
  add_request(skin, request);
  wb_robot_flush_unlocked(__FUNCTION__);

  robot_mutex_unlock();
  return skin->bone_position;
}

const double *wb_skin_get_bone_orientation(WbDeviceTag tag, int index, bool absolute) {
  Skin *skin = skin_get_struct(tag);
  if (!skin) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return NULL;
  }

  robot_mutex_lock();
  // Check if joint index is valid
  if (index < 0 || index >= skin->bone_count) {
    robot_mutex_unlock();
    fprintf(stderr, "Error: The index of %s() is out of the bounds.\n", __FUNCTION__);
    return NULL;
  }

  BoneRequest *request = malloc(sizeof(BoneRequest));
  request->type = C_SKIN_GET_BONE_ORIENTATION;
  request->bone_index = index;
  request->absolute = absolute;
  request->next = NULL;
  add_request(skin, request);
  wb_robot_flush_unlocked(__FUNCTION__);

  robot_mutex_unlock();
  return skin->bone_orientation;
}
