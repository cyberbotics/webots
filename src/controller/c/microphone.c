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
#include <stdlib.h>
#include <string.h>
#include <webots/microphone.h>
#include <webots/nodes.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

typedef struct {
  int enable : 1;       // need to enable device ?
  int sampling_period;  // milliseconds
  double aperture;
  double sensitivity;
  void *sample;     // received sample
  int sample_size;  // received sample size
} Microphone;

static Microphone *microphone_create() {
  Microphone *mic = malloc(sizeof(Microphone));
  mic->enable = 0;
  mic->sampling_period = 0;
  mic->aperture = -1.0;
  mic->sensitivity = -1.0;
  mic->sample = NULL;
  return mic;
}

static void microphone_destroy(Microphone *mic) {
  free(mic->sample);
  free(mic);
}

// Static functions

static inline Microphone *microphone_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_MICROPHONE, true);
  return d ? d->pdata : NULL;
}

static void microphone_read_answer(WbDevice *d, WbRequest *r) {
  Microphone *mic = (Microphone *)d->pdata;
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:
      mic->aperture = request_read_double(r);
      mic->sensitivity = request_read_double(r);
      break;

    case C_MICROPHONE_RECEIVE:
      mic->sample_size = request_read_int32(r);
      free(mic->sample);
      mic->sample = malloc(mic->sample_size);
      memcpy(mic->sample, request_read_data(r, mic->sample_size), mic->sample_size);
      break;

    default:
      ROBOT_ASSERT(0);
  }
}

static void microphone_write_request(WbDevice *d, WbRequest *r) {
  Microphone *mic = (Microphone *)d->pdata;
  if (mic->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, mic->sampling_period);
    mic->enable = 0;  // done
  }
}

static void microphone_cleanup(WbDevice *d) {
  microphone_destroy((Microphone *)d->pdata);
}

int wb_microphone_get_sampling_period(WbDeviceTag tag) {
  int sampling_period = 0;
  robot_mutex_lock();
  Microphone *mic = microphone_get_struct(tag);
  if (mic)
    sampling_period = mic->sampling_period;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return sampling_period;
}

static void microphone_toggle_remote(WbDevice *d, WbRequest *r) {
  Microphone *mic = (Microphone *)d->pdata;
  if (mic->sampling_period != 0)
    mic->enable = 1;
}

void wbr_microphone_set_buffer(WbDeviceTag t, const unsigned char *buffer, int size) {
  Microphone *mic = microphone_get_struct(t);
  if (mic) {
    mic->sample_size = size;
    free(mic->sample);
    mic->sample = malloc(mic->sample_size);
    memcpy(mic->sample, buffer, mic->sample_size);
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

// Exported functions

void wb_microphone_init(WbDevice *d) {
  d->read_answer = microphone_read_answer;
  d->write_request = microphone_write_request;
  d->cleanup = microphone_cleanup;
  d->pdata = microphone_create();
  d->toggle_remote = microphone_toggle_remote;
}

// Public functions available from the user API

void wb_microphone_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  Microphone *mic = microphone_get_struct(tag);
  if (mic) {
    mic->enable = 1;
    mic->sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_microphone_disable(WbDeviceTag tag) {
  Microphone *mic = microphone_get_struct(tag);
  if (mic)
    wb_microphone_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

const void *wb_microphone_get_sample_data(WbDeviceTag tag) {
  const void *result = NULL;
  robot_mutex_lock();
  Microphone *mic = microphone_get_struct(tag);
  if (mic) {
    if (mic->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_microphone_enable().\n", __FUNCTION__);
    result = mic->sample;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

int wb_microphone_get_sample_size(WbDeviceTag tag) {
  int result = -1;
  robot_mutex_lock();
  Microphone *mic = microphone_get_struct(tag);
  if (mic) {
    if (mic->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_microphone_enable().\n", __FUNCTION__);
    result = mic->sample_size;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}
