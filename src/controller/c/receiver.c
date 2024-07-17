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

// ***************************************************************************
// this file implements a communication channel mechanism between controllers
// ***************************************************************************

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/nodes.h>
#include <webots/receiver.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"
#include "scheduler.h"

typedef struct _PacketStruct PacketStruct;

struct _PacketStruct {
  double dir[3];  // direction of emitter in receiver's coordinate system
  char *data;     // user data (packet body)
  int size;       // data size (not including header)
  double signal;  // signal strength
  PacketStruct *next;
};

static PacketStruct *packet_create() {
  PacketStruct *ps = malloc(sizeof(PacketStruct));
  ps->size = 0;
  ps->dir[0] = NAN;
  ps->dir[1] = NAN;
  ps->dir[2] = NAN;
  ps->signal = NAN;
  ps->data = NULL;
  ps->next = NULL;
  return ps;
}

static void packet_destroy(PacketStruct *ps) {
  free(ps->data);
  free(ps);
}

typedef struct {
  int enable : 1;             // need to enable device ?
  int set_channel : 1;        // need to change receiver's channel ?
  int sampling_period;        // milliseconds
  int channel;                // receiver's channel
  int *allowed_channels;      // allowed channels receiver is allowed to listen to
  PacketStruct *queue;        // reception queue
  int queue_length;           // number of packets in the reception queue
  int buffer_size;            // reception buffer size (as in Receiver.bufferSize)
  int buffer_used;            // sum of all packet data size
  int allowed_channels_size;  // size of allowed_channels array
} Receiver;

static Receiver *receiver_create() {
  Receiver *rs = malloc(sizeof(Receiver));
  rs->enable = false;
  rs->set_channel = false;
  rs->sampling_period = 0;
  rs->channel = -1;
  rs->allowed_channels = NULL;
  rs->queue = NULL;
  rs->queue_length = 0;
  rs->buffer_size = -1;
  rs->buffer_used = 0;
  rs->allowed_channels_size = -1;
  return rs;
}

static void receiver_empty_queue(Receiver *rs) {
  PacketStruct *ps = rs->queue;
  while (ps) {
    PacketStruct *garbage = ps;
    ps = ps->next;
    packet_destroy(garbage);
  }
  rs->queue = NULL;
  rs->queue_length = 0;
  rs->buffer_used = 0;
}

static void receiver_enqueue(Receiver *rs, PacketStruct *ps) {
  if (rs->queue) {
    PacketStruct *i = rs->queue;
    while (i->next)
      i = i->next;
    i->next = ps;
  } else
    rs->queue = ps;

  rs->queue_length++;
  rs->buffer_used += ps->size;
}

static PacketStruct *receiver_dequeue(Receiver *rs) {
  ROBOT_ASSERT(rs->queue);
  PacketStruct *ps = rs->queue;
  // cppcheck-suppress nullPointerRedundantCheck
  rs->queue = rs->queue->next;
  // cppcheck-suppress nullPointerRedundantCheck
  rs->buffer_used -= ps->size;
  rs->queue_length--;
  return ps;
}

// Static functions

static Receiver *receiver_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_RECEIVER, true);
  return d ? d->pdata : NULL;
}

static void receiver_read_answer(WbDevice *d, WbRequest *r) {
  Receiver *rs = d->pdata;
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:
      rs->buffer_size = request_read_int32(r);
      rs->channel = request_read_int32(r);
      rs->allowed_channels_size = request_read_int32(r);
      rs->allowed_channels = (int *)realloc(rs->allowed_channels, rs->allowed_channels_size * sizeof(int));
      for (int i = 0; i < rs->allowed_channels_size; i++)
        rs->allowed_channels[i] = request_read_int32(r);
      break;

    case C_RECEIVER_RECEIVE: {
      // copy receiver channel number from incoming message
      rs->channel = request_read_int32(r);

      // read packet header
      PacketStruct *ps = packet_create();
      ps->dir[0] = request_read_double(r);
      ps->dir[1] = request_read_double(r);
      ps->dir[2] = request_read_double(r);
      ps->signal = request_read_double(r);

      // read packet body
      ps->size = request_read_int32(r);
      ps->data = malloc(ps->size);
      memcpy(ps->data, request_read_data(r, ps->size), ps->size);

      // enqueue packet if there's enough buffer space
      if (rs->buffer_size == -1 || ps->size <= rs->buffer_size - rs->buffer_used)
        receiver_enqueue(rs, ps);
      else
        packet_destroy(ps);

      break;
    }
    default:
      ROBOT_ASSERT(0);
  }
}

static void receiver_write_request(WbDevice *d, WbRequest *r) {
  Receiver *rs = d->pdata;
  if (rs->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, rs->sampling_period);
    rs->enable = false;  // done
  }
  if (rs->set_channel) {
    request_write_uchar(r, C_RECEIVER_SET_CHANNEL);
    request_write_uint32(r, rs->channel);
    rs->set_channel = false;  // done
  }
}

static void receiver_cleanup(WbDevice *d) {
  Receiver *rs = d->pdata;
  receiver_empty_queue(rs);
  free(rs->allowed_channels);
  free(rs);
}

// Exported functions

void wb_receiver_init(WbDevice *d) {
  d->read_answer = receiver_read_answer;
  d->write_request = receiver_write_request;
  d->cleanup = receiver_cleanup;
  d->pdata = receiver_create();
}

// Public functions available from the user API

void wb_receiver_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }
  robot_mutex_lock();
  Receiver *rs = receiver_get_struct(tag);
  if (!rs)
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  else {
    rs->enable = true;
    rs->sampling_period = sampling_period;
  }
  robot_mutex_unlock();
}

void wb_receiver_disable(WbDeviceTag tag) {
  const Receiver *rs = receiver_get_struct(tag);
  if (!rs)
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  else
    wb_receiver_enable(tag, 0);
}

int wb_receiver_get_sampling_period(WbDeviceTag tag) {
  int sampling_period;
  robot_mutex_lock();
  const Receiver *rs = receiver_get_struct(tag);
  if (!rs) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    sampling_period = 0;
  } else
    sampling_period = rs->sampling_period;
  robot_mutex_unlock();
  return sampling_period;
}

void wb_receiver_set_channel(WbDeviceTag tag, int channel) {
  if (channel < -1) {
    fprintf(stderr, "Error: %s() called with an invalid channel=%d. Please use a channel inside the range [-1,inf).\n",
            __FUNCTION__, channel);
    return;
  }

  robot_mutex_lock();
  Receiver *rs = receiver_get_struct(tag);
  if (!rs)
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  else if (channel != rs->channel) {
    bool is_allowed = true;

    if (rs->allowed_channels_size > 0) {
      is_allowed = false;
      for (int i = 0; i < rs->allowed_channels_size; i++) {
        if (rs->allowed_channels[i] == channel) {
          is_allowed = true;
          break;
        }
      }
    }

    if (!is_allowed)
      fprintf(stderr,
              "Error: %s() called with channel=%d, which is not between allowed channels. Please use an allowed channel.\n",
              __FUNCTION__, channel);
    else {
      rs->set_channel = true;
      rs->channel = channel;
    }
  }
  robot_mutex_unlock();
}

int wb_receiver_get_channel(WbDeviceTag tag) {
  int result;
  robot_mutex_lock();
  const Receiver *rs = receiver_get_struct(tag);
  if (!rs) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    result = -1;
  } else
    result = rs->channel;
  robot_mutex_unlock();
  return result;
}

void wb_receiver_next_packet(WbDeviceTag tag) {
  robot_mutex_lock();
  Receiver *rs = receiver_get_struct(tag);
  if (!rs)
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  else if (rs->queue)
    packet_destroy(receiver_dequeue(rs));
  robot_mutex_unlock();
}

int wb_receiver_get_queue_length(WbDeviceTag tag) {
  int result;
  robot_mutex_lock();
  const Receiver *rs = receiver_get_struct(tag);
  if (!rs) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    result = -1;
  } else {
    if (rs->sampling_period <= 0)
      fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_receiver_enable().\n", __FUNCTION__);
    result = rs->queue_length;
  }
  robot_mutex_unlock();
  return result;
}

int wb_receiver_get_data_size(WbDeviceTag tag) {
  int result;
  robot_mutex_lock();
  Receiver *rs = receiver_get_struct(tag);
  if (!rs) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    result = -1;
  } else if (rs->queue)
    result = rs->queue->size;
  else {
    fprintf(stderr, "Error: %s(): the receiver queue is empty.\n", __FUNCTION__);
    result = -1;
  }
  robot_mutex_unlock();
  return result;
}

const void *wb_receiver_get_data(WbDeviceTag tag) {
  const void *result;
  robot_mutex_lock();
  Receiver *rs = receiver_get_struct(tag);
  if (!rs) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    result = NULL;
  } else if (rs->queue)
    result = rs->queue->data;
  else {
    fprintf(stderr, "Error: %s(): the receiver queue is empty.\n", __FUNCTION__);
    result = NULL;
  }
  robot_mutex_unlock();
  return result;
}

const double *wb_receiver_get_emitter_direction(WbDeviceTag tag) {
  const double *result;
  robot_mutex_lock();
  Receiver *rs = receiver_get_struct(tag);
  if (!rs) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    result = NULL;
  } else if (rs->queue)
    result = rs->queue->dir;
  else {
    fprintf(stderr, "Error: %s(): the receiver queue is empty.\n", __FUNCTION__);
    result = NULL;
  }
  robot_mutex_unlock();
  return result;
}

double wb_receiver_get_signal_strength(WbDeviceTag tag) {
  double result;
  robot_mutex_lock();
  Receiver *rs = receiver_get_struct(tag);
  if (!rs) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    result = NAN;
  } else if (rs->queue)
    result = rs->queue->signal;
  else {
    fprintf(stderr, "Error: %s(): the receiver queue is empty.\n", __FUNCTION__);
    result = NAN;
  }
  robot_mutex_unlock();
  return result;
}
