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
// purpose: communication mechanism between controller and Webots
//***************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/nodes.h>
#include <webots/types.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"
#include "scheduler.h"

#define UNKNOWN_CHANNEL -1

typedef struct _PacketStruct Packet;

struct _PacketStruct {
  int channel;   // value of channel at the time that emitter_send...() was invoked
  double range;  // value of range at the time thet emitter_send...() was invoked
  int size;      // user data size (not including header)
  char *data;    // user data (packet body)
  Packet *next;  // next packet in emitter queue
};

static Packet *packet_create(const void *data, int size, int channel, double range) {
  Packet *ps = malloc(sizeof(Packet));
  ps->channel = channel;
  ps->range = range;
  ps->size = size;
  ps->data = malloc(size);
  memcpy(ps->data, data, size);
  ps->next = NULL;
  return ps;
}

static void packet_destroy(Packet *ps) {
  if (ps->data)
    free(ps->data);
  free(ps);
}

typedef struct {
  int channel;                // current emitter's channel
  int buffer_used;            // currently used buffer size
  int buffer_size;            // max buffer size (as in Emitter node)
  double byte_rate;           // max bytes sent per millisecond
  double bytes_to_send;       // bytes count according to byte_rate and elapsed time since the packet was enqueued
  Packet *queue;              // emission queue
  double range;               // current range
  double max_range;           // maximal range allowed
  int *allowed_channels;      // allowed channels emitter is allowed to emit to
  int allowed_channels_size;  // size of allowed_channels array
  bool has_range_change;
  bool has_channel_changed;
} Emitter;

static Emitter *emitter_create() {
  Emitter *es = malloc(sizeof(Emitter));
  es->channel = UNKNOWN_CHANNEL;
  es->buffer_used = 0;
  es->buffer_size = -1;
  es->byte_rate = -1.0;
  es->bytes_to_send = 0.0;
  es->queue = NULL;
  es->range = -1;
  es->max_range = -1;
  es->allowed_channels = NULL;
  es->allowed_channels_size = -1;
  es->has_range_change = false;
  es->has_channel_changed = false;
  return es;
}

static void emitter_destroy(Emitter *es) {
  Packet *ps = es->queue;
  while (ps) {
    Packet *garbage = ps;
    ps = ps->next;
    packet_destroy(garbage);
  }
  free(es->allowed_channels);
  free(es);
}

static void emitter_enqueue(Emitter *es, Packet *ps) {
  if (es->queue) {
    Packet *i = es->queue;
    while (i->next)
      i = i->next;
    i->next = ps;
  } else {
    es->queue = ps;
    es->bytes_to_send = 0.0;
  }

  es->buffer_used += ps->size;
}

static Packet *emitter_dequeue(Emitter *es) {
  Packet *ps = es->queue;
  es->queue = es->queue->next;
  es->buffer_used -= ps->size;
  return ps;
}

// Static functions

static inline WbDevice *emitter_get_device(WbDeviceTag t) {
  return robot_get_device_with_node(t, WB_NODE_EMITTER, true);
}

static void emitter_read_answer(WbDevice *d, WbRequest *r) {
  switch (request_read_uchar(r)) {
    case C_CONFIGURE: {
      Emitter *es = d->pdata;
      ROBOT_ASSERT(es->buffer_size == -1);
      es->buffer_size = request_read_int32(r);
      es->channel = request_read_int32(r);
      es->byte_rate = request_read_double(r);
      es->range = request_read_double(r);
      es->max_range = request_read_double(r);
      es->allowed_channels_size = request_read_int32(r);
      es->allowed_channels = (int *)malloc(es->allowed_channels_size * sizeof(int));
      for (int i = 0; i < es->allowed_channels_size; i++)
        es->allowed_channels[i] = request_read_int32(r);
      break;
    }
    case C_EMITTER_SET_CHANNEL: {
      Emitter *es = d->pdata;
      es->channel = request_read_int32(r);
      break;
    }
    case C_EMITTER_SET_RANGE: {
      Emitter *es = d->pdata;
      es->range = request_read_double(r);
      break;
    }
    case C_EMITTER_SET_BUFFER_SIZE: {
      Emitter *es = d->pdata;
      es->buffer_size = request_read_int32(r);
      break;
    }
    case C_EMITTER_SET_ALLOWED_CHANNELS: {
      Emitter *es = d->pdata;
      es->allowed_channels_size = request_read_int32(r);
      es->allowed_channels = (int *)realloc(es->allowed_channels, es->allowed_channels_size * sizeof(int));
      for (int i = 0; i < es->allowed_channels_size; i++)
        es->allowed_channels[i] = request_read_int32(r);
      break;
    }
    default:
      ROBOT_ASSERT(0);
  }
}

// send as many packets as the Emitter.baudRate allows
static void emitter_write_request(WbDevice *d, WbRequest *r) {
  Emitter *es = d->pdata;

  if (es->queue && es->byte_rate != -1.0)
    es->bytes_to_send += es->byte_rate * wb_robot_get_step_duration();

  while (es->queue && (es->byte_rate == -1.0 || es->queue->size <= es->bytes_to_send)) {
    Packet *ps = emitter_dequeue(es);
    request_write_uchar(r, C_EMITTER_SEND);
    request_write_int32(r, ps->channel);
    request_write_double(r, ps->range);
    request_write_int32(r, ps->size);
    request_write_data(r, ps->data, ps->size);
    es->bytes_to_send -= ps->size;
    packet_destroy(ps);
  }

  if (es->has_range_change) {
    es->has_range_change = false;
    request_write_uchar(r, C_EMITTER_SET_RANGE);
    request_write_double(r, es->range);
  }

  if (es->has_channel_changed) {
    es->has_channel_changed = false;
    request_write_uchar(r, C_EMITTER_SET_CHANNEL);
    request_write_int32(r, es->channel);
  }
}

static void emitter_cleanup(WbDevice *d) {
  emitter_destroy(d->pdata);
}

// Exported functions

void wb_emitter_init(WbDevice *d) {
  d->read_answer = emitter_read_answer;
  d->write_request = emitter_write_request;
  d->cleanup = emitter_cleanup;
  d->pdata = emitter_create();
}

// Public functions (available from the user API)

int wb_emitter_send(WbDeviceTag tag, const void *data, int size) {
  if (data == NULL) {
    fprintf(stderr, "Error: %s(): invalid argument: data = NULL.\n", __FUNCTION__);
    return 0;
  }

  if (size < 1) {
    fprintf(stderr, "Error: %s(): invalid size=%d argument.\n", __FUNCTION__, size);
    return 0;
  }

  int result = 0;
  robot_mutex_lock();
  WbDevice *d = emitter_get_device(tag);
  if (d) {
    Emitter *es = d->pdata;
    if (es->buffer_size == -1 || size <= es->buffer_size - es->buffer_used) {
      emitter_enqueue(es, packet_create(data, size, es->channel, es->range));
      result = 1;
    }
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

int wb_emitter_get_buffer_size(WbDeviceTag tag) {
  int result = -1;
  robot_mutex_lock();
  const WbDevice *d = emitter_get_device(tag);
  if (d) {
    const Emitter *es = d->pdata;
    result = es->buffer_size;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

int wb_emitter_get_channel(WbDeviceTag tag) {
  int result = -1;
  robot_mutex_lock();
  const WbDevice *d = emitter_get_device(tag);
  if (d) {
    const Emitter *es = d->pdata;
    result = es->channel;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

void wb_emitter_set_channel(WbDeviceTag tag, int channel) {
  if (channel < -1) {
    fprintf(stderr, "Error: %s() called with an invalid channel=%d. Please use a channel inside the range [-1,inf).\n",
            __FUNCTION__, channel);
    return;
  }

  robot_mutex_lock();
  WbDevice *d = emitter_get_device(tag);
  if (d) {
    Emitter *es = d->pdata;
    bool is_allowed = true;

    if (es->allowed_channels_size > 0) {
      is_allowed = false;
      for (int i = 0; i < es->allowed_channels_size; i++) {
        if (es->allowed_channels[i] == channel) {
          is_allowed = true;
          break;
        }
      }
    }

    if (!is_allowed)
      fprintf(stderr,
              "Error: %s() called with channel=%d, which is not between allowed channels. Please use an allowed channel.\n",
              __FUNCTION__, channel);
    else
      es->channel = channel;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

double wb_emitter_get_range(WbDeviceTag tag) {
  double result = NAN;
  robot_mutex_lock();
  const WbDevice *d = emitter_get_device(tag);
  if (d) {
    const Emitter *es = d->pdata;
    result = es->range;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

void wb_emitter_set_range(WbDeviceTag tag, double range) {
  if (range < 0.0 && range != -1.0) {
    fprintf(stderr, "Error: %s(): invalid range=%f argument.\n", __FUNCTION__, range);
    return;
  }
  robot_mutex_lock();
  WbDevice *d = emitter_get_device(tag);
  if (d) {
    Emitter *es = d->pdata;
    if (range == -1.0)  // requested range is infinite
      es->range = es->max_range;
    else if (es->max_range == -1.0)  // maxRange is infinite
      es->range = range;
    else if (range > es->max_range)
      es->range = es->max_range;  // clip requested value
    else
      es->range = range;  // normal case
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}
