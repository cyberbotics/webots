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

#include <plugins/radio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/nodes.h>
#include <webots/radio.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"
#include "scheduler.h"

// static functions

typedef struct _GSList GSList;
struct _GSList {
  void *data;
  GSList *next;
};

static void g_slist_free(GSList *list) {
  if (list->next)
    g_slist_free(list->next);
  free(list);
}

static GSList *g_slist_append(GSList *list, void *data) {
  GSList *last;
  for (last = list; list && last->next; last = last->next) {
  }
  GSList *l = malloc(sizeof(GSList));
  l->data = data;
  l->next = NULL;
  if (list)
    last->next = l;
  else
    return l;
  return list;
}

typedef struct {
  int length;
  char *body;
  char *destination;
  double delay;
} _RadioMessage;

typedef struct {
  int enable : 1;
  int set_address : 1;
  int set_frequency : 1;
  int set_channel : 1;
  int set_bitrate : 1;
  int set_rx_sensitivity : 1;
  int set_tx_power : 1;
  int sampling_period;
  char *address;
  double frequency;
  int channel;
  int bitrate;
  double rx_sensitivity;
  double tx_power;
  GSList *send;
  GSList *receive;
  void (*callback)(WbRadioEvent);
} _Radio;

static inline WbDevice *radio_get_device(WbDeviceTag t) {
  return robot_get_device_with_node(t, WB_NODE_RADIO, true);
}

static inline _Radio *radio_get_device_data(WbDeviceTag t) {
  return (_Radio *)robot_get_device_with_node(t, WB_NODE_RADIO, true)->pdata;
}

static void radio_read_answer(WbDevice *d, WbRequest *r) {
  struct WebotsRadioEvent event;
  int tag;
  _Radio *radio = (_Radio *)d->pdata;
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:
      radio->address = request_read_string(r);
      radio->frequency = request_read_double(r);
      radio->channel = request_read_int32(r);
      radio->bitrate = request_read_int32(r);
      radio->rx_sensitivity = request_read_double(r);
      radio->tx_power = request_read_double(r);
      fflush(stdout);
      break;
    case C_RADIO_RECEIVE:
      event.type = 1;
      event.rssi = request_read_double(r);
      event.from = request_read_string(r);
      event.data_size = request_read_uint32(r);
      event.data = (const char *)request_read_data(r, event.data_size);
      tag = robot_get_device_tag(d);
      event.user_data = &tag;
      if (radio->callback)
        (*radio->callback)(&event);
      break;
  }
}

static void radio_write_request(WbDevice *d, WbRequest *r) {
  // enable device if requested
  _Radio *radio = (_Radio *)d->pdata;
  if (radio->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, radio->sampling_period);
    radio->enable = false;  // done
  }

  // set device parameters
  if (radio->set_address) {
    request_write_uchar(r, C_RADIO_SET_ADDRESS);
    request_write_int32(r, strlen(radio->address) + 1);
    request_write_data(r, radio->address, strlen(radio->address) + 1);
    radio->set_address = false;
  }
  if (radio->set_frequency) {
    request_write_uchar(r, C_RADIO_SET_FREQUENCY);
    request_write_double(r, radio->frequency);
    radio->set_frequency = false;
  }
  if (radio->set_channel) {
    request_write_uchar(r, C_RADIO_SET_CHANNEL);
    request_write_int32(r, radio->channel);
    radio->set_channel = false;
  }
  if (radio->set_bitrate) {
    request_write_uchar(r, C_RADIO_SET_BITRATE);
    request_write_int32(r, radio->bitrate);
    radio->set_bitrate = false;
  }
  if (radio->set_rx_sensitivity) {
    request_write_uchar(r, C_RADIO_SET_RX_SENSITIVITY);
    request_write_double(r, radio->rx_sensitivity);
    radio->set_rx_sensitivity = false;
  }
  if (radio->set_tx_power) {
    request_write_uchar(r, C_RADIO_SET_TX_POWER);
    request_write_double(r, radio->tx_power);
    radio->set_tx_power = false;
  }

  // send pending packets
  GSList *i;
  for (i = radio->send; i; i = i->next) {
    _RadioMessage *msg = (_RadioMessage *)i->data;
    request_write_uchar(r, C_RADIO_SEND);
    request_write_int32(r, strlen(msg->destination) + 1);
    request_write_data(r, msg->destination, strlen(msg->destination) + 1);
    request_write_int32(r, msg->length);
    request_write_data(r, msg->body, msg->length);
    request_write_double(r, msg->delay);
    wb_radio_message_delete(msg);
  }
  if (radio->send != NULL)
    g_slist_free(radio->send);
  radio->send = NULL;
}

static void radio_cleanup(WbDevice *d) {
  _Radio *radio = (_Radio *)d->pdata;
  GSList *i;
  if (radio) {
    if (radio->address)
      free(radio->address);
    if (radio->send) {
      for (i = radio->send; i; i = i->next)
        wb_radio_message_delete((_RadioMessage *)i->data);
      g_slist_free(radio->send);
    }
    if (radio->receive) {
      for (i = radio->receive; i; i = i->next)
        wb_radio_message_delete((_RadioMessage *)i->data);
      g_slist_free(radio->receive);
    }
    free(radio);
  }
}

#ifdef EXTCONTROLLER
static void radio_toggle_remote(WbDevice *d, WbRequest *r) {
  // TODO
}
#endif

// Exported functions

void wb_radio_init(WbDevice *d) {
  _Radio *radio = malloc(sizeof(_Radio));
  radio->enable = false;  // do not enable
  radio->set_address = false;
  radio->set_frequency = false;
  radio->set_channel = false;
  radio->set_bitrate = false;
  radio->set_rx_sensitivity = false;
  radio->set_tx_power = false;
  radio->sampling_period = 0;
  radio->address = NULL;
  radio->frequency = -1.0;
  radio->channel = -1;
  radio->bitrate = -1;
  radio->rx_sensitivity = NAN;
  radio->tx_power = NAN;
  radio->send = NULL;
  radio->receive = NULL;
  radio->callback = NULL;

  d->read_answer = radio_read_answer;
  d->write_request = radio_write_request;
  d->cleanup = radio_cleanup;
  d->pdata = radio;
#ifdef EXTCONTROLLER
  d->toggle_remote = radio_toggle_remote;
#endif
}

// Public functions available from the user API

void wb_radio_enable(WbDeviceTag tag, int sampling_period) {
  robot_mutex_lock();
  WbDevice *d = radio_get_device(tag);
  if (d) {
    _Radio *radio = (_Radio *)d->pdata;
    radio->enable = true;
    radio->sampling_period = sampling_period;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

void wb_radio_disable(WbDeviceTag tag) {
  const WbDevice *d = radio_get_device(tag);
  if (d)
    wb_radio_enable(tag, 0);
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

WbRadioMessage wb_radio_message_new(int length, const char *body, const char *destination) {
  _RadioMessage *msg;
  msg = malloc(sizeof(_RadioMessage));
  msg->length = length;
  msg->body = malloc(length);
  memcpy(msg->body, body, length);
  msg->destination = malloc(strlen(destination) + 1);
  strcpy(msg->destination, destination);
  msg->delay = 0.0;
  return msg;
}

void wb_radio_message_delete(WbRadioMessage msg) {
  _RadioMessage *m = (_RadioMessage *)msg;
  if (m == NULL)
    return;
  if (m->body)
    free(m->body);
  if (m->destination)
    free(m->destination);
  free(m);
}

const char *wb_radio_message_get_destination(WbRadioMessage msg) {
  return ((_RadioMessage *)msg)->destination;
}

int wb_radio_message_get_length(WbRadioMessage msg) {
  return ((_RadioMessage *)msg)->length;
}

const char *wb_radio_message_get_body(WbRadioMessage msg) {
  return ((_RadioMessage *)msg)->body;
}

void wb_radio_set_address(WbDeviceTag tag, const char *address) {
  if (address == NULL) {
    fprintf(stderr, "Error: %s(): invalid NULL argument.\n", __FUNCTION__);
    return;
  }

  _Radio *radio = radio_get_device_data(tag);
  if (radio) {
    free(radio->address);
    radio->address = (char *)malloc(strlen(address) + 1);
    strcpy(radio->address, address);
    radio->set_address = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

const char *wb_radio_get_address(WbDeviceTag tag) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio)
    return radio->address;

  fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  return NULL;
}

void wb_radio_set_frequency(WbDeviceTag tag, double hz) {
  if (hz <= 0.0) {
    fprintf(stderr, "Error: %s(): invalid negative or zero hz argument: %f.\n", __FUNCTION__, hz);
    return;
  }

  _Radio *radio = radio_get_device_data(tag);
  if (radio) {
    radio->frequency = hz;
    radio->set_frequency = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

double wb_radio_get_frequency(WbDeviceTag tag) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio)
    return radio->frequency;
  else {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return NAN;
  }
}

void wb_radio_set_channel(WbDeviceTag tag, int channel) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio) {
    radio->channel = channel;
    radio->set_channel = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_radio_get_channel(WbDeviceTag tag) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio)
    return radio->channel;
  else {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return -1;
  }
}

void wb_radio_set_bitrate(WbDeviceTag tag, int bits_per_second) {
  if (bits_per_second <= 0) {
    fprintf(stderr, "Error: %s(): invalid negative or zero bits_per_second argument: %d.\n", __FUNCTION__, bits_per_second);
    return;
  }

  _Radio *radio = radio_get_device_data(tag);
  if (radio) {
    radio->bitrate = bits_per_second;
    radio->set_bitrate = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

int wb_radio_get_bitrate(WbDeviceTag tag) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio)
    return radio->bitrate;

  fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  return 0;
}

void wb_radio_set_rx_sensitivity(WbDeviceTag tag, double dBm) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio) {
    radio->rx_sensitivity = dBm;
    radio->set_rx_sensitivity = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

double wb_radio_get_rx_sensitivity(WbDeviceTag tag) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio)
    return radio->rx_sensitivity;

  fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  return NAN;
}

void wb_radio_set_tx_power(WbDeviceTag tag, double dBm) {
  if (dBm <= 0) {
    fprintf(stderr, "Error: %s(): invalid negative or zero dBm argument: %f.\n", __FUNCTION__, dBm);
    return;
  }

  _Radio *radio = radio_get_device_data(tag);
  if (radio) {
    radio->tx_power = dBm;
    radio->set_tx_power = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

double wb_radio_get_tx_power(WbDeviceTag tag) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio)
    return radio->tx_power;

  fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  return NAN;
}

void wb_radio_set_callback(WbDeviceTag tag, void (*callback)(WbRadioEvent)) {
  _Radio *radio = radio_get_device_data(tag);
  if (radio)
    radio->callback = callback;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

void wb_radio_send(WbDeviceTag tag, void *msg, double delay) {
  if (msg == NULL) {
    fprintf(stderr, "Error: %s(): invalid NULL argument.\n", __FUNCTION__);
    return;
  }
  if (delay < 0.0) {
    fprintf(stderr, "Error: %s(): invalid negative delay argument: %f.\n", __FUNCTION__, delay);
    return;
  }

  _Radio *radio = radio_get_device_data(tag);
  if (radio == NULL) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }

  _RadioMessage *message = (_RadioMessage *)msg;
  message->delay = delay;
  radio->send = g_slist_append(radio->send, message);
}

WbDeviceTag wb_radio_event_get_radio(const WbRadioEvent e) {
  struct WebotsRadioEvent *ev = (struct WebotsRadioEvent *)e;
  return (WbDeviceTag)(*(int *)(ev->user_data));
}

char *wb_radio_event_get_data(const WbRadioEvent e) {
  struct WebotsRadioEvent *ev = (struct WebotsRadioEvent *)e;
  return (char *)ev->data;
}

int wb_radio_event_get_data_size(const WbRadioEvent e) {
  const struct WebotsRadioEvent *ev = (struct WebotsRadioEvent *)e;
  return ev->data_size;
}

char *wb_radio_event_get_emitter(const WbRadioEvent e) {
  struct WebotsRadioEvent *ev = (struct WebotsRadioEvent *)e;
  return (char *)ev->from;
}

double wb_radio_event_get_rssi(const WbRadioEvent e) {
  const struct WebotsRadioEvent *ev = (struct WebotsRadioEvent *)e;
  return ev->rssi;
}
