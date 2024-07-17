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

// ***************************************************
//  this file contains the API code for the LED device
// ***************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/led.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
  bool set_state;
  int state;
} LED;

static LED *led_create() {
  LED *led = malloc(sizeof(LED));
  led->set_state = false;
  led->state = 0;
  return led;
}

static LED *led_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_LED, true);
  return d ? d->pdata : NULL;
}

static void led_write_request(WbDevice *d, WbRequest *r) {
  LED *led = d->pdata;
  if (led->set_state) {
    request_write_uchar(r, C_LED_SET);
    request_write_int32(r, led->state);
    led->set_state = false;
  }
}

static void led_cleanup(WbDevice *d) {
  free(d->pdata);
}

static void led_toggle_remote(WbDevice *d, WbRequest *r) {
  LED *led = d->pdata;
  if (led->state != 0)
    led->set_state = true;
}

// Exported functions

void wb_led_init(WbDevice *d) {
  d->read_answer = NULL;
  d->write_request = led_write_request;
  d->cleanup = led_cleanup;
  d->pdata = led_create();
  d->toggle_remote = led_toggle_remote;
}

// Public functions (available from the user API)

void wb_led_set(WbDeviceTag tag, int value) {
  robot_mutex_lock();
  LED *led = led_get_struct(tag);
  if (led) {
    led->state = value;
    led->set_state = true;
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
}

int wb_led_get(WbDeviceTag tag) {
  int state = 0;
  robot_mutex_lock();
  const LED *led = led_get_struct(tag);
  if (led)
    state = led->state;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return state;
}
