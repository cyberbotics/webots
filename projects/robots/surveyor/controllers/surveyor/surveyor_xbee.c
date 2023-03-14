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

/*
 * Description:  The controller used to control the Surveyor using Xbee.
 */

#include <device/emitter.h>
#include <device/receiver.h>
#include <device/robot.h>
#include <stdio.h>
#include "surveyor_protocol.h"

#define TIME_STEP 20
#define MAX_BUFFER_SIZE 90

/* The functions used by Webots. */
static void reset(void);
static int run(int);

static DeviceTag emitter, receiver;

int main() {
  robot_live(reset);
  robot_run(run);

  return 0;
}

static void reset(void) {
  emitter = robot_get_device("emitter");
  receiver = robot_get_device("receiver");

  receiver_enable(receiver, TIME_STEP);

  surveyor_init();

  return;
}

static int run(int ms) {
  int data_available;

  surveyor_update(TIME_STEP);

  data_available = receiver_get_buffer_size(receiver);

  if (data_available > 0) {
    const unsigned char *receiver_buffer = (unsigned char *)receiver_get_buffer(receiver);
    const unsigned char *emitter_buffer = (unsigned char *)emitter_get_buffer(emitter);
    surveyor_send(receiver_buffer, emitter_buffer);
    emitter_send(emitter, MAX_BUFFER_SIZE);
  }

  return TIME_STEP;
}
