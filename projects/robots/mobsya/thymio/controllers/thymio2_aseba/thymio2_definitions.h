/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Strongly inspired from https://github.com/aseba-community/aseba-target-thymio2/blob/master/skel-usb-user.h

#ifndef SKEL_USB_USER_H
#define SKEL_USB_USER_H

#include <common/types.h>

#define PRODUCT_ID 8
#define VM_VARIABLES_FREE_SPACE 512
#define VM_VARIABLES_ARG_SIZE 32

struct _vmVariables {
  sint16 id;
  sint16 source;
  sint16 args[VM_VARIABLES_ARG_SIZE];
  sint16 fwversion[2];
  sint16 productid;

  // sint16 buttons[5];

  sint16 buttons_state[5];
  // sint16 buttons_mean[5];
  // sint16 buttons_noise[5];

  sint16 prox[7];

  // sint16 sensor_data[7];
  // sint16 intensity[7];
  sint16 rx_data;     // not implemented
  sint16 ir_tx_data;  // not implemented

  sint16 ground_ambiant[2];
  sint16 ground_reflected[2];
  sint16 ground_delta[2];

  sint16 target[2];
  // sint16 vbat[2];
  // sint16 imot[2];
  sint16 uind[2];
  sint16 pwm[2];

  sint16 acc[3];

  sint16 ntc;  // not implemented

  sint16 rc5_address;  // not implemented
  sint16 rc5_command;  // not implemented

  sint16 sound_level;  // not implemented
  sint16 sound_tresh;  // not implemented
  sint16 sound_mean;   // not implemented

  sint16 timers[2];

  sint16 acc_tap;  // not implemented

  sint16 freeSpace[VM_VARIABLES_FREE_SPACE];
};

enum Event {
  EVENT_B_BACKWARD = 0,
  EVENT_B_LEFT,
  EVENT_B_CENTER,
  EVENT_B_FORWARD,
  EVENT_B_RIGHT,
  EVENT_BUTTONS,
  EVENT_PROX,
  EVENT_DATA,  // not implemented
  EVENT_TAP,
  EVENT_ACC,
  EVENT_MIC,
  EVENT_SOUND_FINISHED,  // not implemented
  EVENT_TEMPERATURE,     // not implemented
  EVENT_RC5,             // not implemented
  EVENT_MOTOR,
  EVENT_TIMER0,
  EVENT_TIMER1,
  EVENT_COUNT
};

#endif
