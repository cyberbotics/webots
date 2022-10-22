/*
 * Copyright 1996-2022 Cyberbotics Ltd.
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
  short int id;
  short int source;
  short int args[VM_VARIABLES_ARG_SIZE];
  short int fwversion[2];
  short int productid;

  // short int buttons[5];

  short int buttons_state[5];
  // short int buttons_mean[5];
  // short int buttons_noise[5];

  short int prox[7];

  // short int sensor_data[7];
  // short int intensity[7];
  short int rx_data;     // not implemented
  short int ir_tx_data;  // not implemented

  short int ground_ambiant[2];
  short int ground_reflected[2];
  short int ground_delta[2];

  short int target[2];
  // short int vbat[2];
  // short int imot[2];
  short int uind[2];
  short int pwm[2];

  short int acc[3];

  short int ntc;  // not implemented

  short int rc5_address;  // not implemented
  short int rc5_command;  // not implemented

  short int sound_level;  // not implemented
  short int sound_tresh;  // not implemented
  short int sound_mean;   // not implemented

  short int timers[2];

  short int acc_tap;  // not implemented

  short int freeSpace[VM_VARIABLES_FREE_SPACE];
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
