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

// Strongly inspired from https://github.com/aseba-community/aseba-target-thymio2/blob/master/skel-usb-user.c

#include "thymio2_definitions.h"
#include "thymio2_natives.h"

const AsebaVMDescription vmDescription = {"thymio-II",
                                          {{1, "_id"},
                                           {1, "event.source"},
                                           {VM_VARIABLES_ARG_SIZE, "event.args"},
                                           {2, "_fwversion"},
                                           {1, "_productId"},

                                           // {5, "buttons._raw"},
                                           {1, "button.backward"},
                                           {1, "button.left"},
                                           {1, "button.center"},
                                           {1, "button.forward"},
                                           {1, "button.right"},

                                           // {5, "buttons._mean"},
                                           // {5, "buttons._noise"},

                                           {7, "prox.horizontal"},

                                           // {7, "prox.comm.rx._payloads"},
                                           // {7, "prox.comm.rx._intensities"},
                                           {1, "prox.comm.rx"},  // not implemented
                                           {1, "prox.comm.tx"},  // not implemented

                                           {2, "prox.ground.ambiant"},
                                           {2, "prox.ground.reflected"},
                                           {2, "prox.ground.delta"},

                                           {1, "motor.left.target"},
                                           {1, "motor.right.target"},
                                           // {2, "_vbat"},
                                           // {2, "_imot"},
                                           {1, "motor.left.speed"},
                                           {1, "motor.right.speed"},
                                           {1, "motor.left.pwm"},
                                           {1, "motor.right.pwm"},

                                           {3, "acc"},

                                           {1, "temperature"},  // not implemented

                                           {1, "rc5.address"},  // not implemented
                                           {1, "rc5.command"},  // not implemented

                                           {1, "mic.intensity"},  // not implemented
                                           {1, "mic.threshold"},  // not implemented
                                           {1, "mic._mean"},      // not implemented

                                           {2, "timer.period"},

                                           {1, "acc._tap"},  // not implemented

                                           {0, NULL}}};

const AsebaLocalEventDescription localEvents[] = {
  {"button.backward", "Backward button status changed"},
  {"button.left", "Left button status changed"},
  {"button.center", "Center button status changed"},
  {"button.forward", "Forward button status changed"},
  {"button.right", "Right button status changed"},
  {"buttons", "Buttons values updated"},
  {"prox", "Proximity values updated"},
  {"prox.comm", "Data received on the proximity communication"},  // not implemented
  {"tap", "A tap is detected"},                                   // not implemented
  {"acc", "Accelerometer values updated"},
  {"mic", "Fired when microphone intensity is above threshold"},                        // not implemented
  {"sound.finished", "Fired when the playback of a user initiated sound is finished"},  // not implemented
  {"temperature", "Temperature value updated"},                                         // not implemented
  {"rc5", "RC5 message received"},                                                      // not implemented
  {"motor", "Motor timer"},
  {"timer0", "Timer 0"},
  {"timer1", "Timer 1"},
  {NULL, NULL}};

const AsebaNativeFunctionDescription *nativeFunctionsDescriptions[] = {&AsebaNativeDescription__system_reboot,
                                                                       &AsebaNativeDescription__system_settings_read,
                                                                       &AsebaNativeDescription__system_settings_write,
                                                                       &AsebaNativeDescription__system_settings_flash,

                                                                       ASEBA_NATIVES_STD_DESCRIPTIONS,

                                                                       THYMIO_NATIVES_DESCRIPTIONS,

                                                                       &AsebaNativeDescription_poweroff,
                                                                       0};

AsebaNativeFunctionPointer nativeFunctions[] = {AsebaNative__system_reboot,
                                                AsebaNative__system_settings_read,
                                                AsebaNative__system_settings_write,
                                                AsebaNative__system_settings_flash,

                                                ASEBA_NATIVES_STD_FUNCTIONS,

                                                THYMIO_NATIVES_FUNCTIONS,

                                                power_off};
