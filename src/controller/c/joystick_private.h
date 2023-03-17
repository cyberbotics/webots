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

#ifndef JOYSTICK_PRIVATE_H
#define JOYSTICK_PRIVATE_H

#include "request.h"

void joystick_write_request(WbRequest *req);
void joystick_set_sampling_period(int sampling_period);
bool joystick_read_answer(int message, WbRequest *r);
void joystick_step_end();
void wb_joystick_init();

#endif  // JOYSTICK_PRIVATE_H
