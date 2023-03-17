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

#ifndef REMOTE_CONTROL_PRIVATE_H
#define REMOTE_CONTROL_PRIVATE_H

#include <webots/remote_control.h>
#include "request.h"

// remote control
void remote_control_init(const char *library_name);
void remote_control_cleanup();
bool remote_control_is_initialized();

const char *remote_control_get_last_error();

bool remote_control_start(const char *);
void remote_control_stop();

void remote_control_stop_actuators();
bool remote_control_has_failed();
void remote_control_step(int duration);
WbRequest *remote_control_handle_messages(WbRequest *r);
void remote_control_handle_one_message(WbRequest *r, WbDeviceTag tag);

bool remote_control_is_function_defined(const char *function_name);

#endif  // REMOTE_CONTROL_PRIVATE_H
