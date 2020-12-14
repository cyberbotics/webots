/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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

#ifndef ROBOT_WINDOW_PRIVATE_H
#define ROBOT_WINDOW_PRIVATE_H

#include <webots/types.h>

void robot_window_init(const char *library_name);
bool robot_window_is_initialized();
const char *robot_window_get_last_error();
void robot_window_pre_update_gui();
void robot_window_update_gui();
void robot_window_read_sensors();
void robot_window_write_actuators();
void robot_window_cleanup();
void robot_window_show();

#endif  // ROBOT_WINDOW_H
