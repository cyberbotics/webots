/*
 * Copyright 1996-2021 Cyberbotics Ltd.
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

/*
 * Description:  Defines the entry point of the robot window library
 */

#ifndef GENERIC_H
#define GENERIC_H

#include <webots/types.h>

void configure_generic_robot_window(const char *message);
void init_generic_robot_window();
void parse_generic_command(char *token, WbDeviceTag *tag, bool *robot);

#endif
