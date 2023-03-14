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

/*******************************************************************************************************/
/* Description:  Header file of the robot API for the e-puck crosscompilation                          */
/*******************************************************************************************************/

#ifndef ROBOT_H
#define ROBOT_H

#include "printf_override.h"
#include "types.h"

int          wb_robot_init();
int          wb_robot_step(int duration); /* duration in milliseconds */
void         wb_robot_cleanup();
double       wb_robot_get_time();
WbDeviceTag  wb_robot_get_device(const char *name);
#define      wb_robot_get_name() "e-puck"
#define      wb_robot_get_mode() 1

#endif /* ROBOT_H */
