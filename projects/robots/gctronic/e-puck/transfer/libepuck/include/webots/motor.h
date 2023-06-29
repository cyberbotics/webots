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
/* Description:  Header file of the motor API for the e-puck crosscompilation                          */
/*******************************************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include "types.h"

#ifndef M_PI
  #define M_PI 3.14159
#endif

#ifndef INFINITY
  #define INFINITY 0
#endif

void   wb_motor_set_velocity(WbDeviceTag dt, double velocity);
double wb_motor_get_velocity(WbDeviceTag dt);
void   wb_motor_set_position() { }

#endif /* MOTOR_H */
