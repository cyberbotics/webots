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
/* Description:  Header file of the light_sensor API for the e-puck crosscompilation                   */
/*******************************************************************************************************/

#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "types.h"

void   wb_light_sensor_enable(WbDeviceTag dt,int sampling_period);
void   wb_light_sensor_disable(WbDeviceTag dt);
double wb_light_sensor_get_value(WbDeviceTag dt);

#endif /* LIGHT_SENSOR_H */
