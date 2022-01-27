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

/**********************************************************************************/
/* Description:  Webots C programming interface for the Motor node                */
/**********************************************************************************/

#ifndef WB_POSITION_SENSOR_H
#define WB_POSITION_SENSOR_H

#define WB_USING_C_API
#include "types.h"

#ifndef WB_MATLAB_LOADLIBRARY
#include <math.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

void wb_position_sensor_enable(WbDeviceTag tag, int sampling_period);  // milliseconds
void wb_position_sensor_disable(WbDeviceTag tag);
int wb_position_sensor_get_sampling_period(WbDeviceTag tag);
double wb_position_sensor_get_value(WbDeviceTag tag);  // rad or meters
WbJointType wb_position_sensor_get_type(WbDeviceTag tag);
WbDeviceTag wb_position_sensor_get_motor(WbDeviceTag tag);
WbDeviceTag wb_position_sensor_get_brake(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_POSITION_SENSOR_H */
