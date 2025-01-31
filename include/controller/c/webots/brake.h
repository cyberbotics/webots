/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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

/**********************************************************************************/
/* Description:  Webots C programming interface for the brake node                */
/**********************************************************************************/

#ifndef WB_BRAKE_H
#define WB_BRAKE_H

#define WB_USING_C_API
#include "types.h"

#ifndef WB_MATLAB_LOADLIBRARY
#include <math.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

void wb_brake_set_damping_constant(WbDeviceTag tag, double damping_constant);
WbJointType wb_brake_get_type(WbDeviceTag tag);
WbDeviceTag wb_brake_get_motor(WbDeviceTag tag);
WbDeviceTag wb_brake_get_position_sensor(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_BRAKE_H */
