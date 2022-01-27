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
/* Description:  Webots C programming interface for the Accelerometer node        */
/**********************************************************************************/

#ifndef WB_ACCELEROMETER_H
#define WB_ACCELEROMETER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_accelerometer_enable(WbDeviceTag tag, int sampling_period);
void wb_accelerometer_disable(WbDeviceTag tag);
int wb_accelerometer_get_sampling_period(WbDeviceTag tag);

int wb_accelerometer_get_lookup_table_size(WbDeviceTag tag);
const double *wb_accelerometer_get_lookup_table(WbDeviceTag tag);

// return a pointer to an array of 3 double for X, Y and Z accelerations
const double *wb_accelerometer_get_values(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_ACCELEROMETER_H */
