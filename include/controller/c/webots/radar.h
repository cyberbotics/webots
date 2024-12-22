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
/* Description:  Webots C programming interface for the Radar node                */
/**********************************************************************************/

#ifndef WB_RADAR_H
#define WB_RADAR_H

#define WB_USING_C_API
#include "radar_target.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_radar_enable(WbDeviceTag tag, int sampling_period);
void wb_radar_disable(WbDeviceTag tag);
int wb_radar_get_sampling_period(WbDeviceTag tag);

int wb_radar_get_number_of_targets(WbDeviceTag tag);
const WbRadarTarget *wb_radar_get_targets(WbDeviceTag tag);

double wb_radar_get_min_range(WbDeviceTag tag);
double wb_radar_get_max_range(WbDeviceTag tag);
double wb_radar_get_horizontal_fov(WbDeviceTag tag);
double wb_radar_get_vertical_fov(WbDeviceTag tag);

#ifdef WB_MATLAB_LOADLIBRARY
// This function should be used only in the Matlab wrapper
const WbRadarTarget *wb_radar_get_target(WbDeviceTag tag, int index);
#endif

#ifdef __cplusplus
}
#endif

#endif /* WB_RADAR_H */
