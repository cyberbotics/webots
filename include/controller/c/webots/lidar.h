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

/**********************************************************************************/
/* Description:  Webots C programming interface for the Lidar node                */
/**********************************************************************************/

#ifndef WB_LIDAR_H
#define WB_LIDAR_H

#define WB_USING_C_API
#include "lidar_point.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_lidar_enable(WbDeviceTag tag, int sampling_period);
void wb_lidar_enable_point_cloud(WbDeviceTag tag);
void wb_lidar_disable(WbDeviceTag tag);
void wb_lidar_disable_point_cloud(WbDeviceTag tag);
int wb_lidar_get_sampling_period(WbDeviceTag tag);
bool wb_lidar_is_point_cloud_enabled(WbDeviceTag tag);

const float *wb_lidar_get_range_image(WbDeviceTag tag);
const float *wb_lidar_get_layer_range_image(WbDeviceTag tag, int layer);

const WbLidarPoint *wb_lidar_get_point_cloud(WbDeviceTag tag);
const WbLidarPoint *wb_lidar_get_layer_point_cloud(WbDeviceTag tag, int layer);
int wb_lidar_get_number_of_points(WbDeviceTag tag);

int wb_lidar_get_horizontal_resolution(WbDeviceTag tag);
int wb_lidar_get_number_of_layers(WbDeviceTag tag);
double wb_lidar_get_min_frequency(WbDeviceTag tag);
double wb_lidar_get_max_frequency(WbDeviceTag tag);
double wb_lidar_get_frequency(WbDeviceTag tag);
void wb_lidar_set_frequency(WbDeviceTag tag, double frequency);
double wb_lidar_get_fov(WbDeviceTag tag);
double wb_lidar_get_vertical_fov(WbDeviceTag tag);
double wb_lidar_get_min_range(WbDeviceTag tag);
double wb_lidar_get_max_range(WbDeviceTag tag);

#ifdef WB_MATLAB_LOADLIBRARY
// This function should be used only in the Matlab wrapper
const WbLidarPoint *wb_lidar_get_point(WbDeviceTag tag, int index);
#endif

#ifdef __cplusplus
}
#endif

#endif /* WB_LIDAR_H */
