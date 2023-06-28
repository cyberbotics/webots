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
/* Description:  Webots C programming interface for the DistanceSensor node       */
/**********************************************************************************/

#ifndef WB_DISTANCE_SENSOR_H
#define WB_DISTANCE_SENSOR_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_distance_sensor_enable(WbDeviceTag tag, int sampling_period);
void wb_distance_sensor_disable(WbDeviceTag tag);
int wb_distance_sensor_get_sampling_period(WbDeviceTag tag);
double wb_distance_sensor_get_value(WbDeviceTag tag);

double wb_distance_sensor_get_max_value(WbDeviceTag tag);
double wb_distance_sensor_get_min_value(WbDeviceTag tag);
double wb_distance_sensor_get_aperture(WbDeviceTag tag);

int wb_distance_sensor_get_lookup_table_size(WbDeviceTag tag);
const double *wb_distance_sensor_get_lookup_table(WbDeviceTag tag);

typedef enum {
  WB_DISTANCE_SENSOR_GENERIC = 0,
  WB_DISTANCE_SENSOR_INFRA_RED,
  WB_DISTANCE_SENSOR_SONAR,
  WB_DISTANCE_SENSOR_LASER
} WbDistanceSensorType;

WbDistanceSensorType wb_distance_sensor_get_type(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_DISTANCE_SENSOR_H */
