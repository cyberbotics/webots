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
/* Description:  Webots C programming interface for the GPS node                  */
/**********************************************************************************/

#ifndef WB_GPS_H
#define WB_GPS_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_gps_enable(WbDeviceTag tag, int sampling_period);
void wb_gps_disable(WbDeviceTag tag);
int wb_gps_get_sampling_period(WbDeviceTag tag);

double wb_gps_get_speed(WbDeviceTag tag);
const double *wb_gps_get_speed_vector(WbDeviceTag tag);
const double *wb_gps_get_values(WbDeviceTag tag);

const char *wb_gps_convert_to_degrees_minutes_seconds(double decimal_degrees);

typedef enum { WB_GPS_LOCAL_COORDINATE = 0, WB_GPS_WGS84_COORDINATE } WbGpsCoordinateSystem;

WbGpsCoordinateSystem wb_gps_get_coordinate_system(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_GPS_H */
