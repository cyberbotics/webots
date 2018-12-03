/*
 * Copyright 1996-2018 Cyberbotics Ltd.
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
/* Description:  Webots C programming interface for the DifferentialWheels node   */
/**********************************************************************************/

#ifndef WB_DIFFERENTIAL_WHEELS_H
#define WB_DIFFERENTIAL_WHEELS_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_differential_wheels_set_speed(double left, double right);
double wb_differential_wheels_get_left_speed();
double wb_differential_wheels_get_right_speed();
double wb_differential_wheels_get_max_speed();
double wb_differential_wheels_get_speed_unit();

void wb_differential_wheels_enable_encoders(int sampling_period);
void wb_differential_wheels_disable_encoders();
int wb_differential_wheels_get_encoders_sampling_period();
double wb_differential_wheels_get_left_encoder();
double wb_differential_wheels_get_right_encoder();
void wb_differential_wheels_set_encoders(double left, double right);

#ifdef __cplusplus
}
#endif

#endif /* WB_DIFFERENTIAL_WHEELS_H */
