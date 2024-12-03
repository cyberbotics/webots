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
/* Description:  Webots C programming interface for the VacuumGripper node            */
/**********************************************************************************/

#ifndef WB_VACUUM_GRIPPER_H
#define WB_VACUUM_GRIPPER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_vacuum_gripper_enable_presence(WbDeviceTag tag, int sampling_period);
void wb_vacuum_gripper_disable_presence(WbDeviceTag tag);
int wb_vacuum_gripper_get_presence_sampling_period(WbDeviceTag tag);
bool wb_vacuum_gripper_get_presence(WbDeviceTag tag);
void wb_vacuum_gripper_turn_on(WbDeviceTag tag);
void wb_vacuum_gripper_turn_off(WbDeviceTag tag);
bool wb_vacuum_gripper_is_on(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_VACUUM_GRIPPER_H */
