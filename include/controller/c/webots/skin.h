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
/* Description:  Webots C programming interface for the Skin node                 */
/**********************************************************************************/

#ifndef WB_SKIN_H
#define WB_SKIN_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_skin_set_bone_orientation(WbDeviceTag tag, int index, const double orientation[4], bool absolute);
void wb_skin_set_bone_position(WbDeviceTag tag, int index, const double position[3], bool absolute);
int wb_skin_get_bone_count(WbDeviceTag tag);
const char *wb_skin_get_bone_name(WbDeviceTag tag, int index);
const double *wb_skin_get_bone_orientation(WbDeviceTag tag, int index, bool absolute);
const double *wb_skin_get_bone_position(WbDeviceTag tag, int index, bool absolute);

#ifdef __cplusplus
}
#endif

#endif /* WB_SKIN_H */
