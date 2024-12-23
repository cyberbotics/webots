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
/* Description:  Webots C programming interface for the Emitter node              */
/**********************************************************************************/

#ifndef WB_EMITTER_H
#define WB_EMITTER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WB_CHANNEL_BROADCAST
#define WB_CHANNEL_BROADCAST -1
#endif

int wb_emitter_send(WbDeviceTag tag, const void *data, int size);
int wb_emitter_get_buffer_size(WbDeviceTag tag);
void wb_emitter_set_channel(WbDeviceTag tag, int channel);
int wb_emitter_get_channel(WbDeviceTag tag);
double wb_emitter_get_range(WbDeviceTag tag);
void wb_emitter_set_range(WbDeviceTag tag, double range);

#ifdef __cplusplus
}
#endif

#endif /* WB_EMITTER_H */
