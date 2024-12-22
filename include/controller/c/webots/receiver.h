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
/* Description:  Webots C programming interface for the Receiver node             */
/**********************************************************************************/

#ifndef WB_RECEIVER_H
#define WB_RECEIVER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WB_CHANNEL_BROADCAST
#define WB_CHANNEL_BROADCAST -1
#endif

// device functions
void wb_receiver_enable(WbDeviceTag tag, int sampling_period);
void wb_receiver_disable(WbDeviceTag tag);
int wb_receiver_get_sampling_period(WbDeviceTag tag);

void wb_receiver_set_channel(WbDeviceTag tag, int channel);
int wb_receiver_get_channel(WbDeviceTag tag);
int wb_receiver_get_queue_length(WbDeviceTag tag);
void wb_receiver_next_packet(WbDeviceTag tag);
int wb_receiver_get_data_size(WbDeviceTag tag);
const void *wb_receiver_get_data(WbDeviceTag tag);
double wb_receiver_get_signal_strength(WbDeviceTag tag);
const double *wb_receiver_get_emitter_direction(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_RECEIVER_H */
