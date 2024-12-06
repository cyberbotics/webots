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
/* Description:  Webots C programming interface for the Connector node            */
/**********************************************************************************/

#ifndef WB_CONNECTOR_H
#define WB_CONNECTOR_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_connector_enable_presence(WbDeviceTag tag, int sampling_period);
void wb_connector_disable_presence(WbDeviceTag tag);
int wb_connector_get_presence_sampling_period(WbDeviceTag tag);
int wb_connector_get_presence(WbDeviceTag tag);
void wb_connector_lock(WbDeviceTag tag);
void wb_connector_unlock(WbDeviceTag tag);
bool wb_connector_is_locked(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_CONNECTOR_H */
