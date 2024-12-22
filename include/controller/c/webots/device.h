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
/* Description:  Abstraction of a robot device                                    */
/**********************************************************************************/

#ifndef WB_DEVICE_H
#define WB_DEVICE_H

#define WB_USING_C_API
#include "nodes.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

const char *wb_device_get_name(WbDeviceTag dt);
const char *wb_device_get_model(WbDeviceTag dt);
WbNodeType wb_device_get_node_type(WbDeviceTag dt);

// deprecated since Webots 8.0.0, please use wb_device_get_node_type() instead
WbNodeType wb_device_get_type(WbDeviceTag dt) WB_DEPRECATED;

#ifdef __cplusplus
}
#endif

#endif /* WB_DEVICE_H */
