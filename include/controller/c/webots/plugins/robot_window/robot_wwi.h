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

#ifndef WB_ROBOT_WWI_H
#define WB_ROBOT_WWI_H

#ifdef __cplusplus
#include <cstring>
extern "C" {
#else
#include <string.h>
#endif

void wb_robot_wwi_send(const char *data, int size);
const char *wb_robot_wwi_receive(int *size);
const char *wb_robot_wwi_receive_text();
#define wb_robot_wwi_send_text(t) wb_robot_wwi_send(t, strlen(t) + 1)

#ifdef __cplusplus
}
#endif

#endif /* WB_ROBOT_WWI_H */
