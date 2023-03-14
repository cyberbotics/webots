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

/*************************************************************************************/
/* Description:  Webots C utility to enable/disable devices and retrieve the         */
/*               measurements from a robot window                                    */
/*************************************************************************************/

#ifndef WBU_GENERIC_ROBOT_WINDOW_H
#define WBU_GENERIC_ROBOT_WINDOW_H

#include <webots/types.h>

#ifdef __cplusplus
extern "C" {
#endif

void wbu_generic_robot_window_parse_device_command(char *token, char *tokens);
bool wbu_generic_robot_window_parse_device_control_command(char *first_token, char *tokens);
bool wbu_generic_robot_window_handle_messages(const char *message);
void wbu_generic_robot_window_init();
void wbu_generic_robot_window_update();
bool wbu_generic_robot_window_is_hidden();
double wbu_generic_robot_window_refresh_rate();
bool wbu_generic_robot_window_needs_update();

#ifdef __cplusplus
}
#endif

#endif  // WBU_GENERIC_ROBOT_WINDOW_H
