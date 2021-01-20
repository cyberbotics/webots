/*
 * Copyright 1996-2021 Cyberbotics Ltd.
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

/*
 * Description:  Defines generic functions to use devices in a robot window.
 */

#ifndef GENERIC_ROBOT_WINDOW_H
#define GENERIC_ROBOT_WINDOW_H

#include <webots/types.h>

#ifdef __cplusplus
extern "C" {
#endif

void parse_device_command(char *token, char *tokens);
bool parse_device_control_command(char *token, char *tokens);
bool handle_generic_robot_window_messages(const char *message);
void init_robot_window();
void update_robot_window();
bool robot_window_is_hidden();
double robot_window_refresh_rate();
bool robot_window_needs_update();

// Utility functions to parse robot window message
char *string_utils_strsep(char **stringp, const char *delim);
char *string_utils_replace(char *orig, char *rep, char *with);

#ifdef __cplusplus
}
#endif

#endif  // GENERIC_ROBOT_WINDOW_H
