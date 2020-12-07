/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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

#ifndef HTML_ROBOT_WINDOW_PRIVATE_H
#define HTML_ROBOT_WINDOW_PRIVATE_H

bool wb_robot_window_load_library(const char *name);
void html_robot_window_init();
void html_robot_window_step(int step);
void html_robot_window_cleanup();

#endif  // HTML_ROBOT_WINDOW__PRIVATE_H
