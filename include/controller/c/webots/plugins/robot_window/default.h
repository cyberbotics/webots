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

/**************************************************************************/
/* Description:  Webots C utility programming interface for the default   */
/*               robot window. This module is not yet complete as several */
/*               devices are missing for now. More usage examples and     */
/*               documentation will be provided once it is complete.      */
/**************************************************************************/

#ifndef WBU_DEFAULT_ROBOT_WINDOW_H
#define WBU_DEFAULT_ROBOT_WINDOW_H

#include "robot_wwi.h"

#ifdef __cplusplus
extern "C" {
#endif

void wbu_default_robot_window_configure();
void wbu_default_robot_window_update();
void wbu_default_robot_window_set_images_max_size(int max_width, int max_height);

#ifdef __cplusplus
}
#endif

#endif /* WBU_DEFAULT_ROBOT_WINDOW_H */
