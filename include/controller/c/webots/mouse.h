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

/************************************************************************/
/* Description:  Webots C programming interface for the computer mouse. */
/************************************************************************/

#ifndef WB_MOUSE_H
#define WB_MOUSE_H

#define WB_USING_C_API
#include "mouse_state.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_mouse_enable(int sampling_period);
void wb_mouse_disable();
int wb_mouse_get_sampling_period();

void wb_mouse_enable_3d_position();
void wb_mouse_disable_3d_position();
bool wb_mouse_is_3d_position_enabled();

#ifndef WB_MATLAB_LOADLIBRARY
WbMouseState wb_mouse_get_state();
#else
// This function should be used only in the Matlab wrapper
WbMouseState *wb_mouse_get_state_pointer();
#endif

#ifdef __cplusplus
}
#endif

#endif /* WB_MOUSE_H */
