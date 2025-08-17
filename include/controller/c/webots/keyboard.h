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
/* Description:  Webots C programming interface for the computer keyboard         */
/**********************************************************************************/

#ifndef WB_KEYBOARD_H
#define WB_KEYBOARD_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
  WB_KEYBOARD_END = 312,
  WB_KEYBOARD_HOME,
  WB_KEYBOARD_LEFT,
  WB_KEYBOARD_UP,
  WB_KEYBOARD_RIGHT,
  WB_KEYBOARD_DOWN,
  WB_KEYBOARD_PAGEUP = 366,
  WB_KEYBOARD_PAGEDOWN,
  WB_KEYBOARD_NUMPAD_HOME = 375,
  WB_KEYBOARD_NUMPAD_LEFT,
  WB_KEYBOARD_NUMPAD_UP,
  WB_KEYBOARD_NUMPAD_RIGHT,
  WB_KEYBOARD_NUMPAD_DOWN,
  WB_KEYBOARD_NUMPAD_END = 382,
  WB_KEYBOARD_KEY = 0x0000ffff,
  WB_KEYBOARD_SHIFT = 0x00010000,
  WB_KEYBOARD_CONTROL = 0x00020000,
  WB_KEYBOARD_ALT = 0x00040000
};

void wb_keyboard_enable(int sampling_period);
void wb_keyboard_disable();
int wb_keyboard_get_sampling_period();
int wb_keyboard_get_key();

#ifdef __cplusplus
}
#endif

#endif /* WB_KEYBOARD_H */
