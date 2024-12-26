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
/* Description:  Webots C programming interface for Motion file playback          */
/**********************************************************************************/

#ifndef WBU_MOTION_H
#define WBU_MOTION_H

#include "../types.h"

#ifdef __cplusplus
extern "C" {
#endif

WbMotionRef wbu_motion_new(const char *filename);
void wbu_motion_delete(WbMotionRef motion);

void wbu_motion_play(WbMotionRef motion);
void wbu_motion_stop(WbMotionRef motion);
void wbu_motion_set_loop(WbMotionRef motion, bool loop);
void wbu_motion_set_reverse(WbMotionRef motion, bool reverse);

bool wbu_motion_is_over(WbMotionRef motion);
int wbu_motion_get_duration(WbMotionRef motion);
int wbu_motion_get_time(WbMotionRef motion);
void wbu_motion_set_time(WbMotionRef motion, int time);

#ifdef __cplusplus
}
#endif

#endif /* WBU_MOTION_H */
