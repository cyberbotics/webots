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

/*
 * Description:   Example controller for the Puma 560 Manipulator
 */

#include <webots/robot.h>
#include <webots/utils/motion.h>

int main() {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  // load and start forward motion
  WbMotionRef motion = wbu_motion_new("puma560.motion");
  wbu_motion_play(motion);

  // forever
  bool reverse = false;
  while (1) {
    reverse = reverse ? false : true;
    wbu_motion_set_reverse(motion, reverse);

    // play to the end of the motion
    while (!wbu_motion_is_over(motion))
      wb_robot_step(time_step);
  }

  // never reached
  return 0;
}
