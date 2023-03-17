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

#include <stdlib.h>

#include <webots/robot.h>
#include <webots/utils/motion.h>

int main(int argc, char **argv) {
  wb_robot_init();
  const int time_step = (int)wb_robot_get_basic_time_step();
  const WbMotionRef motion = wbu_motion_new("scout.motion");  // motion file created by Matt Denton
  wbu_motion_set_loop(motion, true);
  wbu_motion_play(motion);
  while (wb_robot_step(time_step) != -1) {
  }
  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
