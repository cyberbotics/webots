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
 * Description:  Default controller of the Nao robot
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/utils/motion.h>

int main(int argc, char *argv[]) {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  printf("Default (hello!) controller of the Nao robot started...\n");

  // load motion
  WbMotionRef hand_wave = wbu_motion_new("HandWave.motion");

  while (1) {
    // start motion
    wbu_motion_play(hand_wave);

    // wait for termination of motion
    while (!wbu_motion_is_over(hand_wave))
      wb_robot_step(time_step);

    // wait 5 seconds
    int pause;
    for (pause = 5000; pause > 0; pause -= time_step)
      wb_robot_step(time_step);
  }

  wb_robot_cleanup();

  return 0;
}
