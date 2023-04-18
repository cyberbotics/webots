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

#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  char name[8];
  const double lateral_velocity[6] = {0, 0, 0, 20, 0, 0};
  const double spinning_velocity[6] = {0, 0, 0, 0, 0, 20};

  for (int i = 6; i < 16; ++i) {
    sprintf(name, "BALL_%d", i);
    const WbNodeRef ball_node = wb_supervisor_node_get_from_def(name);
    if (i < 11)
      wb_supervisor_node_set_velocity(ball_node, lateral_velocity);
    else
      wb_supervisor_node_set_velocity(ball_node, spinning_velocity);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
  };

  wb_robot_cleanup();

  return 0;
}
