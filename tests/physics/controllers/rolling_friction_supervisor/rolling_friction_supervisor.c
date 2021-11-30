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

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  char name[8];
  const double spinning_velocity[6] = {0, 0, 0, 0, 0, 20};
  const double lateral_velocity[6] = {0, 0, 0, -25, 0, 0};

  for (int i = 4; i < 7; ++i) {
    sprintf(name, "BALL_%d", i);
    const WbNodeRef ball_node = wb_supervisor_node_get_from_def(name);
    wb_supervisor_node_set_velocity(ball_node, spinning_velocity);
  }
  

  for (int i = 7; i < 10; ++i) {
    sprintf(name, "BALL_%d", i);
    const WbNodeRef ball_node = wb_supervisor_node_get_from_def(name);
    wb_supervisor_node_set_velocity(ball_node, lateral_velocity);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
  };

  ts_send_success();
  return EXIT_SUCCESS;
}
