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
 * Description:  Example showing how to use wb_supervisor_node_get_center_of_mass()
                 to retrieve the center of mass of a robot expressed within world's frame
 */
#include <math.h>
#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#define ROBOT_INDEX 6  // position of the robot node with respect to root node children

int main(int argc, char **argv) {
  wb_robot_init();
  const int TIME_STEP = wb_robot_get_basic_time_step();
  wb_robot_step(TIME_STEP);

  WbNodeRef root, robot;
  WbFieldRef children;

  root = wb_supervisor_node_get_root();
  children = wb_supervisor_node_get_field(root, "children");
  robot = wb_supervisor_field_get_mf_node(children, ROBOT_INDEX);

  while (wb_robot_step(TIME_STEP) != -1) {
    const double *com = wb_supervisor_node_get_center_of_mass(robot);
    // Computing the image of com by the robot's inverse transform
    const double *position = wb_supervisor_node_get_position(robot);
    const double *rotation = wb_supervisor_node_get_orientation(robot);
    const double delta[3] = {com[0] - position[0], com[1] - position[1], com[2] - position[2]};
    double relative_com[3];
    int i, j;
    for (i = 0; i < 3; ++i) {
      double s = 0.0;
      for (j = 0; j < 3; ++j) {
        const int k = 3 * j;
        s += rotation[k + i] * delta[j];
      }
      relative_com[i] = s;
    }
    printf("Center of mass expressed w.r.t.\n");
    printf(" - world's frame: (%f6 %f6 %f6)\n", com[0], com[1], com[2]);
    printf(" - robot's frame: (%f6 %f6 %f6)\n", relative_com[0], relative_com[1], relative_com[2]);
  }

  wb_robot_cleanup();
  return 0;
}
