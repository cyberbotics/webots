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
 * Description:  Example showing how to use wb_supervisor_node_get_contact_points()
                 to retrieve the contact points of a solid expressed within world's frame
 */
#include <math.h>
#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#define RED_CYLINDER_INDEX 5  // position of the red cylinder node with respect to root node children
#define RED_CYLINDER_HALF_HEIGHT 0.25
#define TOLERANCE 0.005

int main(int argc, char **argv) {
  wb_robot_init();
  const int TIME_STEP = wb_robot_get_basic_time_step();
  WbNodeRef root, red_cylinder;
  WbFieldRef children;

  root = wb_supervisor_node_get_root();
  children = wb_supervisor_node_get_field(root, "children");
  red_cylinder = wb_supervisor_field_get_mf_node(children, RED_CYLINDER_INDEX);
  // counter for contact points located on the upper (resp. lower) cylinder cap for the previous time step
  int pu = -1, pl = -1;

  while (wb_robot_step(TIME_STEP) != -1) {
    int number_of_contacts;
    WbContactPoint *contact_points = wb_supervisor_node_get_contact_points(red_cylinder, false, &number_of_contacts);
    const double *const position = wb_supervisor_node_get_position(red_cylinder);
    const double *const rotation = wb_supervisor_node_get_orientation(red_cylinder);
    int n;
    int u = 0, l = 0;  //  counter for contact points located on upper (resp. lower) cap for the current time step
    for (n = 0; n < number_of_contacts; ++n) {
      // Computing the y-coordinate of the contact point with respect to solid's frame
      const double *cp = contact_points[n].point;
      const double delta[3] = {cp[0] - position[0], cp[1] - position[1], cp[2] - position[2]};
      const double relative_cp_z = rotation[2] * delta[0] + rotation[5] * delta[1] + rotation[8] * delta[2];
      if (fabs(relative_cp_z - RED_CYLINDER_HALF_HEIGHT) <= TOLERANCE)
        ++u;
      else if (fabs(relative_cp_z + RED_CYLINDER_HALF_HEIGHT) <= TOLERANCE)
        ++l;
    }

    // Print contact information only if it has changed since the last time step
    if (pu != u || pl != l) {
      printf("The red cylinder has %d contact points\n", number_of_contacts);
      if (u > 0)
        printf("-  %d  on the upper cap\n", u);
      if (l > 0)
        printf("-  %d  on the lower cap\n", l);
      const int body_counter = number_of_contacts - l - u;
      if (body_counter > 0)
        printf("-  %d  on the body\n", body_counter);
      pu = u;
      pl = l;
    }
  }

  wb_robot_cleanup();
  return 0;
}
