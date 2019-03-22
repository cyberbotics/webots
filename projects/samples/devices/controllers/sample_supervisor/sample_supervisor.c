/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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

#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

int main(int argc, char *argv[]) {
  WbNodeRef root, node;
  WbFieldRef children, field;
  const double *gravity;
  double location[3] = {0.5, 0.3, 0.5};
  int n, i;

  wb_robot_init();

  root = wb_supervisor_node_get_root();
  children = wb_supervisor_node_get_field(root, "children");
  n = wb_supervisor_field_get_count(children);
  printf("This world contains %d nodes:\n", n);
  for (i = 0; i < n; i++) {
    node = wb_supervisor_field_get_mf_node(children, i);
    printf("-> %s\n", wb_supervisor_node_get_type_name(node));
  }
  printf("\n");
  node = wb_supervisor_field_get_mf_node(children, 0);
  field = wb_supervisor_node_get_field(node, "gravity");
  gravity = wb_supervisor_field_get_sf_vec3f(field);
  printf("WorldInfo.gravity = %g %g %g\n\n", gravity[0], gravity[1], gravity[2]);
  printf("Going to move the location of the PointLight in 8 seconds (simulation time)...\n");
  wb_robot_step(8000);                                 /* wait for 8 seconds */
  node = wb_supervisor_field_get_mf_node(children, 3); /* PointLight */
  field = wb_supervisor_node_get_field(node, "location");
  wb_supervisor_field_set_sf_vec3f(field, location);
  printf("Moved location of the PointLight!\n");
  while (wb_robot_step(32) != -1) {
  }
  wb_robot_cleanup();
  return 0;
}
