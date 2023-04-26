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

#include <math.h>
#include <stdio.h>

#include <webots/robot.h>
#include <webots/supervisor.h>

int main(int argc, char *argv[]) {
  WbNodeRef node;
  WbFieldRef field;
  int i;

  wb_robot_init();

  // get the root children field
  const WbNodeRef root_node = wb_supervisor_node_get_root();
  const WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
  const int n = wb_supervisor_field_get_count(root_children_field);
  printf("This world contains %d nodes:\n", n);

  // check what type of nodes are present in the world
  for (i = 0; i < n; i++) {
    node = wb_supervisor_field_get_mf_node(root_children_field, i);
    printf("-> %s\n", wb_supervisor_node_get_type_name(node));
  }
  printf("\n");

  // get the content of the 'gravity' field of the 'WorldInfo' node
  node = wb_supervisor_field_get_mf_node(root_children_field, 0);
  field = wb_supervisor_node_get_field(node, "gravity");
  const double gravity = wb_supervisor_field_get_sf_float(field);
  printf("WorldInfo.gravity = %g\n\n", gravity);

  // use a label to display information in the 3D view
  wb_supervisor_set_label(0, "Going to move the location of the PointLight\nin 2 seconds (simulation time)...", 0.0, 0.0, 0.1,
                          0x00FF00, 0.1, "Georgia");
  // move the 'PointLight' node after waiting 2 seconds
  printf("Going to move the location of the PointLight in 2 seconds (simulation time)...\n");
  wb_robot_step(2000);                                             // wait for 2 seconds
  node = wb_supervisor_field_get_mf_node(root_children_field, 3);  // PointLight
  field = wb_supervisor_node_get_field(node, "location");
  const double location[3] = {0.5, 0.5, 0.3};
  wb_supervisor_field_set_sf_vec3f(field, location);

  // import a new sphere node after waiting 2 seconds
  wb_supervisor_set_label(0, "Going to import a Sphere in 2 seconds (simulation time)...", 0.0, 0.0, 0.1, 0x00FF00, 0.1,
                          "Georgia");
  printf("Going to import a Sphere in 2 seconds (simulation time)...\n");
  wb_robot_step(2000);
  wb_supervisor_field_import_mf_node_from_string(
    root_children_field, -1,  // import at the end of the root children field
    "Pose { children [ Shape { appearance PBRAppearance { } geometry Sphere { radius 0.1 subdivision 3 } } ] }");

  // main simulation loop
  wb_supervisor_set_label(0, "Going to move the Sphere in 2 seconds (simulation time)...", 0.0, 0.0, 0.1, 0x00FF00, 0.1,
                          "Georgia");
  printf("Going to move the Sphere in 2 seconds (simulation time)...\n");
  wb_robot_step(2000);
  wb_supervisor_set_label(0, "", 0.0, 0.0, 0.0, 0x00FF00, 0.0, "Georgia");
  double translation[3] = {0.0, 0.0, 0.0};
  // get the last node of the root children field (the Sphere)
  node = wb_supervisor_field_get_mf_node(root_children_field, -1);
  field = wb_supervisor_node_get_field(node, "translation");
  while (wb_robot_step(32) != -1) {
    // move the Sphere node in a circle of 0.3m of radius
    translation[0] = 0.3 * cos(wb_robot_get_time());
    translation[1] = 0.3 * sin(wb_robot_get_time());
    wb_supervisor_field_set_sf_vec3f(field, translation);
  }

  wb_robot_cleanup();
  return 0;
}
