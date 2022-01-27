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

#include <webots/mouse.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>

int main(int argc, char **argv) {
  wb_robot_init();

  printf("Run the simulation and left click somewhere "
         "in the 3D window to display the picked 3D coordinate "
         "in the Webots console.\n");

  const int time_step = (int)wb_robot_get_basic_time_step();

  wb_mouse_enable(time_step);
  wb_mouse_enable_3d_position();
  wb_robot_step(time_step);

  bool previous_left = false;
  WbNodeRef previous_selected_node = NULL;
  while (wb_robot_step(time_step) != 1) {
    const WbMouseState mouse_state = wb_mouse_get_state();
    if (mouse_state.left && mouse_state.left != previous_left) {
      printf("Click detected at:\n");
      printf("u: %g\n", mouse_state.u);
      printf("v: %g\n", mouse_state.v);
      printf("x: %g\n", mouse_state.x);
      printf("y: %g\n", mouse_state.y);
      printf("z: %g\n", mouse_state.z);
    }
    previous_left = mouse_state.left;

    const WbNodeRef selected_node = wb_supervisor_node_get_selected();
    if (selected_node && selected_node != previous_selected_node)
      printf("You have selected a '%s' node.\n", wb_supervisor_node_get_type_name(selected_node));
    previous_selected_node = selected_node;
  }

  wb_robot_cleanup();

  return 0;
}
