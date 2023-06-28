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

/**********************************************
 *
 * Supervisor of the curriculum exercise:
 * about pattern recognition
 *
 * Its role is to place the e-puck at
 * a specific location by pressing on the
 * '1' to '5' (and '0') key stroke
 *
 **********************************************/

#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 32

int main() {
  wb_robot_init(); /* initialize the webots controller library */

  wb_keyboard_enable(TIME_STEP);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    WbNodeRef epuck = wb_supervisor_node_get_from_def("EPUCK");
    WbFieldRef rot = wb_supervisor_node_get_field(epuck, "rotation");
    WbFieldRef pos = wb_supervisor_node_get_field(epuck, "translation");
    double epuck_rot[4] = {0, 0, 1, 0};
    double epuck_pos[3] = {0, 0, 0};

    // get the pressed key
    int key = wb_keyboard_get_key();

    // move and orientate the e-puck according to the pressed key
    switch (key) {
      case '1':
        epuck_pos[0] = 0;     // x-coordinate
        epuck_pos[1] = -0.8;  // y-coordinate
        epuck_rot[3] = 0.0;   // orientation angle
        wb_supervisor_field_set_sf_vec3f(pos, epuck_pos);
        wb_supervisor_field_set_sf_rotation(rot, epuck_rot);
        break;
      case '2':
        epuck_pos[0] = 0;
        epuck_pos[1] = -0.8;
        epuck_rot[3] = 1.57;
        wb_supervisor_field_set_sf_vec3f(pos, epuck_pos);
        wb_supervisor_field_set_sf_rotation(rot, epuck_rot);
        break;
      case '3':
        epuck_pos[0] = 0;
        epuck_pos[1] = -0.8;
        epuck_rot[3] = 3.14;
        wb_supervisor_field_set_sf_vec3f(pos, epuck_pos);
        wb_supervisor_field_set_sf_rotation(rot, epuck_rot);
        break;
      case '4':
        epuck_pos[0] = 0;
        epuck_pos[1] = -0.8;
        epuck_rot[3] = 4.71;
        wb_supervisor_field_set_sf_vec3f(pos, epuck_pos);
        wb_supervisor_field_set_sf_rotation(rot, epuck_rot);
        break;
      case '5':
      case '0':
        epuck_pos[0] = 0.0;
        epuck_pos[2] = 0.0;
        epuck_rot[3] = 0.0;
        wb_supervisor_field_set_sf_vec3f(pos, epuck_pos);
        wb_supervisor_field_set_sf_rotation(rot, epuck_rot);
        break;
      default:
        break;
    }
  }

  wb_robot_cleanup();

  return 0;
}
