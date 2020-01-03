/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/utils/default_robot_window.h>
#include "../../../include/robotbenchmark.h"

#define TIME_STEP 128

int main(int argc, char **argv) {
  wb_robot_init();

  WbNodeRef op2 = wb_supervisor_node_get_from_def("BATTERYOP2");

  WbFieldRef op2_translation_field = wb_supervisor_node_get_field(op2, "translation");
  WbFieldRef op2_battery_field = wb_supervisor_node_get_field(op2, "battery");

  double initial_x_position = wb_supervisor_field_get_sf_vec3f(op2_translation_field)[0];
  double distance = 0;

  // Main simulation loop
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *position = wb_supervisor_field_get_sf_vec3f(op2_translation_field);
    if (position[1] < 0.2) {  // the robot has fallen down
      printf("The benchmark stopped because the robot is down.\n");
      break;
    }

    // Compute current distance
    distance = position[0] - initial_x_position;
    if (distance < 0.0)
      distance = 0.0;

    // Get battery level
    const double battery_level = wb_supervisor_field_get_mf_float(op2_battery_field, 0);
    char buffer[32];
    snprintf(buffer, 32, "run:%.3f:%.2f", distance, battery_level);
    wb_robot_wwi_send_text(buffer);

    if (battery_level == 0) {
      printf("The robot has run out of battery.\n");
      break;
    }
  }

  // Notify the robot window that the benchmark is completed
  wb_robot_wwi_send_text("stop");

  // Wait for user credentials and show benchmark score in robot window
  while (wb_robot_step(TIME_STEP) != -1) {
    const char *message = wb_robot_wwi_receive_text();
    if (message) {
      if (strncmp(message, "record:", 7) == 0) {
        robotbenchmark_record(message, "humanoid_marathon", distance);
        break;
      } else if (strcmp(message, "exit") == 0)
        break;
    }
  }

  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);

  wb_robot_cleanup();

  return 0;
}
