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

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/types.h>

#include "../../../include/robotbenchmark.h"

#include <math.h>
#include <stdio.h>

#define CENTER_X 0
#define CENTER_Z 0

#define MAX_TIME 60.0

static double get_distance_from_center(WbNodeRef node) {
  const double *position = wb_supervisor_node_get_position(node);
  const double distanceX = position[0] - CENTER_X;
  const double distanceZ = position[2] - CENTER_Z;
  return sqrt(distanceX * distanceX + distanceZ * distanceZ);
}

int main(int argc, char **argv) {
  char str_buffer[512];

  wb_robot_init();

  const double time_step = wb_robot_get_basic_time_step();
  const WbNodeRef bb8_robot = wb_supervisor_node_get_from_def("ROBOT_BB-8");
  const WbNodeRef pit = wb_supervisor_node_get_from_def("PIT");

  const double pit_radius = wb_supervisor_field_get_sf_float(wb_supervisor_node_get_field(pit, "pitRadius"));

  double longest_distance = 0;
  double metric = 0.0;
  double time = wb_robot_get_time();

  while (wb_robot_step(time_step) != -1 && longest_distance < pit_radius && time < MAX_TIME) {
    double distance = get_distance_from_center(bb8_robot);
    if (distance > longest_distance)
      longest_distance = distance;
    metric = 0.5 * longest_distance / pit_radius;
    sprintf(str_buffer, "update: %.2f", 100 * metric);
    wb_robot_wwi_send_text(str_buffer);
    time = wb_robot_get_time();
  }

  if (longest_distance > pit_radius) {
    metric = 0.5;
    if (time < MAX_TIME)
      metric += 0.5 * (MAX_TIME - time) / MAX_TIME;
  }

  sprintf(str_buffer, "update: %.2f", 100 * metric);
  wb_robot_wwi_send_text(str_buffer);

  wb_robot_wwi_send_text("stop");

  bool waiting_answer = true;

  do {
    const char *answer_message = wb_robot_wwi_receive_text();

    if (answer_message) {
      if (strncmp(answer_message, "record:", 7) == 0) {
        robotbenchmark_record(answer_message, "pit_escape", metric);
        waiting_answer = false;
      } else if (strcmp(answer_message, "exit") == 0)
        waiting_answer = false;
    }

  } while (wb_robot_step(time_step) != -1 && waiting_answer);

  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
  wb_robot_cleanup();

  return 0;
}
