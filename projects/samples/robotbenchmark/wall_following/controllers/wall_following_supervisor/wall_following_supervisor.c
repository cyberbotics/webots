/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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

#include <webots/plugins/robot_window/default.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../include/robotbenchmark.h"
#include "wall_following_metric.h"

#include <stdio.h>
#include <stdlib.h>

#define BENCHMARK_DURATION 60
// #define ALLOW_LABELS true // uncomment to enable labels

static WallFollowingMetric *generate_metric_from_proto(const double *robot_starting_position) {
  WallFollowingMetric *metric = NULL;

  const char *custom_data = wb_robot_get_custom_data();

  int nb_angles;

  if (sscanf(custom_data, "%d", &nb_angles) != 1 || nb_angles == 0) {
    printf("Invalid custom_data field!\n");
    return NULL;
  }

  int *angles = calloc(nb_angles, sizeof(int));

  if (angles) {
    int i;
    for (i = 0; i < nb_angles; ++i) {
      custom_data = strpbrk(custom_data, ",") + 1;
      if (sscanf(custom_data, "%d", &angles[i]) != 1) {
        printf("Invalid custom_data field!\n");
        free(angles);
        return NULL;
      }
    }

    // Recovers the node corresponding to the LinkedWall PROTO.
    WbNodeRef wall = wb_supervisor_node_get_from_def("WALL");
    WbFieldRef translation_field = wb_supervisor_node_get_field(wall, "translation");
    WbFieldRef rotation_field = wb_supervisor_node_get_field(wall, "rotation");
    WbFieldRef length_field = wb_supervisor_node_get_field(wall, "length");

    // Recovers the position of the wall.
    const double *translation = wb_supervisor_field_get_sf_vec3f(translation_field);
    const double first_segment_position[2] = {translation[2], translation[0]};

    // Recover the orientation of the wall.
    double first_segment_orientation = wb_supervisor_field_get_sf_rotation(rotation_field)[3];

    // Recovers the size of one wall segment.
    double segment_length = wb_supervisor_field_get_sf_float(length_field);

    metric = create_new_wall_following_metric(robot_starting_position, first_segment_position, angles, nb_angles,
                                              first_segment_orientation, segment_length, 0.5);
    free(angles);
  }
  return metric;
}

int main(int argc, char **argv) {
  wb_robot_init();

  double time_step = wb_robot_get_basic_time_step();
  WbNodeRef pioneer = wb_supervisor_node_get_from_def("PIONEER");
  const double *position;
  double position2d[2];

  position = wb_supervisor_node_get_position(pioneer);
  position2d[0] = position[2];
  position2d[1] = position[0];

  WallFollowingMetric *metric = generate_metric_from_proto(position2d);

  if (metric) {
#ifdef ALLOW_LABELS
    double distance;
#endif

    char str_buffer[512];

    while (wb_robot_step(time_step) != -1 && wb_robot_get_time() < BENCHMARK_DURATION) {
      position = wb_supervisor_node_get_position(pioneer);
      position2d[0] = position[2];
      position2d[1] = position[0];

#ifdef ALLOW_LABELS
      PathSegment *current_segment = path_get_closest_segment(metric->path, position2d, &distance);

      sprintf(str_buffer, "Distance from path: %f", distance);
      wb_supervisor_set_label(0, str_buffer, 0.01, 0.01, 0.05, 0x0000ff, 0.0, "Lucida Console");

      sprintf(str_buffer, "Current segment: %d", current_segment->index);
      wb_supervisor_set_label(1, str_buffer, 0.01, 0.04, 0.05, 0x0000ff, 0.0, "Lucida Console");
#endif

      wall_following_metric_update(metric, position2d);

#ifdef ALLOW_LABELS
      sprintf(str_buffer, "Current performance: %f", metric->performance);
      wb_supervisor_set_label(2, str_buffer, 0.01, 0.07, 0.05, 0x0000ff, 0.0, "Lucida Console");

      sprintf(str_buffer, "Current time: %f", wb_robot_get_time());
      wb_supervisor_set_label(3, str_buffer, 0.01, 0.1, 0.05, 0x0000ff, 0.0, "Lucida Console");
#endif

      sprintf(str_buffer, "update: %.2f", 100 * metric->performance);
      wb_robot_wwi_send_text(str_buffer);
    }

    wb_robot_wwi_send_text("stop");

    int waiting_answer = 1;

    do {
      const char *answer_message;
      while ((answer_message = wb_robot_wwi_receive_text())) {
        if (strncmp(answer_message, "record:", 7) == 0) {
          robotbenchmark_record(answer_message, "wall_following", metric->performance);
          waiting_answer = 0;
        } else if (strcmp(answer_message, "exit") == 0)
          waiting_answer = 0;
      }

    } while (wb_robot_step(time_step) != -1 && waiting_answer);

    wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
    free_wall_following_metric(metric);
  } else
    printf("Failed to allocate memory for the metric!\n");

  wb_robot_cleanup();
  return 0;
}
