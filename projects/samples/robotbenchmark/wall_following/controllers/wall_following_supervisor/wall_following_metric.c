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

#include "wall_following_metric.h"

#include "path.h"
#include "vector.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>  // malloc

// Updates the performance given a new position for the robot.
void wall_following_metric_update(WallFollowingMetric *metric, const double *point) {
  if (!metric || !point)
    return;

  // Recovers the closest segment and its distance to the robot.
  double distance;
  PathSegment *segment = path_get_closest_segment(metric->path, point, &distance);
  if (!segment)
    return;

  // Updates the last visited segment index if necessary.
  if (metric->last_visited_segment < segment->index)
    metric->last_visited_segment = segment->index;

  // Recovers a pointer to the SegmentPerformance corresponding to our PathSegment.
  SegmentPerformance *segment_performance = &metric->segment_performances[segment->index];

  // Updates the average distance and the performance for this segment.
  double sum = (segment_performance->nb_points * segment_performance->average_distance) + distance;
  ++segment_performance->nb_points;
  segment_performance->average_distance = sum / segment_performance->nb_points;

  segment_performance->value = GET_PERFORMANCE(segment_performance->average_distance);

  // Updates the overall performance.
  int i;
  metric->performance = 0;
  segment = metric->path->first;
  for (i = 0; i < metric->path->nb_segments; ++i) {
    // The weight of an individual segment over the performance is relative to its length.
    const double weight = segment->length / metric->path->length;
    if (i <= metric->last_visited_segment) {
      if (metric->segment_performances[i].nb_points > 0)
        metric->performance += weight * metric->segment_performances[i].value;
      // Segments before the last visited segment with 0 points have been skipped,
      // so in this case we don't add anything (performance of 0 for this segment).
    } else
      metric->performance += weight * MIN_PERFORMANCE;

    segment = segment->next;
  }
}

// Returns a pointer to a newly allocated and initilized WallFollowingMetric structure.
// If any argument is invalid, or if the memory allocation failed, a NULL pointer will be returned instead.
WallFollowingMetric *create_new_wall_following_metric(const double *robot_starting_position,
                                                      const double *first_segment_position, const int *angles, int angles_n,
                                                      double first_segment_orientation, double segment_length,
                                                      double target_distance) {
  if (!robot_starting_position || !first_segment_position || !angles)
    return NULL;
  WallFollowingMetric *metric = calloc(1, sizeof(WallFollowingMetric));

  if (metric) {
    metric->path = create_new_path(robot_starting_position);
    if (!metric->path) {
      free_wall_following_metric(metric);
      return NULL;
    }

    // This vector corresponds to the first segment of the wall.
    double robot_first_wall_segment_vector[2] = {segment_length * cos(first_segment_orientation),
                                                 segment_length * sin(first_segment_orientation)};

    // This vector represents the distance between the starting position of the robot and the first segment
    // of the wall.
    double robot_to_wall_vector[2];
    vector_substract(robot_to_wall_vector, first_segment_position, robot_starting_position);
    double robot_to_wall_distance = vector_get_norm(robot_to_wall_vector);

    // We can now compute the first segment of the path.
    double robot_first_step_distance = robot_to_wall_distance - target_distance;
    double robot_current_step_vector[2];
    vector_product(robot_current_step_vector, robot_to_wall_vector, robot_first_step_distance / robot_to_wall_distance);

    // Adds the first segment to the path.
    path_add_line_segment(metric->path, robot_current_step_vector);

    // The robot is supposed to reach the wall near the middle of the first segment of the wall, so the second segment of the
    // path will be half of the first segment of the wall.
    vector_product(robot_current_step_vector, robot_first_wall_segment_vector, 0.5);
    double current_step_norm = 0.5 * segment_length;

    // Various variables used during the loop.
    double angle_radiant;
    double arc_center[2];
    double segment_crop;
    int i;
    int current_orientation = 0;

    for (i = 0; i < angles_n; ++i) {
      // Checks the angle is valid.
      if (angles[i] < WALL_ANGLE_MIN || angles[i] > WALL_ANGLE_MAX) {
        fprintf(stderr, "WARNING! Wall angle %d is invalid, supervisor can't compute a correct path!\n", angles[i]);
        free_wall_following_metric(metric);
        return NULL;
      }

      if (angles[i] >= 0) {
        // In this case we can add the previous segment without cropping it.
        path_add_line_segment(metric->path, robot_current_step_vector);

        // For a positive angle, the robot should curve around the edge, so we will
        // add a curve segment to the path.
        if (angles[i] > 0) {
          // Computes the center of the arc for the curve segment.
          angle_radiant = (M_PI * (current_orientation + 12)) / 24;
          arc_center[0] = metric->path->end[0] + target_distance * cos(angle_radiant);
          arc_center[1] = metric->path->end[1] + target_distance * sin(angle_radiant);

          // Adds the curve to the path.
          path_add_curve_segment(metric->path, arc_center, (M_PI * angles[i]) / 24);

          // Updates the orientation.
          current_orientation += angles[i];
          if (current_orientation >= 48)
            current_orientation -= 48;
        }

        // We compute the next path segment.
        angle_radiant = (M_PI * current_orientation) / 24;
        robot_current_step_vector[0] = segment_length * cos(angle_radiant);
        robot_current_step_vector[1] = segment_length * sin(angle_radiant);
        current_step_norm = segment_length;
      } else {
        // For a negative angle, we need to crop the last part of the segment, because we have a concave edge.
        angle_radiant = (M_PI * angles[i]) / 24;

        // How much we need to crop.
        segment_crop = target_distance / tan((M_PI + angle_radiant) / 2);

        // Updates the segment and adds it to the path.
        vector_product(robot_current_step_vector, robot_current_step_vector,
                       (current_step_norm - segment_crop) / current_step_norm);
        path_add_line_segment(metric->path, robot_current_step_vector);

        // Updates the orientation.
        current_orientation += angles[i];

        if (current_orientation < 0)
          current_orientation += 48;

        // We compute the next path segment (and crop it).
        angle_radiant = (M_PI * current_orientation) / 24;
        current_step_norm = segment_length - segment_crop;
        robot_current_step_vector[0] = current_step_norm * cos(angle_radiant);
        robot_current_step_vector[1] = current_step_norm * sin(angle_radiant);
      }
    }

    // The last segment of the path ends next to the next segment of the wall.
    path_add_line_segment(metric->path, robot_current_step_vector);

    // Allocates an array for the performance data of each segment.
    metric->segment_performances = calloc(metric->path->nb_segments, sizeof(SegmentPerformance));
    if (!metric->segment_performances) {
      free_wall_following_metric(metric);
      return NULL;
    }

    // Array initialization.
    metric->performance = 0;
    metric->last_visited_segment = 0;
    for (i = 0; i < metric->path->nb_segments; ++i) {
      metric->segment_performances[i].nb_points = 0;
      metric->segment_performances[i].average_distance = 0;
    }
  }

  return metric;
}

// Frees all resources associated to this metric.
void free_wall_following_metric(WallFollowingMetric *metric) {
  if (metric) {
    free_path(metric->path);
    free(metric->segment_performances);
    free(metric);
  }
}
