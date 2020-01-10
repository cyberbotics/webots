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

#ifndef WALL_FOLLOWING_METRIC_H
#define WALL_FOLLOWING_METRIC_H

#include "path.h"

// Max and min allowed angle increment in the LinkedWall PROTO.
// If a single angle is not between these bounds, the path will not generate.
#define WALL_ANGLE_MIN -20
#define WALL_ANGLE_MAX 23

// Minimum performance for a segment that was not skipped
// (As long as the robot doesn't try to skip a segment, it will always get at least this value no matter
// how far from it it was.)
#define MIN_PERFORMANCE 0.2

// How far from the path to reach the minimal performance.
#define MAX_DISTANCE 0.5

// Converts an average distance to a performance
#define GET_PERFORMANCE(distance) ((distance) > (MAX_DISTANCE) ? (MIN_PERFORMANCE) : 1 - ((distance) / (MAX_DISTANCE)))

typedef struct {
  double value;
  double average_distance;
  int nb_points;
} SegmentPerformance;

typedef struct {
  Path *path;
  double performance;
  SegmentPerformance *segment_performances;
  int last_visited_segment;
} WallFollowingMetric;

void wall_following_metric_update(WallFollowingMetric *metric, const double *point);

WallFollowingMetric *create_new_wall_following_metric(const double *robot_starting_position,
                                                      const double *first_segment_position, const int *angles, int angles_n,
                                                      double first_segment_orientation, double segment_length,
                                                      double target_distance);
void free_wall_following_metric(WallFollowingMetric *metric);

#endif  // WALL_FOLLOWING_METRIC_H
