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

#ifndef PATH_H
#define PATH_H

typedef enum { LINE, CURVE } SEGMENT_TYPE;

// This represents one segment of a Path.
typedef struct PathSegment_ PathSegment;
struct PathSegment_ {
  SEGMENT_TYPE type;
  PathSegment *next;
  int index;

  // for all segments
  double start[2];
  double end[2];
  double center[2];
  double radius;
  double length;

  // for lines
  double direction[2];
  double unit_direction[2];

  // for curves
  double start_angle;
  double arc_angle;
  double abs_arc_angle;
  double end_angle;
  double equidistant_angle;
};

// This represents the "ideal" path the robot should travel on.
typedef struct {
  PathSegment *first;
  double start[2];
  double end[2];
  double length;
  int nb_segments;
} Path;

double path_get_distance_from_position(const Path *path, const double *position);
PathSegment *path_get_closest_segment(const Path *path, const double *position, double *segment_distance);

Path *create_new_path(const double *start);
void free_path(Path *path);

int path_add_line_segment(Path *path, const double *vector);
int path_add_curve_segment(Path *path, const double *arc_center, double arc_angle);

#endif  // PATH_H
