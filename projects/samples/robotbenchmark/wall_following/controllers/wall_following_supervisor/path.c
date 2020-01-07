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

#include "path.h"
#include "vector.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>  // malloc
#include <string.h>  // memset, memcpy

// Returns the shortest distance between a line segment and a point.
static double get_distance_from_line_segment(const PathSegment *segment, const double *position) {
  if (segment && position) {
    // Computes local coordinates of the point in the segment referential.
    double local_position[2];
    vector_substract(local_position, position, segment->center);

    // Computes the projection of the point onto the segment
    double dot_product = vector_get_dot_product(local_position, segment->unit_direction);
    double local_projection[2];
    double projected_distance = 0;
    int i;
    for (i = 0; i < 2; ++i) {
      local_projection[i] = dot_product * segment->unit_direction[i];
      projected_distance += local_projection[i] * local_projection[i];
    }
    projected_distance = sqrt(projected_distance);

    const double *segment_point, *compare_point = position;

    // If the projection is on the segment, the distance between the point and the segment is the distance
    // between the projection and the point.
    if (projected_distance < segment->radius) {
      segment_point = local_projection;
      compare_point = local_position;
    }
    // Otherwise we take the distance between one extremity of the segment and the point, using the sign of the
    // dot product to choose which extremity to use.
    else if (dot_product < 0)
      segment_point = segment->start;
    else
      segment_point = segment->end;

    return get_distance(segment_point, compare_point);
  }
  return INFINITY;
}

// Returns the shortest distance between a curve segment and a point.
double get_distance_from_curve_segment(const PathSegment *segment, const double *position) {
  if (segment && position) {
    // Computes local coordinates of the point in the segment referential.
    double local_position[2];
    vector_substract(local_position, position, segment->center);

    // Special case, the robot is exactly in the center of the arc.
    // We need the special case because atan2 would give an error with (0,0) as argument.
    if (local_position[0] == 0 && local_position[1] == 0)
      return segment->radius;

    // Angle between the vector from the center of the arc to the point and the X axis.
    double angle = atan2(local_position[1], local_position[0]);

    // We want a value between 0 and 2PI.
    if (angle < 0)
      angle += 2 * M_PI;

    // To avoid corner cases when comparing angles, we will use a referential based on the absolute value of the
    // arc angle. In this referential, a vector with an angle smaller than the arc angle will be in the arc, and
    // outside of the arc with an angle between the arc angle and 2 M_PI.

    // Translate the angle of the vector to our referential.
    double local_angle = angle - segment->start_angle;

    // We want a value between 0 and 2PI.
    if (local_angle < 0)
      local_angle += 2 * M_PI;

    // If the arc angle is negative, we need to take the opposite to be in the same referential.
    if (segment->arc_angle < 0)
      local_angle = -local_angle + 2 * M_PI;

    // If we are in the arc, the distance is the difference between the vector and the radius.
    if (local_angle < segment->abs_arc_angle)
      return fabs(segment->radius - get_distance(position, segment->center));

    // Otherwise we need to compute the distance between the point and one of the extremities
    // to decide which extremity to use, we'll use the precomputed equidistant angle, which is also in our
    // referential.
    else if (local_angle < segment->equidistant_angle)
      return get_distance(position, segment->end);
    else
      return get_distance(position, segment->start);
  }
  return INFINITY;
}

// Array that maps each segment type to its corresponding get_distance function.
static double (*const TYPE_TO_FUNCTION[2])(const PathSegment *, const double *) = {
  get_distance_from_line_segment,
  get_distance_from_curve_segment,
};

// Returns the shortest distance between a segment and a point.
#define GET_DISTANCE_FROM_SEGMENT(segment, position) TYPE_TO_FUNCTION[(segment)->type](segment, position)

// Returns the shortest distance between a path and a point.
double path_get_distance_from_position(const Path *path, const double *position) {
  if (!path || !path->first || !position)
    return INFINITY;

  double closest = INFINITY;
  PathSegment *segment = path->first;

  // We will iterate every segment, and keep the smallest value.
  while (segment) {
    const double distance = GET_DISTANCE_FROM_SEGMENT(segment, position);

    if (distance < closest)
      closest = distance;

    segment = segment->next;
  }

  return closest;
}

// Returns the closest segment from the given position.
// segment_distance is set to the distance between the position and the returned segment,
// or INFINITY if it returned NULL.
PathSegment *path_get_closest_segment(const Path *path, const double *position, double *segment_distance) {
  if (!path || !path->first || !position || !segment_distance) {
    if (segment_distance)
      *segment_distance = INFINITY;
    return NULL;
  }

  double closest = INFINITY;
  PathSegment *closest_segment = NULL;
  PathSegment *segment = path->first;

  // We will iterate every segment, and keep the smallest value.
  while (segment) {
    const double distance = GET_DISTANCE_FROM_SEGMENT(segment, position);

    if (distance < closest) {
      closest = distance;
      closest_segment = segment;
    }

    segment = segment->next;
  }

  *segment_distance = closest;
  return closest_segment;
}

// Returns a pointer to a newly allocated Path.
// Returns NULL if the allocation failed.
Path *create_new_path(const double *start) {
  Path *new_path = malloc(sizeof(Path));
  if (new_path) {
    new_path->first = NULL;
    new_path->length = 0;
    new_path->nb_segments = 0;
    if (start)
      memcpy(new_path->start, start, VECTOR_SIZE);
    else
      memset(new_path->start, 0, VECTOR_SIZE);

    // Updates the end position of the path.
    memcpy(new_path->end, new_path->start, VECTOR_SIZE);
  }
  return new_path;
}

// Frees the resources allocated to this path and all its segments.
void free_path(Path *path) {
  if (path) {
    PathSegment *segment = path->first;
    PathSegment *next = NULL;
    while (segment) {
      next = segment->next;
      free(segment);
      segment = next;
    }
    free(path);
  }
}

// Returns a pointer to a newly allocated segment, may return NULL if the allocation failed.
static PathSegment *path_init_new_segment(Path *path) {
  PathSegment *new_segment = malloc(sizeof(PathSegment));

  // If the allocation did not fail.
  if (new_segment) {
    new_segment->next = NULL;
    // If there exists at least one segment, we need to iterate the segment to find the last one.
    if (path->first) {
      PathSegment *previous = path->first;
      while (previous->next)
        previous = previous->next;

      // The first point of our new segment is the last point of the previous segment.
      memcpy(new_segment->start, previous->end, VECTOR_SIZE);
      previous->next = new_segment;
      new_segment->index = previous->index + 1;
    }
    // Otherwise we add our segment as the first segment of the path.
    else {
      // The first point of our new segment is the first point of the path.
      memcpy(new_segment->start, path->start, VECTOR_SIZE);
      new_segment->index = 0;
      path->first = new_segment;
    }
    ++path->nb_segments;
  }

  return new_segment;
}

// Adds a new line segment to this path. Returns an error in case of a NULL pointer, or if the allocation
// of the new segment failed.
int path_add_line_segment(Path *path, const double *vector) {
  if (!path || !vector)
    return -1;

  PathSegment *new_segment = path_init_new_segment(path);
  if (!new_segment)
    return -2;

  new_segment->type = LINE;

  // Makes a copy of the direction vector.
  memcpy(new_segment->direction, vector, VECTOR_SIZE);

  // Special case when the direction vector is the null vector.
  if (vector[0] == 0 && vector[1] == 0) {
    new_segment->radius = 0;
    new_segment->length = 0;
    memcpy(new_segment->end, new_segment->start, VECTOR_SIZE);
    memcpy(new_segment->center, new_segment->start, VECTOR_SIZE);
    new_segment->unit_direction[0] = 1.0;
    new_segment->unit_direction[1] = 0.0;
  } else {
    int i;

    // The length of the segment is the norm of the direction vector.
    new_segment->length = vector_get_norm(vector);

    // The radius is the distance between the center and the extremities.
    // In the case of a line segment it is simply half the length.
    new_segment->radius = new_segment->length / 2;

    // Precomputes some values useful to compute the distance between the segment and a random
    // point.
    for (i = 0; i < 2; ++i) {
      new_segment->end[i] = new_segment->start[i] + vector[i];
      new_segment->center[i] = new_segment->start[i] + vector[i] / 2;
      new_segment->unit_direction[i] = vector[i] / new_segment->length;
    }
  }

  // Adds the length of the new segment to the total length of the path.
  path->length += new_segment->length;

  // Updates the end position of the path.
  memcpy(path->end, new_segment->end, VECTOR_SIZE);

  return 0;
}

// Adds a new curve segment to this path. Returns an error in case of a NULL pointer, or if the allocation
// of the new segment failed.
int path_add_curve_segment(Path *path, const double *arc_center, double arc_angle) {
  if (!path || !arc_center)
    return -1;

  PathSegment *new_segment = path_init_new_segment(path);
  if (!new_segment)
    return -2;

  new_segment->type = CURVE;

  // Makes a copy of the center point.
  memcpy(new_segment->center, arc_center, VECTOR_SIZE);

  // If the center is the same point as the first point of the arc, it means the radius will be 0, which means
  // all vectors will be the null vectors and angles are meaningless. In this case we simply set all values as 0.
  if (new_segment->start[0] == arc_center[0] && new_segment->start[1] == arc_center[1]) {
    memcpy(new_segment->end, new_segment->start, VECTOR_SIZE);

    new_segment->radius = 0;
    new_segment->length = 0;

    new_segment->start_angle = 0;
    new_segment->end_angle = 0;
    new_segment->arc_angle = 0;
    new_segment->abs_arc_angle = 0;
    new_segment->equidistant_angle = 0;
  } else {
    double local_start[2];
    double local_end[2];

    // Computes the local coordinates of the first point of the segment.
    vector_substract(local_start, new_segment->start, arc_center);

    // The radius will be the norm of the vector we just computed.
    new_segment->radius = vector_get_norm(local_start);

    // Angle between this vector and the X axis.
    new_segment->start_angle = atan2(local_start[1], local_start[0]);

    // We want a value between 0 and 2PI.
    if (new_segment->start_angle < 0)
      new_segment->start_angle += 2 * M_PI;

    // Copies the value of the arc angle.
    new_segment->arc_angle = arc_angle;

    // Angle between the vector pointing to the last point of the segment and the X axis.
    new_segment->end_angle = new_segment->start_angle + arc_angle;

    // We can use this angle to compute said vector.
    local_end[0] = new_segment->radius * cos(new_segment->end_angle);
    local_end[1] = new_segment->radius * sin(new_segment->end_angle);

    // We can then use this to compute the absolute coordinates of the last point of the segment.
    vector_add(new_segment->end, local_end, arc_center);

    // We need the absolute value of the arc angle during the distance computation, so we precompute it here.
    if (arc_angle < 0) {
      new_segment->abs_arc_angle = -arc_angle;
      if (new_segment->end_angle < 0)
        new_segment->end_angle += 2 * M_PI;
    } else {
      new_segment->abs_arc_angle = arc_angle;
      if (new_segment->end_angle > 2 * M_PI)
        new_segment->end_angle -= 2 * M_PI;
    }

    // Special angle needed during the distance computation.
    new_segment->equidistant_angle = new_segment->abs_arc_angle + (2 * M_PI - new_segment->abs_arc_angle) / 2;

    // Computes the length of the arc.
    new_segment->length = new_segment->radius * new_segment->abs_arc_angle;
  }

  // Adds the length of the new segment to the total length of the path.
  path->length += new_segment->length;

  // Updates the end position of the path.
  memcpy(path->end, new_segment->end, VECTOR_SIZE);

  return 0;
}
