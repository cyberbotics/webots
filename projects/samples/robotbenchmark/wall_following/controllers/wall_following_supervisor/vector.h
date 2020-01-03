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

#ifndef VECTOR_H
#define VECTOR_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Size of a vector, in bytes.
#define VECTOR_SIZE (2 * sizeof(double))

// Computes the distance between 2 points.
static inline double get_distance(const double *a, const double *b) {
  if (a && b)
    return sqrt(((a[0] - b[0]) * (a[0] - b[0])) + ((a[1] - b[1]) * (a[1] - b[1])));

  return INFINITY;
}

// Computes the norm of a vector.
static inline double vector_get_norm(const double *v) {
  if (v)
    return sqrt((v[0] * v[0]) + (v[1] * v[1]));

  return 0;
}

// Computes the difference between 2 vectors.
static inline void vector_substract(double *result, const double *u, const double *v) {
  if (u && v && result) {
    result[0] = u[0] - v[0];
    result[1] = u[1] - v[1];
  }
}

// Computes the sum of 2 vectors.
static inline void vector_add(double *result, const double *u, const double *v) {
  if (u && v && result) {
    result[0] = u[0] + v[0];
    result[1] = u[1] + v[1];
  }
}

// Computes the product of a vector and a scalar.
static inline void vector_product(double *result, const double *u, double scalar) {
  if (u && result) {
    result[0] = u[0] * scalar;
    result[1] = u[1] * scalar;
  }
}

// Computes the dot product between 2 vectors.
static inline double vector_get_dot_product(const double *u, const double *v) {
  if (u && v)
    return (u[0] * v[0]) + (u[1] * v[1]);

  return 0;
}

#endif  // VECTOR_H
