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

#include "tiny_math.h"
#include <math.h>

void vector3_set_values(Vector3 *vect, double u, double v, double w) {
  vect->u = u;
  vect->v = v;
  vect->w = w;
}

void matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv,
                         double cw) {
  vector3_set_values(&(m->a), au, av, aw);
  vector3_set_values(&(m->b), bu, bv, bw);
  vector3_set_values(&(m->c), cu, cv, cw);
}

void matrix33_set_identity(Matrix33 *m) {
  matrix33_set_values(m, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

void matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v) {
  res->u = m->a.u * v->u + m->b.u * v->v + m->c.u * v->w;
  res->v = m->a.v * v->u + m->b.v * v->v + m->c.v * v->w;
  res->w = m->a.w * v->u + m->b.w * v->v + m->c.w * v->w;
}

double vector2_norm(const Vector2 *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

void vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

double vector2_angle(const Vector2 *v1, const Vector2 *v2) {
  return atan2(v2->v, v2->u) - atan2(v1->v, v1->u);
}

double bound(double v, double a, double b) {
  return (v > b) ? b : (v < a) ? a : v;
}
