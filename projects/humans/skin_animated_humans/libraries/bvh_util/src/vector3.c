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

#include "vector3.h"
#include <stdio.h>
#include <stdlib.h>

double wbu_vector3_length(WbuVector3 v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

WbuVector3 wbu_vector3_normalize(WbuVector3 v) {
  const double n = wbu_vector3_length(v);
  WbuVector3 res;
  res.x = v.x / n;
  res.y = v.y / n;
  res.z = v.z / n;
  return res;
}

double wbu_vector3_dot(WbuVector3 v1, WbuVector3 v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

WbuVector3 wbu_vector3_cross(WbuVector3 v1, WbuVector3 v2) {
  WbuVector3 result;
  result.x = v1.y * v2.z - v1.z * v2.y;
  result.y = v1.z * v2.x - v1.x * v2.z;
  result.z = v1.x * v2.y - v1.y * v2.x;
  return result;
}

WbuVector3 wbu_vector3_rotate_by_quaternion(WbuVector3 v, WbuQuaternion q) {
  q = wbu_quaternion_normalize(q);
  WbuQuaternion q_ = wbu_quaternion_conjugate(q);
  q_ = wbu_quaternion_normalize(q_);

  WbuQuaternion p;
  p.w = 0;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;

  WbuQuaternion w;
  w = wbu_quaternion_multiply(q, wbu_quaternion_multiply(p, q_));
  WbuVector3 res;
  res.x = w.x;
  res.y = w.y;
  res.z = w.z;
  return res;
}

WbuVector3 wbu_vector3(WbuVector3Type type) {
  WbuVector3 v;
  v.x = 0;
  v.y = 0;
  v.z = 0;

  if (type == X_AXIS)
    v.x = 1;
  else if (type == Y_AXIS)
    v.y = 1;
  else if (type == Z_AXIS)
    v.z = 1;
  return v;
}
