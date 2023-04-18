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

#include "quaternion.h"

#include <stdio.h>
#include <stdlib.h>

WbuQuaternion wbu_quaternion_zero() {
  WbuQuaternion q;
  q.w = 1.0;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  return q;
}

WbuQuaternion wbu_quaternion(double w, double x, double y, double z) {
  WbuQuaternion res;
  res.w = w;
  res.x = x;
  res.y = y;
  res.z = z;
  return res;
}

WbuQuaternion wbu_quaternion_normalize(WbuQuaternion q) {
  WbuQuaternion res;
  double d = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (d == 0.0) {
    res.w = 1.0;
    res.x = q.x;
    res.y = q.y;
    res.z = q.z;
    return res;
  }
  if (fabs(1.0 - d) < 2.107342e-08)  // 2.107342e-08 magic number (> ULP/2 for IEEE doubles)
    d = 2.0 / (1.0 + d);             // first order Padé approximant
  else
    d = 1.0 / sqrt(d);
  res.w = q.w * d;
  res.x = q.x * d;
  res.y = q.y * d;
  res.z = q.z * d;

  return res;
}

WbuQuaternion wbu_quaternion_multiply(WbuQuaternion q1, WbuQuaternion q2) {
  WbuQuaternion res;
  res.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  res.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  res.y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
  res.z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;
  return res;
}

WbuQuaternion wbu_quaternion_conjugate(WbuQuaternion q) {
  WbuQuaternion res;
  res.w = q.w;
  res.x = -q.x;
  res.y = -q.y;
  res.z = -q.z;
  return wbu_quaternion_normalize(res);
}

WbuQuaternion wbu_quaternion_from_axis_angle(double x, double y, double z, double angle) {
  WbuQuaternion res = wbu_quaternion_zero();
  double l = x * x + y * y + z * z;
  if (l > 0.0) {
    angle *= 0.5;
    l = sin(angle) / sqrt(l);
    res.w = cos(angle);
    res.x = x * l;
    res.y = y * l;
    res.z = z * l;
  }
  // res = wbu_quaternion_normalize(res);
  return res;
}

void wbu_quaternion_to_axis_angle(WbuQuaternion q, double *axis_angle) {
  // if q.w > 1, acos will return nan
  // if this actually happens we should normalize the quaternion here
  if (q.w > 1.0)
    wbu_quaternion_normalize(q);
  if (q.w <= -1.0)
    axis_angle[3] = 2.0 * M_PI;
  else if (q.w < 1.0)
    axis_angle[3] = 2.0 * acos(q.w);
  else
    // q.w could still be slightly greater than 1.0 (floating point inaccuracy)
    axis_angle[3] = 0.0;

  if (axis_angle[3] < 0.0001) {
    // if e[3] close to zero then direction of axis not important
    axis_angle[0] = 0.0;
    axis_angle[1] = 1.0;
    axis_angle[2] = 0.0;
    axis_angle[3] = 0.0;
    return;
  }

  // normalize axes
  const double inv = 1.0 / sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  axis_angle[0] = q.x * inv;
  axis_angle[1] = q.y * inv;
  axis_angle[2] = q.z * inv;
}

void wbu_quaternion_print(WbuQuaternion q) {
  printf("quaternion %f %f %f %f\n", q.w, q.x, q.y, q.z);
}
