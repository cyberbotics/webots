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

#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

typedef struct wbu_quaternion {
  double w;
  double x;
  double y;
  double z;
} WbuQuaternion;

WbuQuaternion wbu_quaternion_zero();
WbuQuaternion wbu_quaternion(double w, double x, double y, double z);
WbuQuaternion wbu_quaternion_normalize(WbuQuaternion q);
WbuQuaternion wbu_quaternion_multiply(WbuQuaternion q1, WbuQuaternion q2);  // This returns q1 * q2
WbuQuaternion wbu_quaternion_conjugate(WbuQuaternion q);
WbuQuaternion wbu_quaternion_from_axis_angle(double x, double y, double z, double angle);
void wbu_quaternion_to_axis_angle(WbuQuaternion q, double *axis_angle);
void wbu_quaternion_print(WbuQuaternion q);

#endif /* QUATERNION_H */
