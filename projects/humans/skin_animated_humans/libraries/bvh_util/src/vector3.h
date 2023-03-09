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

#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>
#include "quaternion.h"

typedef enum { X_AXIS, Y_AXIS, Z_AXIS, ZERO } WbuVector3Type;

typedef struct wbu_vector3 {
  double x;
  double y;
  double z;
} WbuVector3;

WbuVector3 wbu_vector3(WbuVector3Type type);
double wbu_vector3_length(WbuVector3 v);
WbuVector3 wbu_vector3_normalize(WbuVector3 v);
double wbu_vector3_dot(WbuVector3 v1, WbuVector3 v2);
WbuVector3 wbu_vector3_cross(WbuVector3 v1, WbuVector3 v2);
WbuVector3 wbu_vector3_rotate_by_quaternion(WbuVector3 v, WbuQuaternion q);

#endif /* VECTOR3_H */
