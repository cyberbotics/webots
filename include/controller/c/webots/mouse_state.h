/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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

/**************************************************************************************/
/* Description:  Common definition of the 'WbMouseState' for both the C and C++ APIs. */
/**************************************************************************************/

#ifndef WB_MOUSE_STATE_H
#define WB_MOUSE_STATE_H

#include "types.h"

typedef struct {
  // mouse 2D position in the 3D window
  double u;
  double v;
  // mouse 3D position
  double x;
  double y;
  double z;
  // mouse buttons state
  bool left;
  bool middle;
  bool right;
} WbMouseState;

#endif /* WB_MOUSE_STATE_H */
