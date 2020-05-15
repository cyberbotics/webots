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

/***************************************************************************************************/
/* Description:  Common definition of the 'WbTransformNodeObject' for both the C and C++ APIs */
/**************************************************************************************************/

#ifndef WB_TRANSFORM_NODE_OBJECT_H
#define WB_TRANSFORM_NODE_OBJECT_H

#include <stdbool.h>

typedef struct _WbTransformNodeObject WbTransformNodeObject;

typedef enum { WB_TF_NODE_JOINT = 0, WB_TF_NODE_LINK } WbTransformNodeObjectType;

struct _WbTransformNodeObject {
  int id;
  char *name;
  WbTransformNodeObjectType type;
  int n_children;
  WbTransformNodeObject **children;
  WbTransformNodeObject *parent;

  // LINK only fields
  bool is_device;
  double translation[3];
  double rotation[9];

  // JOINT only fields
  double axis[3];
  double position;
  char *position_sensor_name;
  char *position_sensor_name_2;  // HingeJoint2
  char *position_sensor_name_3;  // BallJoint
};

#endif /* WB_TRANSFORM_NODE_OBJECT_H */
