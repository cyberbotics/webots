#ifndef VECTOR3_PRIVATE_H
#define VECTOR3_PRIVATE_H

#include <math.h>
#include "quaternion_private.h"

typedef enum {
  X_AXIS, Y_AXIS, Z_AXIS, ZERO
} WbuVector3Type;


typedef struct wbu_vector3 {
  double x;
  double y;
  double z;
} WbuVector3;


WbuVector3 wbu_vector3(WbuVector3Type type);
double     wbu_vector3_length(WbuVector3 v);
WbuVector3 wbu_vector3_normalize(WbuVector3 v);
double     wbu_vector3_dot(WbuVector3 v1, WbuVector3 v2);
WbuVector3 wbu_vector3_cross(WbuVector3 v1, WbuVector3 v2);
WbuVector3 wbu_vector3_rotate_by_quaternion(WbuVector3 v, WbuQuaternion q);

#endif /* VECTOR3_PRIVATE_H */
