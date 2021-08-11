#ifndef QUATERNION_PRIVATE_H
#define QUATERNION_PRIVATE_H

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
WbuQuaternion wbu_quaternion_multiply(WbuQuaternion q1, WbuQuaternion q2); // This returns q1 * q2
WbuQuaternion wbu_quaternion_conjugate(WbuQuaternion q);
WbuQuaternion wbu_quaternion_from_axis_angle(double x, double y, double z, double angle);
void wbu_quaternion_to_axis_angle(WbuQuaternion q, double *axis_angle);
void wbu_quaternion_print(WbuQuaternion q);

#endif /* QUATERNION_PRIVATE_H */
