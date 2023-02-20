// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbRay.hpp"
#include <limits>
#include "WbAffinePlane.hpp"
#include "WbMatrix3.hpp"
#include "WbMatrix4.hpp"

static double EPSILON = 0.000001;

std::pair<bool, double> WbRay::intersects(const WbAffinePlane &p, bool testCull) const {
  double scalarProduct = mDirection.x() * p.a() + mDirection.y() * p.b() + mDirection.z() * p.c();
  if (fabs(scalarProduct) < std::numeric_limits<double>::epsilon())
    return std::pair<bool, double>(false, 0);

  double alpha = p.d() - mOrigin.x() * p.a() - mOrigin.y() * p.b() - mOrigin.z() * p.c();
  if (testCull && (alpha > 0)) {
    // collision with back face
    return std::pair<bool, double>(false, 0);
  }

  return std::pair<bool, double>(true, alpha / scalarProduct);
}

std::pair<bool, double> WbRay::intersects(const WbVector3 &vert0, const WbVector3 &vert1, const WbVector3 &vert2, bool testCull,
                                          double &u, double &v) const {
  // find vectors for two eges sharing vert0
  WbVector3 edge1 = vert1 - vert0;
  WbVector3 edge2 = vert2 - vert0;

  // begin calculating determinant - also used to calculate U parameter
  WbVector3 pvec = direction().cross(edge2);

  // if determinant is near zero, ray lies in plane of triangle
  double det = edge1.dot(pvec);

  double t = 0.0;
  if (testCull) {
    if (det < EPSILON)
      return std::pair<bool, double>(false, 0);

    // calculate distance from vert0 to ray origin
    WbVector3 tvec = origin() - vert0;

    // calculate U parameter and test bounds
    u = tvec.dot(pvec);
    if (u < 0.0 || u > det)
      return std::pair<bool, double>(false, 0);

    // prepare to test V parameter
    WbVector3 qvec = tvec.cross(edge1);

    // calculate V parameter and test bounds
    v = direction().dot(qvec);
    if (v < 0.0 || u + v > det)
      return std::pair<bool, double>(false, 0);

    // calculate t, scale parameters, ray intersects triangle
    t = edge2.dot(qvec);
    double inv_det = 1.0 / det;
    t *= inv_det;
    u *= inv_det;
    v *= inv_det;
  } else {
    if (det > -EPSILON && det < EPSILON)
      return std::pair<bool, double>(false, 0);
    double inv_det = 1.0 / det;

    // calculate distance from vert0 to ray origin
    WbVector3 tvec = origin() - vert0;

    // calculate U parameter and test bounds
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
      return std::pair<bool, double>(false, 0);

    // prepare to test V parameter
    WbVector3 qvec = tvec.cross(edge1);

    // calculate V parameter and test bounds
    v = direction().dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
      return std::pair<bool, double>(false, 0);

    // calculate t, ray intersects triangle
    t = edge2.dot(qvec) * inv_det;
  }
  return std::pair<bool, double>(true, t);
}

std::pair<bool, double> WbRay::intersects(const WbVector3 &center, double radius, bool testCull) const {
  // calculate distance
  WbVector3 q = origin() - center;
  double a = direction().length2();
  double b = 2 * direction().dot(q);
  double c = q.length2() - radius * radius;

  if (testCull && c <= 0)
    // ray origin is inside the sphere
    return std::pair<bool, double>(false, 0);

  double discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    // no intersection
    return std::pair<bool, double>(false, 0);
  } else if (discriminant == 0) {
    // ray is tangent to sphere
    return std::pair<bool, double>(true, -b / (2 * a));
  }

  // ray intersects the sphere in two points
  discriminant = sqrt(discriminant);
  double t1 = (-b - discriminant) / (2 * a);
  double t2 = (-b + discriminant) / (2 * a);
  if (t1 > 0) {
    return std::pair<bool, double>(true, t1);
  } else if (t2 > 0) {
    return std::pair<bool, double>(true, t2);
  }

  // sphere intersected in the wrong direction
  return std::pair<bool, double>(false, 0);
}

std::pair<bool, double> WbRay::intersects(const WbVector3 &minBound, const WbVector3 &maxBound, double &tMin,
                                          double &tMax) const {
  WbVector3 bounds[2];
  bounds[0] = minBound;
  bounds[1] = maxBound;
  WbVector3 invDirection(1.0 / mDirection.x(), 1.0 / mDirection.y(), 1.0 / mDirection.z());

  int sign[3];
  sign[0] = (invDirection.x() < 0);
  sign[1] = (invDirection.y() < 0);
  sign[2] = (invDirection.z() < 0);

  double tymin, tymax, tzmin, tzmax;
  tMin = (bounds[sign[0]].x() - mOrigin.x()) * invDirection.x();
  tMax = (bounds[1 - sign[0]].x() - mOrigin.x()) * invDirection.x();
  tymin = (bounds[sign[1]].y() - mOrigin.y()) * invDirection.y();
  tymax = (bounds[1 - sign[1]].y() - mOrigin.y()) * invDirection.y();

  if ((tMin > tymax) || (tymin > tMax))
    return std::pair<bool, double>(false, 0);
  if (tymin > tMin)
    tMin = tymin;
  if (tymax < tMax)
    tMax = tymax;

  tzmin = (bounds[sign[2]].z() - mOrigin.z()) * invDirection.z();
  tzmax = (bounds[1 - sign[2]].z() - mOrigin.z()) * invDirection.z();

  if ((tMin > tzmax) || (tzmin > tMax))
    return std::pair<bool, double>(false, 0);
  if (tzmin > tMin)
    tMin = tzmin;
  if (tzmax < tMax)
    tMax = tzmax;

  if (tMin < 0)
    return std::pair<bool, double>(true, tMax);

  return std::pair<bool, double>(true, tMin);
}

WbRay WbRay::transformed(const WbMatrix4 &matrix) const {
  return WbRay(matrix * mOrigin, matrix.sub3x3MatrixDot(mDirection));
}
