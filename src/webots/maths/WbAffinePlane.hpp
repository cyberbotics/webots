// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_AFFINE_PLANE_HPP
#define WB_AFFINE_PLANE_HPP

//
// Description: data structure for a 3D affine plane defined by the equation ax + by + cz = d
//

#include <cmath>
#include "WbVector3.hpp"

class WbAffinePlane {
public:
  // construct an identity quaternion
  WbAffinePlane() : mA(1.0), mB(0.0), mC(0.0), mD(0.0) {}

  // construct from coefficients
  WbAffinePlane(const WbAffinePlane &other) : mA(other.mA), mB(other.mB), mC(other.mC), mD(other.mD) {}
  WbAffinePlane(double a, double b, double c, double d) : mA(a), mB(b), mC(c), mD(d) {}
  explicit WbAffinePlane(const double p[4]) : mA(p[0]), mB(p[1]), mC(p[2]), mD(p[3]) {}
  // construct from a normal vector and a scalar
  WbAffinePlane(const WbVector3 &v, double d) : mA(v.x()), mB(v.y()), mC(v.z()), mD(d) {}
  // construct from a normal vector and a point
  WbAffinePlane(const WbVector3 &v, const WbVector3 &P) {
    mA = v.x(), mB = v.y(), mC = v.z(), mD = v.x() * P.x() + v.y() * P.y() + v.z() * P.z();
  }
  // construct from 3 points
  WbAffinePlane(const WbVector3 &P, const WbVector3 &Q, const WbVector3 &R) { from3Points(P, Q, R); }

  void from3Points(const WbVector3 &P, const WbVector3 &Q, const WbVector3 &R);

  // getters
  double a() const { return mA; }
  double b() const { return mB; }
  double c() const { return mC; }
  double d() const { return mD; }
  WbVector3 normal() const { return WbVector3(mA, mB, mC); }

  // setters
  void setA(double a) { mA = a; }
  void setB(double b) { mB = b; }
  void setC(double c) { mC = c; }
  void setD(double d) { mD = d; }

  //
  void redefine(double a, double b, double c, double d) {
    mA = a;
    mB = b;
    mC = c;
    mD = d;
  }
  void redefine(const double p[4]) {
    mA = p[0];
    mB = p[1];
    mC = p[2];
    mD = p[3];
  }
  void redefine(const WbVector3 &P, const WbVector3 &Q, const WbVector3 &R) { from3Points(P, Q, R); }
  void redefine(const WbVector3 &v, const WbVector3 &P) {
    mA = v.x(), mB = v.y(), mC = v.z(), mD = v.x() * P.x() + v.y() * P.y() + v.z() * P.z();
  }
  // normalize
  void normalize();

  // 3 special planes
  void setXZ() {
    mA = 0.0;
    mB = 1.0;
    mC = 0.0;
    mD = 0.0;
  }
  void setXZ(double d) {
    mA = 0.0;
    mB = 1.0;
    mC = 0.0;
    mD = d;
  }
  void setXY() {
    mA = 0.0;
    mB = 0.0;
    mC = 1.0;
    mD = 0.0;
  }
  void setXY(double d) {
    mA = 0.0;
    mB = 0.0;
    mC = 1.0;
    mD = d;
  }
  void setYZ() {
    mA = 1.0;
    mB = 0.0;
    mC = 0.0;
    mD = 0.0;
  }
  void setYZ(double d) {
    mA = 1.0;
    mB = 0.0;
    mC = 0.0;
    mD = d;
  }

  // assignment: P1 = P2
  WbAffinePlane &operator=(const WbAffinePlane &p) {
    mA = p.mA;
    mB = p.mB;
    mC = p.mC;
    mD = p.mD;
    return *this;
  }

  // pseudo-distance to a 3D point
  // this is the signed distance from P to the plane if (a, b, c) is a unit vector, the sign is positive if P lies in the upper
  // plane determined by (a, b, c) and negative otherwise.
  double distance(const WbVector3 &v) const { return mA * v.x() + mB * v.y() + mC * v.z() - mD; }
  double distance(const double v[3]) const { return mA * v[0] + mB * v[1] + mC * v[2] - mD; }
  double distance(double x, double y, double z) const { return mA * x + mB * y + mC * z - mD; }

  // project a vector on this plane, returns the project vector
  WbVector3 vectorProjection(const WbVector3 &v) const { return v - normal() * (v.dot(normal()) / normal().length2()); }

private:
  double mA, mB, mC, mD;
};

inline void WbAffinePlane::normalize() {
  double length = mA * mA + mB * mB + mC * mC;
  if (length == 0.0) {
    mA = 1.0;
    return;
  }
  length = 1.0 / sqrt(length);
  mA *= length;
  mB *= length;
  mC *= length;
  mD *= length;
}

#endif
