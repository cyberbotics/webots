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

#ifndef WB_VECTOR3_HPP
#define WB_VECTOR3_HPP

//
// Description: 3D vector
//

#include "WbPrecision.hpp"

#include <QtCore/QStringList>

#include <cassert>
#include <cfloat>
#include <cmath>

class WbVector3 {
public:
  // construct as null vector
  WbVector3() : mX(0.0), mY(0.0), mZ(0.0) {}

  // construct from other vector
  WbVector3(double x, double y, double z) : mX(x), mY(y), mZ(z) {}
  WbVector3(const WbVector3 &other) : mX(other.mX), mY(other.mY), mZ(other.mZ) {}
  explicit WbVector3(const double v[3]) : mX(v[0]), mY(v[1]), mZ(v[2]) {}
  explicit WbVector3(const float v[3]) : mX(v[0]), mY(v[1]), mZ(v[2]) {}
  explicit WbVector3(const QString &string) {
    const QStringList splittedText = string.split(' ');
    assert(splittedText.count() == 3);
    mX = splittedText[0].toDouble();
    mY = splittedText[1].toDouble();
    mZ = splittedText[2].toDouble();
  }

  // pointer representation
  const double *ptr() const { return &mX; }

  // getters
  double x() const { return mX; }
  double y() const { return mY; }
  double z() const { return mZ; }
  double operator[](int index) const { return *(&mX + index); }

  // setters
  void setX(double x) { mX = x; }
  void setY(double y) { mY = y; }
  void setZ(double z) { mZ = z; }
  void setXyz(double x, double y, double z) {
    mX = x;
    mY = y;
    mZ = z;
  }
  void setXyz(const double v[3]) {
    mX = v[0];
    mY = v[1];
    mZ = v[2];
  }
  void setXyz(const float v[3]) {
    mX = v[0];
    mY = v[1];
    mZ = v[2];
  }
  double &operator[](int index) { return *(&mX + index); }

  // unary plus and minus signs: +v, -v
  const WbVector3 &operator+() const { return *this; }
  WbVector3 operator-() const { return WbVector3(-mX, -mY, -mZ); }
  WbVector3 abs() const { return WbVector3(fabs(mX), fabs(mY), fabs(mZ)); }

  // vector vector operations: v+w, v-w, v*w, v+=w, v-=w, v*=w
  WbVector3 operator+(const WbVector3 &v) const { return WbVector3(mX + v.mX, mY + v.mY, mZ + v.mZ); }
  WbVector3 operator-(const WbVector3 &v) const { return WbVector3(mX - v.mX, mY - v.mY, mZ - v.mZ); }
  WbVector3 operator*(const WbVector3 &v) const { return WbVector3(mX * v.mX, mY * v.mY, mZ * v.mZ); }
  WbVector3 &operator+=(const WbVector3 &v) {
    mX += v.mX;
    mY += v.mY;
    mZ += v.mZ;
    return *this;
  }
  WbVector3 &operator-=(const WbVector3 &v) {
    mX -= v.mX;
    mY -= v.mY;
    mZ -= v.mZ;
    return *this;
  }
  WbVector3 &operator*=(const WbVector3 &v) {
    mX *= v.mX;
    mY *= v.mY;
    mZ *= v.mZ;
    return *this;
  }
  WbVector3 &operator/=(const WbVector3 &v) {
    mX /= v.mX;
    mY /= v.mY;
    mZ /= v.mZ;
    return *this;
  }

  // vector scalar operations: v*s, v/s, s*v, v*=s, v/=s
  WbVector3 operator*(double d) const { return WbVector3(mX * d, mY * d, mZ * d); }
  WbVector3 operator/(double d) const {
    double inv = 1.0 / d;
    return WbVector3(mX * inv, mY * inv, mZ * inv);
  }
  friend WbVector3 operator*(double d, const WbVector3 &v) { return WbVector3(d * v.mX, d * v.mY, d * v.mZ); }
  WbVector3 &operator*=(double d) {
    mX *= d;
    mY *= d;
    mZ *= d;
    return *this;
  }
  WbVector3 &operator/=(double d) {
    double inv = 1.0 / d;
    mX *= inv;
    mY *= inv;
    mZ *= inv;
    return *this;
  }

  // assignment: v = w
  WbVector3 &operator=(const WbVector3 &v) {
    mX = v.mX;
    mY = v.mY;
    mZ = v.mZ;
    return *this;
  }

  // length and squared length (magnitude)
  double length() const { return sqrt(mX * mX + mY * mY + mZ * mZ); }
  double length2() const { return mX * mX + mY * mY + mZ * mZ; }

  // distance and squared distance
  double distance(const WbVector3 &v) const { return (*this - v).length(); }
  double distance2(const WbVector3 &v) const { return (*this - v).length2(); }

  // normalization: |length| = 1.0
  void normalize() {
    const double l = length();
    if (l)
      *this /= l;
  }
  WbVector3 normalized() const {
    const double l = length();
    return l ? *this / l : *this;
  }

  void clamp(double min = -FLT_MAX, double max = FLT_MAX) {
    if (mX > max)
      mX = max;
    else if (mX < min)
      mX = min;
    if (mY > max)
      mY = max;
    else if (mY < min)
      mY = min;
    if (mZ > max)
      mZ = max;
    else if (mZ < min)
      mZ = min;
  }

  WbVector3 rounded(WbPrecision::Level level) const {
    return WbVector3(WbPrecision::roundValue(mX, level), WbPrecision::roundValue(mY, level),
                     WbPrecision::roundValue(mZ, level));
  }

  // dot product
  double dot(const WbVector3 &v) const { return mX * v.mX + mY * v.mY + mZ * v.mZ; }

  // cross product
  WbVector3 cross(const WbVector3 &v) const {
    return WbVector3(mY * v.mZ - mZ * v.mY, mZ * v.mX - mX * v.mZ, mX * v.mY - mY * v.mX);
  }

  // angle between two vectors (in radians)
  double angle(const WbVector3 &v) const {
    const double l = length2();
    const double lv = v.length2();
    const double s = (l && lv) ? dot(v) / sqrt(l * lv) : 0.0;
    assert(std::abs(s) < 1.0000000001);
    return (s >= 1.0) ? 0 : (s <= -1.0) ? M_PI : acos(s);
  }

  // vector comparison
  bool almostEquals(const WbVector3 &v, double tolerance = WbPrecision::DOUBLE_EQUALITY_TOLERANCE) const {
    return std::abs(mX - v.mX) < tolerance && std::abs(mY - v.mY) < tolerance && std::abs(mZ - v.mZ) < tolerance;
  }
  bool operator==(const WbVector3 &v) const { return mX == v.mX && mY == v.mY && mZ == v.mZ; }
  bool operator!=(const WbVector3 &v) const { return mX != v.mX || mY != v.mY || mZ != v.mZ; }

  // null test
  bool isNull() const { return mX == 0.0 && mY == 0.0 && mZ == 0.0; }

  // validity test for WREN
  bool isNan() const { return std::isnan(mX) || std::isnan(mY) || std::isnan(mZ); }

  void toFloatArray(float *out) const {
    out[0] = static_cast<float>(mX);
    out[1] = static_cast<float>(mY);
    out[2] = static_cast<float>(mZ);
  }

  // test if this point is on a given line segment
  bool isOnEdgeBetweenVertices(const WbVector3 &lineStart, const WbVector3 &lineEnd, const double tolerance = 0.000001) const {
    const WbVector3 lineSegment = lineEnd - lineStart;
    const WbVector3 toPoint = WbVector3(mX, mY, mZ) - lineStart;

    // the points aren't aligned
    if (!lineSegment.cross(toPoint).almostEquals(WbVector3(), tolerance))
      return false;

    // the point isn't on the segment
    if (lineSegment.dot(toPoint) < 0 || lineSegment.dot(toPoint) > lineSegment.length2())
      return false;

    return true;
  }

  // text conversion
  QString toString(WbPrecision::Level level = WbPrecision::DOUBLE_MAX) const {
    return QString("%1 %2 %3")
      .arg(WbPrecision::doubleToString(mX, level))
      .arg(WbPrecision::doubleToString(mY, level))
      .arg(WbPrecision::doubleToString(mZ, level));
  }

private:
  double mX, mY, mZ;
};

#endif
