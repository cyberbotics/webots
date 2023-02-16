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

#ifndef WB_VECTOR4_HPP
#define WB_VECTOR4_HPP

//
// Description: 4D vector
//   Homogeneous coordinates vector designed to be used with WbMatrix4
//

#include "WbPrecision.hpp"

#include <QtCore/QString>
#include <QtCore/QTextStream>

class WbVector3;

class WbVector4 {
public:
  // construct as a [0 0 0 1] vector
  WbVector4() : mX(0.0), mY(0.0), mZ(0.0), mW(1.0) {}

  // construct from other vectors
  WbVector4(double x, double y, double z, double w = 1.0) : mX(x), mY(y), mZ(z), mW(w) {}
  WbVector4(const WbVector4 &v) : mX(v.mX), mY(v.mY), mZ(v.mZ), mW(v.mW) {}
  explicit WbVector4(const WbVector3 &v, double w = 1.0);
  explicit WbVector4(const double v[4]) : mX(v[0]), mY(v[1]), mZ(v[2]), mW(v[3]) {}
  explicit WbVector4(const float v[4]) : mX(v[0]), mY(v[1]), mZ(v[2]), mW(v[3]) {}

  // pointer representation
  const double *ptr() const { return &mX; }

  // getters
  double x() const { return mX; }
  double y() const { return mY; }
  double z() const { return mZ; }
  double w() const { return mW; }
  double operator[](int index) const { return *(&mX + index); }

  // setters
  void setX(double x) { mX = x; }
  void setY(double y) { mY = y; }
  void setZ(double z) { mZ = z; }
  void setW(double w) { mW = w; }
  void setXyzw(double x, double y, double z, double w) {
    mX = x;
    mY = y;
    mZ = z;
    mW = w;
  }
  void setXyzw(const double v[4]) {
    mX = v[0];
    mY = v[1];
    mZ = v[2];
    mW = v[3];
  }
  void setXyzw(const float v[4]) {
    mX = v[0];
    mY = v[1];
    mZ = v[2];
    mW = v[3];
  }
  double &operator[](int index) { return *(&mX + index); }

  // unary plus and minus signs: +v, -v
  const WbVector4 &operator+() const { return *this; }
  WbVector4 operator-() const { return WbVector4(-mX, -mY, -mZ, -mW); }

  // vector vector operations: v+w, v-w, v+=w, v-=w, v*=w
  WbVector4 operator+(const WbVector4 &v) const { return WbVector4(mX + v.mX, mY + v.mY, mZ + v.mZ, mW + v.mW); }
  WbVector4 operator-(const WbVector4 &v) const { return WbVector4(mX - v.mX, mY - v.mY, mZ - v.mZ, mW - v.mW); }
  WbVector4 &operator+=(const WbVector4 &v) {
    mX += v.mX;
    mY += v.mY;
    mZ += v.mZ;
    mW += v.mW;
    return *this;
  }
  WbVector4 &operator-=(const WbVector4 &v) {
    mX -= v.mX;
    mY -= v.mY;
    mZ -= v.mZ;
    mW -= v.mW;
    return *this;
  }
  WbVector4 &operator*=(const WbVector4 &v) {
    mX *= v.mX;
    mY *= v.mY;
    mZ *= v.mZ;
    mW *= v.mW;
    return *this;
  }

  // vector scalar operations: v*s, v/s, s*v, s/v, v*=s, v/= s
  WbVector4 operator*(double d) const { return WbVector4(mX * d, mY * d, mZ * d, mW * d); }
  WbVector4 operator/(double d) const {
    double inv = 1.0 / d;
    return WbVector4(mX * inv, mY * inv, mZ * inv, mW * inv);
  }
  friend WbVector4 operator*(double d, const WbVector4 &v) { return WbVector4(d * v.mX, d * v.mY, d * v.mZ, d * v.mW); }
  friend WbVector4 operator/(double d, const WbVector4 &v) { return WbVector4(d / v.mX, d / v.mY, d / v.mZ, d / v.mW); }
  WbVector4 &operator*=(double d) {
    mX *= d;
    mY *= d;
    mZ *= d;
    mW *= d;
    return *this;
  }
  WbVector4 &operator/=(double d) {
    double inv = 1.0 / d;
    mX *= inv;
    mY *= inv;
    mZ *= inv;
    mW *= inv;
    return *this;
  }

  // assignement: v = w
  WbVector4 &operator=(const WbVector4 &v) {
    mX = v.mX;
    mY = v.mY;
    mZ = v.mZ;
    mW = v.mW;
    return *this;
  }

  // normalization: |w| = 1.0
  void normalize() {
    double inv = 1.0 / mW;
    mX *= inv;
    mY *= inv;
    mZ *= inv;
    mW = 1.0;
  }
  WbVector4 normalized() const {
    double inv = 1.0 / mW;
    return WbVector4(mX * inv, mY * inv, mZ * inv, 1.0);
  }
  WbVector3 toVector3() const;

  double dot(const WbVector4 &v) const { return mX * v.mX + mY * v.mY + mZ * v.mZ + v.mW * mW; }

  // vector comparison
  bool operator==(const WbVector4 &v) const { return mX == v.mX && mY == v.mY && mZ == v.mZ && mW == v.mW; }
  bool operator!=(const WbVector4 &v) const { return mX != v.mX || mY != v.mY || mZ != v.mZ || mW != v.mW; }

  // text conversion
  QString toString(WbPrecision::Level level = WbPrecision::DOUBLE_MAX) const {
    return QString("%1 %2 %3 %4")
      .arg(WbPrecision::doubleToString(mX, level))
      .arg(WbPrecision::doubleToString(mY, level))
      .arg(WbPrecision::doubleToString(mZ, level))
      .arg(WbPrecision::doubleToString(mW, level));
  }
  friend QTextStream &operator<<(QTextStream &stream, const WbVector4 &v);

private:
  double mX, mY, mZ, mW;
};

inline QTextStream &operator<<(QTextStream &stream, const WbVector4 &v) {
  stream << v.toString(WbPrecision::DOUBLE_MAX);
  return stream;
}

#endif
