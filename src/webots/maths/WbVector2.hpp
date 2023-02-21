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

#ifndef WB_VECTOR2_HPP
#define WB_VECTOR2_HPP

//
// Description: 2D vector
//

#include "WbPrecision.hpp"

#include <QtCore/QStringList>

#include <cassert>
#include <cfloat>
#include <cmath>

class WbVector2 {
public:
  // construct as null vector
  WbVector2() : mX(0.0), mY(0.0) {}

  // construct from other vectors
  WbVector2(double x, double y) : mX(x), mY(y) {}
  WbVector2(const WbVector2 &other) : mX(other.mX), mY(other.mY) {}
  explicit WbVector2(const double v[2]) : mX(v[0]), mY(v[1]) {}
  explicit WbVector2(const float v[2]) : mX(v[0]), mY(v[1]) {}
  explicit WbVector2(const QString &string) {
    const QStringList splittedText = string.split(' ');
    assert(splittedText.count() == 2);
    mX = splittedText[0].toDouble();
    mY = splittedText[1].toDouble();
  }

  // pointer representation
  const double *ptr() const { return &mX; }

  // getters
  double x() const { return mX; }
  double y() const { return mY; }
  double operator[](int index) const { return *(&mX + index); }

  // setters
  void setX(double x) { mX = x; }
  void setY(double y) { mY = y; }
  void setXy(double x, double y) {
    mX = x;
    mY = y;
  }
  void setXy(const double v[2]) {
    mX = v[0];
    mY = v[1];
  }
  void setXy(const float v[2]) {
    mX = v[0];
    mY = v[1];
  }
  double &operator[](int index) { return *(&mX + index); }

  // uniry plus and minus signs: +v, -v
  const WbVector2 &operator+() const { return *this; }
  WbVector2 operator-() const { return WbVector2(-mX, -mY); }

  // vector vector operations: v+w, v-w, v+=w, v-=w, v*=w
  WbVector2 operator+(const WbVector2 &v) const { return WbVector2(mX + v.mX, mY + v.mY); }
  WbVector2 operator-(const WbVector2 &v) const { return WbVector2(mX - v.mX, mY - v.mY); }
  WbVector2 &operator+=(const WbVector2 &v) {
    mX += v.mX;
    mY += v.mY;
    return *this;
  }
  WbVector2 &operator-=(const WbVector2 &v) {
    mX -= v.mX;
    mY -= v.mY;
    return *this;
  }
  WbVector2 &operator*=(const WbVector2 &v) {
    mX *= v.mX;
    mY *= v.mY;
    return *this;
  }

  // vector scalar operations: v*s, v/s, s*v, v*=s, v/=s
  WbVector2 operator*(double d) const { return WbVector2(mX * d, mY * d); }
  WbVector2 operator/(double d) const {
    double inv = 1.0 / d;
    return WbVector2(mX * inv, mY * inv);
  }
  friend WbVector2 operator*(double d, const WbVector2 &v) { return WbVector2(d * v.mX, d * v.mY); }
  WbVector2 &operator*=(double d) {
    mX *= d;
    mY *= d;
    return *this;
  }
  WbVector2 &operator/=(double d) {
    double inv = 1.0 / d;
    mX *= inv;
    mY *= inv;
    return *this;
  }

  // assignment: v = w
  WbVector2 &operator=(const WbVector2 &v) {
    mX = v.mX;
    mY = v.mY;
    return *this;
  }

  // length and squared length
  double length() const { return sqrt(mX * mX + mY * mY); }
  double length2() const { return mX * mX + mY * mY; }

  // distance and squared distance
  double distance(const WbVector2 &v) const { return (*this - v).length(); }
  double distance2(const WbVector2 &v) const { return (*this - v).length2(); }

  // returns a unit vector with the same direction: / length
  void normalize() {
    const double l = length();
    if (l)
      *this /= l;
  }
  WbVector2 normalized() const {
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
  }

  // vector comparison
  bool operator==(const WbVector2 &v) const { return mX == v.mX && mY == v.mY; }
  bool operator!=(const WbVector2 &v) const { return mX != v.mX || mY != v.mY; }

  // dot product
  double dot(const WbVector2 &v) const { return mX * v.mX + mY * v.mY; }

  // null test
  bool isNull() const { return mX == 0.0 && mY == 0.0; }

  // text conversion
  QString toString(WbPrecision::Level level = WbPrecision::DOUBLE_MAX) const {
    return QString("%1 %2").arg(WbPrecision::doubleToString(mX, level)).arg(WbPrecision::doubleToString(mY, level));
  }

private:
  double mX, mY;
};

#endif
