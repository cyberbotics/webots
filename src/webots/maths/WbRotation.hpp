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

#ifndef WB_ROTATION_HPP
#define WB_ROTATION_HPP

//
// Description: 3D rotation (VRML-like) angle/axis representation
//

#include "WbMatrix3.hpp"
#include "WbPrecision.hpp"
#include "WbQuaternion.hpp"
#include "WbVector3.hpp"

#include <QtCore/QStringList>

#include <cassert>

class WbRotation {
public:
  // construct as identity rotation
  WbRotation() : mX(0.0), mY(0.0), mZ(1.0), mAngle(0.0) {}

  // construct from other types of rotations
  WbRotation(const WbRotation &r) : mX(r.x()), mY(r.y()), mZ(r.z()), mAngle(r.angle()) {}
  WbRotation(double rx, double ry, double rz, double angle) : mX(rx), mY(ry), mZ(rz), mAngle(angle) {}
  explicit WbRotation(const double r[4]) : mX(r[0]), mY(r[1]), mZ(r[2]), mAngle(r[3]) {}
  explicit WbRotation(const WbQuaternion &q) { fromQuaternion(q); }
  explicit WbRotation(const WbMatrix3 &M) { fromMatrix3(M); }
  explicit WbRotation(const QString &string) {
    const QStringList splittedText = string.split(' ');
    assert(splittedText.count() == 4);
    mX = splittedText[0].toDouble();
    mY = splittedText[1].toDouble();
    mZ = splittedText[2].toDouble();
    mAngle = splittedText[3].toDouble();
  }
  WbRotation(const WbVector3 &vx, const WbVector3 &vy, const WbVector3 &vz) { fromBasisVectors(vx, vy, vz); }

  // assign from other types of rotation
  void fromQuaternion(const WbQuaternion &q);
  void fromMatrix3(const WbMatrix3 &M);
  void fromBasisVectors(const WbVector3 &vx, const WbVector3 &vy, const WbVector3 &vz);
  void fromOpenGlMatrix(const double m[16]);

  // conversion into other types
  WbQuaternion toQuaternion() const { return WbQuaternion(axis(), mAngle); }
  WbMatrix3 toMatrix3() const { return WbMatrix3(mX, mY, mZ, mAngle); }

  void toFloatArray(float *rotation) const;

  // set |axis| = 1.0
  void normalizeAxis();

  // normalize angle between 0 and 2*pi
  void normalizeAngle();

  // normalize axis and angle
  void normalize() {
    normalizeAxis();
    normalizeAngle();
  }

  WbRotation rounded(WbPrecision::Level level) const {
    WbRotation rotation(WbPrecision::roundValue(mX, level), WbPrecision::roundValue(mY, level),
                        WbPrecision::roundValue(mZ, level), WbPrecision::roundValue(mAngle, level));
    rotation.normalize();
    return rotation;
  }

  // invalid only if |axis| == 0.0
  bool isValid() const { return !(mX == 0.0 && mY == 0.0 && mZ == 0.0) && !isnan(mAngle); }

  // identity
  bool isIdentity() const { return mAngle == 0.0; }

  // getters
  double x() const { return mX; }
  double y() const { return mY; }
  double z() const { return mZ; }
  double angle() const { return mAngle; }
  WbVector3 axis() const { return WbVector3(mX, mY, mZ); }
  double operator[](int index) const { return *(&mX + index); }
  WbVector3 right() const;
  WbVector3 up() const;
  WbVector3 direction() const;

  // setters
  void setX(double x) { mX = x; }
  void setY(double y) { mY = y; }
  void setZ(double z) { mZ = z; }
  void setAngle(double angle) { mAngle = angle; }
  void setAxis(double x, double y, double z) {
    mX = x;
    mY = y;
    mZ = z;
    normalizeAxis();
  }
  void setAxisAngle(double x, double y, double z, double angle) {
    mX = x;
    mY = y;
    mZ = z;
    mAngle = angle;
    normalizeAxis();
  }

  // assignment
  WbRotation &operator=(const WbRotation &r) {
    mX = r.mX;
    mY = r.mY;
    mZ = r.mZ;
    mAngle = r.mAngle;
    return *this;
  }

  // comparison operators
  // 'almostEquals' use some tolerance when comparing the double values and return true even if the values have inverted signs
  bool almostEquals(const WbRotation &r, double tolerance) const;
  bool operator==(const WbRotation &r) const { return mX == r.mX && mY == r.mY && mZ == r.mZ && mAngle == r.mAngle; }
  bool operator!=(const WbRotation &r) const { return mX != r.mX || mY != r.mY || mZ != r.mZ || mAngle != r.mAngle; }

  // text conversion
  QString toString(WbPrecision::Level level = WbPrecision::Level::DOUBLE_MAX) const {
    return QString("%1 %2 %3 %4")
      .arg(WbPrecision::doubleToString(mX, level))
      .arg(WbPrecision::doubleToString(mY, level))
      .arg(WbPrecision::doubleToString(mZ, level))
      .arg(WbPrecision::doubleToString(mAngle, level));
  }

private:
  double mX, mY, mZ, mAngle;
};

inline void WbRotation::normalizeAxis() {
  if (!isValid()) {
    mX = 0.0;
    mY = 0.0;
    mZ = 1.0;
    return;
  }
  double invl = 1.0 / sqrt(mX * mX + mY * mY + mZ * mZ);
  if (std::abs(invl - 1.0) > WbPrecision::DOUBLE_EQUALITY_TOLERANCE) {
    mX *= invl;
    mY *= invl;
    mZ *= invl;
  }
}

// returns an angle value lying in [-pi, pi]
inline void WbRotation::normalizeAngle() {
  while (mAngle < -M_PI)
    mAngle += 2.0 * M_PI;
  while (mAngle > M_PI)
    mAngle -= 2.0 * M_PI;
}

inline bool WbRotation::almostEquals(const WbRotation &r, double tolerance = WbPrecision::DOUBLE_EQUALITY_TOLERANCE) const {
  if (std::abs(mAngle - r.mAngle) < tolerance) {
    if (std::abs(mAngle) < tolerance)
      return true;  // axis can be different but rotation is the same
    return std::abs(mX - r.mX) < tolerance && std::abs(mY - r.mY) < tolerance && std::abs(mZ - r.mZ) < tolerance;
  }
  if (std::abs(mAngle + r.mAngle) < tolerance) {
    if (std::abs(mAngle) < tolerance)
      return true;  // axis can be different but rotation is the same
    return std::abs(mX + r.mX) < tolerance && std::abs(mY + r.mY) < tolerance && std::abs(mZ + r.mZ) < tolerance;
  }
  return false;
}

#endif
