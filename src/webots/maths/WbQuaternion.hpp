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

#ifndef WB_QUATERNION_HPP
#define WB_QUATERNION_HPP

//
// Description: quaternion representing a 3D rotation
//

#include "WbMathsUtilities.hpp"
#include "WbPrecision.hpp"
#include "WbVector3.hpp"

#include <QtCore/QString>
#include <QtCore/QTextStream>

#include <cmath>

class WbQuaternion {
public:
  // construct an identity quaternion
  WbQuaternion() : mW(1.0), mX(0.0), mY(0.0), mZ(0.0) {}

  // construct from other quaternion
  WbQuaternion(const WbQuaternion &other) : mW(other.mW), mX(other.mX), mY(other.mY), mZ(other.mZ) {}
  WbQuaternion(double w, double x, double y, double z) : mW(w), mX(x), mY(y), mZ(z) {}
  explicit WbQuaternion(const double q[4]) : mW(q[0]), mX(q[1]), mY(q[2]), mZ(q[3]) {}
  WbQuaternion(double w, const double q[3]) : mW(w), mX(q[0]), mY(q[1]), mZ(q[2]) {}
  WbQuaternion(double w, const WbVector3 &v) {
    mW = w;
    mX = v.x();
    mY = v.y();
    mZ = v.z();
  }
  WbQuaternion(const WbVector3 &vx, const WbVector3 &vy, const WbVector3 &vz) { fromBasisVectors(vx, vy, vz); }

  // construct from Euler axis and angle
  WbQuaternion(const WbVector3 &r, double angle) { fromAxisAngle(r.x(), r.y(), r.z(), angle); }

  // pointer representation (ODE compatible)
  const double *ptr() const { return &mW; }

  // getters
  double w() const { return mW; }
  double x() const { return mX; }
  double y() const { return mY; }
  double z() const { return mZ; }

  // setters
  void setW(double w) { mW = w; }
  void setX(double x) { mX = x; }
  void setY(double y) { mY = y; }
  void setZ(double z) { mZ = z; }

  // normalize
  void normalize();

  double dot(const WbQuaternion &other) const { return mW * other.w() + mX * other.x() + mY * other.y() + mZ * other.z(); }

  // identity
  void setIdentity() {
    mW = 1.0;
    mX = 0.0;
    mY = 0.0;
    mZ = 0.0;
  }
  bool isIdentity() const { return mW == 1.0 && mX == 0.0 && mY == 0.0 && mZ == 0.0; }

  static WbQuaternion slerp(const WbQuaternion &a, const WbQuaternion &b, double slerpAmount);

  // assignment: Q1 = Q2
  WbQuaternion &operator=(const WbQuaternion &q) {
    mW = q.mW;
    mX = q.mX;
    mY = q.mY;
    mZ = q.mZ;
    return *this;
  }

  // quaternion * quaternion multiplication (combine rotations)
  WbQuaternion operator*(const WbQuaternion &q) const;
  WbQuaternion &operator*=(const WbQuaternion &q) {
    *this = *this * q;
    return *this;
  }

  // quaternion * vector 3D multiplication (apply rotation)
  WbVector3 operator*(const WbVector3 &v) const;

  // quaternion conjugacy (inversion for quaternion with unit norm)
  void conjugate() {
    mX = -mX;
    mY = -mY;
    mZ = -mZ;
  }
  WbQuaternion conjugated() const { return WbQuaternion(mW, -mX, -mY, -mZ); }

  // assign from other types of rotation
  void fromAxisAngle(double rx, double ry, double rz, double angle);
  void fromBasisVectors(const WbVector3 &vx, const WbVector3 &vy, const WbVector3 &vz);
  void fromEulerAngles(double yaw, double pitch, double roll);

  // comparison operators
  bool operator==(const WbQuaternion &q) const { return mW == q.mW && mX == q.mX && mY == q.mY && mZ == q.mZ; }
  bool operator!=(const WbQuaternion &q) const { return mW != q.mW || mX != q.mX || mY != q.mY || mZ != q.mZ; }

  // validity test for WREN
  bool isNan() const { return std::isnan(mW) || std::isnan(mX) || std::isnan(mY) || std::isnan(mZ); }
  // text conversion
  QString toString(WbPrecision::Level level) const {
    return QString("%1 %2 %3 %4")
      .arg(WbPrecision::doubleToString(mW, level))
      .arg(WbPrecision::doubleToString(mX, level))
      .arg(WbPrecision::doubleToString(mY, level))
      .arg(WbPrecision::doubleToString(mZ, level));
  }
  friend QTextStream &operator<<(QTextStream &stream, const WbQuaternion &q);

private:
  double mW, mX, mY, mZ;
};

inline void WbQuaternion::fromAxisAngle(double rx, double ry, double rz, double angle) {
  double l = rx * rx + ry * ry + rz * rz;
  if (l > 0.0) {
    angle *= 0.5;
    mW = cos(angle);
    l = sin(angle) / sqrt(l);
    mX = rx * l;
    mY = ry * l;
    mZ = rz * l;
  } else {
    mW = 1.0;
    mX = 0.0;
    mY = 0.0;
    mZ = 0.0;
  }
}

inline void WbQuaternion::fromBasisVectors(const WbVector3 &vx, const WbVector3 &vy, const WbVector3 &vz) {
  if (vx.x() == vy.y() && vx.x() == vz.z() && vx.x() == 1.0) {
    // exception
    mW = 1.0;
    mX = 0.0;
    mY = 0.0;
    mZ = 0.0;
    return;
  }
  double s = 2.0;
  double invS = 1.0;
  if (vx.x() > vy.y() && vx.x() > vz.z()) {  // vx.x is larger than max(vy.y, vz.z)
    s *= sqrt(1.0 + vx.x() - vy.y() - vz.z());
    invS = 1.0 / s;
    mW = (vy.z() - vz.y()) * invS;
    mX = 0.25 * s;
    mY = (vy.x() + vx.y()) * invS;
    mZ = (vx.z() + vz.x()) * invS;
  } else if (vy.y() > vz.z()) {  // vy.y is the largest
    s *= sqrt(1.0 + vy.y() - vz.z() - vx.x());
    invS = 1 / s;
    mW = (vz.x() - vx.z()) * invS;
    mX = (vy.x() + vx.y()) * invS;
    mY = 0.25 * s;
    mZ = (vz.y() + vy.z()) * invS;
  } else {  // vz.z is the largest
    s *= sqrt(1.0 + vz.z() - vx.x() - vy.y());
    invS = 1 / s;
    mW = (vx.y() - vy.x()) * invS;
    mX = (vx.z() + vz.x()) * invS;
    mY = (vz.y() + vy.z()) * invS;
    mZ = 0.25 * s;
  }
}

inline QTextStream &operator<<(QTextStream &stream, const WbQuaternion &q) {
  stream << q.toString(WbPrecision::DOUBLE_MAX);
  return stream;
}

inline WbQuaternion WbQuaternion::operator*(const WbQuaternion &q) const {
  return WbQuaternion(mW * q.mW - mX * q.mX - mY * q.mY - mZ * q.mZ, mW * q.mX + mX * q.mW + mY * q.mZ - mZ * q.mY,
                      mW * q.mY + mY * q.mW + mZ * q.mX - mX * q.mZ, mW * q.mZ + mZ * q.mW + mX * q.mY - mY * q.mX);
}

inline WbVector3 WbQuaternion::operator*(const WbVector3 &v) const {
  double twoX = 2.0 * mX, twoY = 2.0 * mY, twoZ = 2.0 * mZ;
  double sX = twoX * mX, sY = twoY * mY, sZ = twoZ * mZ, sW = 2.0 * mW * mW;
  double pXY = twoX * mY, pXZ = twoX * mZ, pXW = twoX * mW, pYZ = twoY * mZ, pYW = twoY * mW, pZW = twoZ * mW;
  double m00 = -1.0 + sX + sW;
  double m01 = pXY - pZW;
  double m02 = pXZ + pYW;
  double m10 = pXY + pZW;
  double m11 = -1.0 + sY + sW;
  double m12 = pYZ - pXW;
  double m20 = pXZ - pYW;
  double m21 = pYZ + pXW;
  double m22 = -1.0 + sZ + sW;
  return WbVector3(m00 * v.x() + m01 * v.y() + m02 * v.z(), m10 * v.x() + m11 * v.y() + m12 * v.z(),
                   m20 * v.x() + m21 * v.y() + m22 * v.z());
}

inline void WbQuaternion::normalize() {
  double d = mX * mX + mY * mY + mZ * mZ + mW * mW;
  if (d == 0.0) {
    mW = 1.0;
    return;
  }
  // see explanation at https://stackoverflow.com/questions/11667783/quaternion-and-normalization
  if (std::abs(1.0 - d) < 2.107342e-08)  // 2.107342e-08 magic number (> ULP/2 for IEEE doubles)
    d = 2.0 / (1.0 + d);                 // first order Padé approximant
  else
    d = 1.0 / sqrt(d);
  mW *= d;
  mX *= d;
  mY *= d;
  mZ *= d;
}

// spherical lerp between two quaternions
// sourced from here: http://www.technologicalutopia.com/sourcecode/xnageometry/quaternion.cs.htm
inline WbQuaternion WbQuaternion::slerp(const WbQuaternion &a, const WbQuaternion &b, double slerpAmount) {
  double alpha;
  double beta;
  double dotProduct = a.dot(b);
  bool flag = false;

  if (dotProduct < 0.0) {
    flag = true;
    dotProduct = -dotProduct;
  }

  if (dotProduct > 0.999999f) {
    beta = 1.0 - slerpAmount;
    alpha = flag ? -slerpAmount : slerpAmount;
  } else {
    double theta = WbMathsUtilities::clampedAcos(dotProduct);
    double cosecant = (1.0 / sin(theta));
    beta = (sin(((1.0 - slerpAmount) * theta))) * cosecant;
    alpha = flag ? ((-sin((slerpAmount * theta))) * cosecant) : ((sin((slerpAmount * theta))) * cosecant);
  }

  double newX = (beta * a.x()) + (alpha * b.x());
  double newY = (beta * a.y()) + (alpha * b.y());
  double newZ = (beta * a.z()) + (alpha * b.z());
  double newW = (beta * a.w()) + (alpha * b.w());

  return WbQuaternion(newW, newX, newY, newZ);
}

#endif
