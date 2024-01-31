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

#include "WbRotation.hpp"

#include "WbMathsUtilities.hpp"

#include <cassert>

void WbRotation::fromQuaternion(const WbQuaternion &q) {
  // ensure that the quaternion is normalized as it should be
  assert(std::abs(sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w()) - 1) < 0.0000000001);
  // if q.w() is slightly greater than 1 or slightly lower than -1, acos(q.w()) will return nan
  // unfortunately, due to floating point rounding, that happens even if the quaternion was just normalized
  if (q.w() >= 1.0)
    mAngle = 0.0;
  else if (q.w() <= -1.0)
    mAngle = 2.0 * M_PI;
  else
    mAngle = 2.0 * acos(q.w());
  if (mAngle < WbPrecision::DOUBLE_EQUALITY_TOLERANCE) {
    // if the angle is close to zero, then the direction of the axis is not important
    mX = 0.0;
    mY = 0.0;
    mZ = 1.0;
    mAngle = 0.0;
    return;
  }

  // normalise axes
  const double inv = 1.0 / sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  mX = q.x() * inv;
  mY = q.y() * inv;
  mZ = q.z() * inv;
}

void WbRotation::fromMatrix3(const WbMatrix3 &M) {
  // Reference: https://www.geometrictools.com/Documentation/RotationRepresentations.pdf
  const double theta = WbMathsUtilities::clampedAcos((M(0, 0) + M(1, 1) + M(2, 2) - 1) / 2);
  if (theta < WbPrecision::DOUBLE_EQUALITY_TOLERANCE) {  // If `theta == 0`
    mX = 1;
    mY = 0;
    mZ = 0;
    mAngle = 0;
    return;
  } else if (M_PI - theta < WbPrecision::DOUBLE_EQUALITY_TOLERANCE) {  // If `theta == pi`
    if (M(0, 0) > M(1, 1) && M(0, 0) > M(2, 2)) {
      mX = sqrt(M(0, 0) - M(1, 1) - M(2, 2) + 1) / 2;
      mY = M(0, 1) / (2 * mX);
      mZ = M(0, 2) / (2 * mX);
    } else if (M(1, 1) > M(0, 0) && M(1, 1) > M(2, 2)) {
      mY = sqrt(M(1, 1) - M(0, 0) - M(2, 2) + 1) / 2;
      mX = M(0, 1) / (2 * mY);
      mZ = M(1, 2) / (2 * mY);
    } else {
      mZ = sqrt(M(2, 2) - M(0, 0) - M(1, 1) + 1) / 2;
      mX = M(0, 2) / (2 * mZ);
      mY = M(1, 2) / (2 * mZ);
    }
  } else {  // If `theta in (0, pi)`
    mX = M(2, 1) - M(1, 2);
    mY = M(0, 2) - M(2, 0);
    mZ = M(1, 0) - M(0, 1);
  }
  mAngle = theta;
  normalizeAxis();
}

void WbRotation::fromBasisVectors(const WbVector3 &vx, const WbVector3 &vy, const WbVector3 &vz) {
  const double cosAngle = 0.5 * (vx.x() + vy.y() + vz.z() - 1.0);
  if (fabs(cosAngle) > 1.0) {
    // exception
    mX = 1.0;
    mY = 0.0;
    mZ = 0.0;
    mAngle = 0.0;
  } else {
    mX = vy.z() - vz.y();
    mY = vz.x() - vx.z();
    mZ = vx.y() - vy.x();
    mAngle = acos(cosAngle);
  }
}

void WbRotation::toFloatArray(float *rotation) const {
  rotation[0] = static_cast<float>(mAngle);
  rotation[1] = static_cast<float>(mX);
  rotation[2] = static_cast<float>(mY);
  rotation[3] = static_cast<float>(mZ);
}

WbVector3 WbRotation::direction() const {
  const double c = cos(mAngle), s = sin(mAngle), t = 1 - c;
  const double tTimesX = t * mX;
  return WbVector3(tTimesX * mX + c, tTimesX * mY + s * mZ, tTimesX * mZ - s * mY);
}

WbVector3 WbRotation::right() const {
  const double c = cos(mAngle), s = sin(mAngle), t = 1 - c;
  const double tTimesY = t * mY;
  return WbVector3(tTimesY * mX - s * mZ, tTimesY * mY + c, tTimesY * mZ + s * mX);
}

WbVector3 WbRotation::up() const {
  const double c = cos(mAngle), s = sin(mAngle), t = 1 - c;
  const double tTimesZ = t * mZ;
  return WbVector3(tTimesZ * mX + s * mY, tTimesZ * mY - s * mX, tTimesZ * mZ + c);
}
