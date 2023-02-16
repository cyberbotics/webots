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

#include "WbMatrix3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbQuaternion.hpp"
#include "WbRotation.hpp"

static const double IDENTITY[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

void WbMatrix3::setIdentity() {
  memcpy(mM, IDENTITY, sizeof(mM));
}

void WbMatrix3::fromAxisAngle(const WbRotation &r) {
  fromAxisAngle(r.x(), r.y(), r.z(), r.angle());
}

void WbMatrix3::fromQuaternion(const WbQuaternion &q) {
  double twoQx = 2.0 * q.x(), twoQy = 2.0 * q.y(), twoQz = 2.0 * q.z(), twoQw = 2.0 * q.w();
  double sX = twoQx * q.x(), sY = twoQy * q.y(), sZ = twoQz * q.z(), sW = twoQw * q.w();
  double pXY = twoQx * q.y(), pXZ = twoQx * q.z(), pXW = twoQx * q.w();
  double pYZ = twoQy * q.z(), pYW = twoQy * q.w();
  double pZW = twoQz * q.w();
  mM[0] = -1.0 + sX + sW;
  mM[1] = pXY - pZW;
  mM[2] = pXZ + pYW;
  mM[3] = pXY + pZW;
  mM[4] = -1.0 + sY + sW;
  mM[5] = pYZ - pXW;
  mM[6] = pXZ - pYW;
  mM[7] = pXW + pYZ;
  mM[8] = -1.0 + sZ + sW;
}

WbQuaternion WbMatrix3::toQuaternion() const {
  if (mM[0] == mM[4] && mM[0] == mM[8] && mM[0] == 1.0) {
    // exception
    return WbQuaternion();
  }

  double s = 2.0;
  double invS = 1.0;
  const double trace = mM[0] + mM[4] + mM[8];
  if (trace >= 0.0) {
    s *= sqrt(trace + 1);  // we divide by s = 4w, which is large enough
    invS = 1.0 / s;
    return WbQuaternion(0.25 * s, (mM[7] - mM[5]) * invS, (mM[2] - mM[6]) * invS, (mM[3] - mM[1]) * invS);
  }
  if (mM[0] > mM[4] && mM[0] > mM[8]) {      // M(0) is larger than max(M(4), M(8))
    s *= sqrt(1.0 + mM[0] - mM[4] - mM[8]);  // s = 4x
    invS = 1.0 / s;
    return WbQuaternion((mM[7] - mM[5]) * invS, 0.25 * s, (mM[1] + mM[3]) * invS, (mM[6] + mM[2]) * invS);
  }
  if (mM[4] > mM[8]) {                       // M(4) is the largest
    s *= sqrt(1.0 + mM[4] - mM[8] - mM[0]);  // s = 4y
    invS = 1.0 / s;
    return WbQuaternion((mM[2] - mM[6]) * invS, (mM[1] + mM[3]) * invS, 0.25 * s, (mM[5] + mM[7]) * invS);
  }
  // M(8) is the largest
  s *= sqrt(1.0 + mM[8] - mM[0] - mM[4]);  // s = 4z
  invS = 1.0 / s;
  return WbQuaternion((mM[3] - mM[1]) * invS, (mM[6] + mM[2]) * invS, (mM[5] + mM[7]) * invS, 0.25 * s);
}

// Reference: https://www.geometrictools.com/Documentation/EulerAngles.pdf
WbVector3 WbMatrix3::toEulerAnglesZYX() const {
  WbVector3 angles;

  if (mM[6] < 1) {
    if (mM[6] > -1) {
      angles.setX(atan2(mM[7], mM[8]));
      angles.setY(WbMathsUtilities::clampedAsin(-mM[6]));
      angles.setZ(atan2(mM[3], mM[0]));
    } else {
      angles.setX(0);
      angles.setY(-M_PI / 2);
      angles.setZ(atan2(-mM[5], mM[4]));
    }
  } else {
    angles.setX(0);
    angles.setY(-M_PI / 2);
    angles.setZ(atan2(-mM[5], mM[4]));
  }
  return angles;
}

WbAxisAngle WbMatrix3::toAxisAngle() const {
  // Reference: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm

  const double sqrt2Half = sqrt(2) / 2;
  const double epsilon = 0.01;
  const double epsilon2 = 0.1;

  double angle, x, y, z;
  if ((abs(mM[1] - mM[3]) < epsilon) && (abs(mM[2] - mM[6]) < epsilon) && (abs(mM[5] - mM[7]) < epsilon)) {
    if ((abs(mM[1] + mM[3]) < epsilon2) && (abs(mM[2] + mM[6]) < epsilon2) && (abs(mM[5] + mM[7]) < epsilon2) &&
        (abs(mM[0] + mM[4] + mM[8] - 3) < epsilon2)) {
      return WbAxisAngle(1, 0, 0, 0);
    }
    angle = M_PI;
    double xx = (mM[0] + 1) / 2;
    double yy = (mM[4] + 1) / 2;
    double zz = (mM[8] + 1) / 2;
    double xy = (mM[1] + mM[3]) / 4;
    double xz = (mM[2] + mM[6]) / 4;
    double yz = (mM[5] + mM[7]) / 4;
    if ((xx > yy) && (xx > zz)) {
      if (xx < epsilon) {
        x = 0;
        y = sqrt2Half;
        z = sqrt2Half;
      } else {
        x = sqrt(xx);
        y = xy / x;
        z = xz / x;
      }
    } else if (yy > zz) {
      if (yy < epsilon) {
        x = sqrt2Half;
        y = 0;
        z = sqrt2Half;
      } else {
        y = sqrt(yy);
        x = xy / y;
        z = yz / y;
      }
    } else {
      if (zz < epsilon) {
        x = sqrt2Half;
        y = sqrt2Half;
        z = 0;
      } else {
        z = sqrt(zz);
        x = xz / z;
        y = yz / z;
      }
    }
    return WbAxisAngle(x, y, z, angle);
  }
  double s = sqrt((mM[7] - mM[5]) * (mM[7] - mM[5]) + (mM[2] - mM[6]) * (mM[2] - mM[6]) + (mM[3] - mM[1]) * (mM[3] - mM[1]));
  if (abs(s) < 0.001)
    s = 1;
  angle = WbMathsUtilities::clampedAcos((mM[0] + mM[4] + mM[8] - 1) / 2);
  x = (mM[7] - mM[5]) / s;
  y = (mM[2] - mM[6]) / s;
  z = (mM[3] - mM[1]) / s;
  return WbAxisAngle(x, y, z, angle);
}
