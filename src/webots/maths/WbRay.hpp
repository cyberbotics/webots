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

#ifndef WB_RAY_HPP
#define WB_RAY_HPP

//
// Description: data structure for a 3D ray
//

#include <cmath>
#include "WbVector3.hpp"

class WbAffinePlane;
class WbMatrix4;

class WbRay {
public:
  // construct default ray
  WbRay() : mOrigin(0.0, 0.0, 0.0), mDirection(0.0, 0.0, 0.0) {}
  WbRay(const WbRay &other) : mOrigin(other.mOrigin), mDirection(other.mDirection) {}
  // construct from two 3D vectors
  WbRay(const WbVector3 &origin, const WbVector3 &direction) : mOrigin(origin), mDirection(direction) {}

  // getters
  const WbVector3 &origin() const { return mOrigin; }
  const WbVector3 &direction() const { return mDirection; }
  WbVector3 point(double t) const {
    return WbVector3(t * mDirection.x() + mOrigin.x(), t * mDirection.y() + mOrigin.y(), t * mDirection.z() + mOrigin.z());
  }

  // setters
  void setOrigin(const WbVector3 &origin) { mOrigin.setXyz(origin.x(), origin.y(), origin.z()); }
  void setOrigin(const double origin[3]) { mOrigin.setXyz(origin[0], origin[1], origin[2]); }
  void setOrigin(double x, double y, double z) { mOrigin.setXyz(x, y, z); }
  void setDirection(const WbVector3 &direction) { mDirection.setXyz(direction.x(), direction.y(), direction.z()); }
  void setDirection(const double direction[3]) { mDirection.setXyz(direction[0], direction[1], direction[2]); }
  void setDirection(double x, double y, double z) { mDirection.setXyz(x, y, z); }
  //

  void redefine(const WbVector3 &origin, const WbVector3 &direction) {
    mOrigin.setXyz(origin.x(), origin.y(), origin.z());
    mDirection.setXyz(direction.x(), direction.y(), direction.z());
  }

  // assignment: R1 = R2
  WbRay &operator=(const WbRay &r) {
    mOrigin = r.mOrigin;
    mDirection = r.mDirection;
    return *this;
  }

  // normalize
  void normalize();

  // transform
  WbRay transformed(const WbMatrix4 &matrix) const;

  // intersection methods
  std::pair<bool, double> intersects(const WbAffinePlane &p, bool testCull = false) const;
  std::pair<bool, double> intersects(const WbVector3 &vert0, const WbVector3 &vert1, const WbVector3 &vert2, bool testCull,
                                     double &u, double &v) const;
  std::pair<bool, double> intersects(const WbVector3 &center, double radius,
                                     bool testCull = false) const;  // intersection with a sphere
  std::pair<bool, double> intersects(const WbVector3 &minBound, const WbVector3 &maxBound, double &tMin,
                                     double &tMax) const;  // intersection with a box

private:
  WbVector3 mOrigin;
  WbVector3 mDirection;
};

inline void WbRay::normalize() {
  double d = mDirection.x() * mDirection.x() + mDirection.y() * mDirection.y() + mDirection.z() * mDirection.z();
  if (d == 0.0) {
    mDirection.setXyz(1.0, 0.0, 0.0);
  }
  d = 1.0 / sqrt(d);
  mDirection.setXyz(mDirection.x() * d, mDirection.y() * d, mDirection.z() * d);
}

#endif
