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

#ifndef WB_AXIS_ANGLE_HPP
#define WB_AXIS_ANGLE_HPP

#include "WbVector3.hpp"

//
// Description: Axis-angle representation
//

class WbAxisAngle {
public:
  WbAxisAngle(double x, double y, double z, double angle) : mAxis(WbVector3(x, y, z)), mAngle(angle){};

  WbVector3 &axis() { return mAxis; };

  double angle() const { return mAngle; }

  // text conversion
  QString toString(WbPrecision::Level level = WbPrecision::Level::DOUBLE_MAX) const {
    return QString("%1 %2 %3 %4")
      .arg(WbPrecision::doubleToString(mAxis.x(), level))
      .arg(WbPrecision::doubleToString(mAxis.y(), level))
      .arg(WbPrecision::doubleToString(mAxis.z(), level))
      .arg(WbPrecision::doubleToString(mAngle, level));
  }

private:
  WbVector3 mAxis;
  double mAngle;
};

#endif
