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

#include "WbAffinePlane.hpp"

void WbAffinePlane::from3Points(const WbVector3 &P, const WbVector3 &Q, const WbVector3 &R) {
  const double u[3] = {Q.x() - P.x(), Q.y() - P.y(), Q.z() - P.z()};
  const double v[3] = {R.x() - P.x(), R.y() - P.y(), R.z() - P.z()};
  // Compute the cross product of u with v
  mA = u[1] * v[2] - u[2] * v[1];
  mB = u[2] * v[0] - u[0] * v[2];
  mC = u[0] * v[1] - u[1] * v[0];
  // Compute the scalar product of u cross v with OP
  mD = mA * P.x() + mB * P.y() + mC * P.z();
  normalize();
}
