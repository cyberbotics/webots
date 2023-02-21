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

#include "WbPolygon.hpp"

WbPolygon::WbPolygon() : QVarLengthArray<WbVector2, 32>(), mSize(0) {
  resize(3);
}

WbPolygon::~WbPolygon() {
}

bool WbPolygon::contains(double x, double y) const {
  // The vertices of the polygon are ordered counter clockwise, except if there are only 3 vertices
  if (mSize == 1)
    return (x == value(0).x() && y == value(0).y());

  double dx, dy, sideX, sideY, det;

  if (mSize == 2) {
    dx = x - value(0).x();
    dy = y - value(0).y();
    sideX = value(1).x() - value(0).x();
    sideY = value(1).y() - value(0).y();
    det = sideX * dy - sideY * dx;
    return (det == 0.0 && dx * sideX + dy * sideY < 0.0);
  }

  if (mSize == 3) {  // In this case, the vertex order is possibly clockwise
    // Side A_0 A_1 of the polygon
    sideX = value(1).x() - value(0).x();
    sideY = value(1).y() - value(0).y();
    dx = x - value(0).x();
    dy = y - value(0).y();
    const double det1 = sideX * dy - sideY * dx;

    // Side A_1 A_2 of the polygon
    sideX = value(2).x() - value(1).x();
    sideY = value(2).y() - value(1).y();
    dx = x - value(1).x();
    dy = y - value(1).y();
    const double det2 = sideX * dy - sideY * dx;

    if ((det1 < 0.0) != (det2 < 0.0))
      return false;

    // Side A_2 A_0 of the polygon
    sideX = value(0).x() - value(2).x();
    sideY = value(0).y() - value(2).y();
    dx = x - value(2).x();
    dy = y - value(2).y();
    const double det3 = sideX * dy - sideY * dx;

    if ((det2 < 0.0) != (det3 < 0.0))
      return false;

    return true;
  }

  // General case (size > 3)
  const int sizeMinusOne = mSize - 1;
  for (int i = 0; i < sizeMinusOne; i++) {
    // Side A_i A_{i + 1} of the polygon
    sideX = value(i + 1).x() - value(i).x();
    sideY = value(i + 1).y() - value(i).y();
    dx = x - value(i).x();
    dy = y - value(i).y();
    det = sideX * dy - sideY * dx;
    if (det < 0.0)
      return false;
  }

  // Side A_n A_0 of the polygon
  sideX = value(0).x() - value(sizeMinusOne).x();
  sideY = value(0).y() - value(sizeMinusOne).y();
  dx = x - value(sizeMinusOne).x();
  dy = y - value(sizeMinusOne).y();
  det = sideX * dy - sideY * dx;
  if (det < 0.0)
    return false;

  return true;
}

bool WbPolygon::contains(const WbVector2 &point) const {
  return contains(point.x(), point.y());
}
