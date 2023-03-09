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

#ifndef WB_POLYGON_HPP
#define WB_POLYGON_HPP

//
// Description: 2D Polygon
//

#include <QtCore/QVarLengthArray>
#include "WbVector2.hpp"

class WbPolygon : public QVarLengthArray<WbVector2, 32> {
public:
  WbPolygon();
  virtual ~WbPolygon();
  // Accessor
  int actualSize() const {
    // mSize is used to avoid reallocation memory when the number of vertices decreases (we always have: actualSize() <= size())
    return mSize;
  }
  void setActualSize(int size) { mSize = size; }
  bool contains(const WbVector2 &point) const;
  bool contains(double x, double y) const;
  void reserveVertices(int size);

private:
  int mSize;  // number of vertices
};

#endif  // WB_POLYGON_HPP
