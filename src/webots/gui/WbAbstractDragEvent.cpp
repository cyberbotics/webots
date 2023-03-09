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

#include "WbAbstractDragEvent.hpp"

#include "WbVector2.hpp"
#include "WbVector3.hpp"
#include "WbWrenLabelOverlay.hpp"

// Main abstract class //
/////////////////////////
const float WbDragEvent::cFloatMax = std::numeric_limits<float>::max();

WbDragEvent::WbDragEvent() : QObject() {
}

bool WbDragEvent::exceedsFloatMax(const WbVector3 &v) {
  return (fabs(v.x()) >= cFloatMax || fabs(v.y()) >= cFloatMax || fabs(v.z()) >= cFloatMax);
}

bool WbDragEvent::exceedsFloatMax(double x) {
  return fabs(x) >= cFloatMax;
}

bool WbDragEvent::exceedsFloatMax(float x) {
  return fabs(x) >= cFloatMax;
}

//
//  View 3D drag event classes (dragging the mouse modifies the content of the 3D view and requires WbViewpoint)
//

// Abstract classes //
//////////////////////

// WbDragView3DEvent constructor
WbDragView3DEvent::WbDragView3DEvent(WbViewpoint *viewpoint) :
  WbDragEvent(),
  mViewpoint(viewpoint),
  mViewDistanceUnscaling(1.0f) {
}

WbVector2 WbDragView3DEvent::clampLabelPosition(const float x, const float y, const WbWrenLabelOverlay *overlay) {
  WbVector2 clamped(x, y);
  const float labelWidth = overlay->width();
  const float labelHeight = overlay->height();

  if (x < 0.0f)
    clamped[0] = 0.0f;
  else if (x > 1.0f - labelWidth)
    clamped[0] = 1.0f - labelWidth;

  if (y < 0.0f)
    clamped[1] = 0.0f;
  else if (y > 1.0f - labelHeight)
    clamped[1] = 1.0f - labelHeight;

  return clamped;
}

// Kinematics drag events

// WbDragKinematicsEvent constructor
WbDragKinematicsEvent::WbDragKinematicsEvent(WbViewpoint *viewpoint) : WbDragView3DEvent(viewpoint) {
}
