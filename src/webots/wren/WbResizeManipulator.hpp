// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_NON_REGULAR_RESIZE_MANIPULATOR_HPP
#define WB_NON_REGULAR_RESIZE_MANIPULATOR_HPP

// Implemented class of resize manipulator for WbCone, WbIndexedFaceSet, WbElevation grid and all scale manipulators

#include "WbWrenAbstractResizeManipulator.hpp"

class WbRegularResizeManipulator : public WbWrenAbstractResizeManipulator {
  Q_OBJECT

public:
  explicit WbRegularResizeManipulator(int uniqueId,
                                      ResizeConstraint constraint = WbWrenAbstractResizeManipulator::NO_CONSTRAINT);
};

class WbPlaneResizeManipulator : public WbWrenAbstractResizeManipulator {
  Q_OBJECT

public:
  explicit WbPlaneResizeManipulator(int uniqueId);
};

// Scale manipulator for WbTransform
class WbScaleManipulator : public WbWrenAbstractResizeManipulator {
  Q_OBJECT

public:
  WbScaleManipulator(int uniqueId, ResizeConstraint constraint);
};

#endif  // WB_NON_REGULAR_RESIZE_MANIPULATOR_HPP
