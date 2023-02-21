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

#ifndef WB_TRANSLATE_ROTATE_MANIPULATOR_HPP
#define WB_TRANSLATE_ROTATE_MANIPULATOR_HPP

//
// Description: class implementing the object translate manipulator
//              objects have to be created
//
#include "WbRotation.hpp"
#include "WbVector3.hpp"
#include "WbWrenAbstractManipulator.hpp"

#include <wren/material.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <vector>

class WbTranslateRotateManipulator : public WbWrenAbstractManipulator {
  Q_OBJECT

public:
  WbTranslateRotateManipulator(bool isTranslationAvailable, bool isRotationAvailable);
  virtual ~WbTranslateRotateManipulator();

  // Setters
  void initializeHandlesEntities();
  void showRotationLine(bool show);

  // Visibility
  void highlightAxis(int index) override;
  void showNormal() override;

  void adjustHandleOrientation(int handleNumber);

  // Others
  void updateRotationLine(const WbVector3 &begin, const WbVector3 &end, const WbRotation &orientation, float arrowScale);
  int coordinate(int handleNumber) const override { return handleNumber % 3; }
  int coordinateToHandleNumber(int coord) override { return coord; };
  const WbVector3 &coordinateVector(int handleNumber) const override { return STANDARD_COORDINATE_VECTORS[handleNumber % 3]; }
  WbVector3 relativeHandlePosition(int handleNumber) const override;

private:
  // Utility constants
  static const WbVector3 STANDARD_COORDINATE_VECTORS[3];

  bool mHasRotationHandles;
  bool mHasTranslationHandles;

  std::vector<WrStaticMesh *> mMeshes;
  std::vector<WrRenderable *> mRenderables;

  WrMaterial *mHandlesMaterials[3][2];
  WrMaterial *mActiveRotationHandleMaterial;

  WrTransform *mTranslationHandlesTransforms[3];
  WrTransform *mRotationHandlesTransforms[3];
  WrTransform *mInfiniteAxesTransforms[3];
  WrTransform *mRotationLineTransform;
  WrTransform *mRotationDoubleArrowTransform;
  WrTransform *mAxesTransform;
};

#endif  // WB_TRANSLATE_MANIPULATOR_HPP
