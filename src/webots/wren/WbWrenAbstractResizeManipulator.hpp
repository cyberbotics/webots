// Copyright 1996-2022 Cyberbotics Ltd.
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

#ifndef WB_WREN_ABSTRACT_RESIZE_MANIPULATOR_HPP
#define WB_WREN_ABSTRACT_RESIZE_MANIPULATOR_HPP

#include "WbVector3.hpp"
#include "WbWrenAbstractManipulator.hpp"

#include <wren/material.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>

#include <vector>

class WbWrenAbstractResizeManipulator : public WbWrenAbstractManipulator {
  Q_OBJECT

public:
  enum ResizeConstraint { NO_CONSTRAINT, UNIFORM, X_EQUAL_Y };

  virtual ~WbWrenAbstractResizeManipulator();

  // Setters
  virtual void setResizeConstraint(ResizeConstraint constraint);

  ResizeConstraint resizeConstraint() { return mConstraint; }

  // Visibility
  void highlightAxis(int index) override;
  void showNormal() override;

  // Utility constants
  static const int STANDARD_COORDINATES[3];
  static const WbVector3 STANDARD_COORDINATE_VECTORS[3];

  // Others
  int coordinate(int handleNumber) const override { return STANDARD_COORDINATES[handleNumber]; }
  int coordinateToHandleNumber(int coord) override { return coord; };
  const WbVector3 &coordinateVector(int handleNumber) const override { return STANDARD_COORDINATE_VECTORS[handleNumber]; }
  WbVector3 relativeHandlePosition(int handleNumber) const override {
    return mScale * STANDARD_COORDINATE_VECTORS[coordinate(handleNumber)];
  }
  void updateHandleDimensions(const float scaleFactor, const float viewDistanceScale);

protected:
  explicit WbWrenAbstractResizeManipulator(ResizeConstraint constraint);

  enum { X, Y, Z };

  ResizeConstraint mConstraint;
  int mUniformMaterialIndex;

  std::vector<WrStaticMesh *> mMeshes;
  std::vector<WrRenderable *> mRenderables;
  WrMaterial *mHandleMaterials[3];
  WrMaterial *mAxisMaterials[3];
  WrTransform *mHandleTransforms[3];
  WrTransform *mAxisTransforms[3];

  void initializeHandlesEntities(bool resize = true);

private:
  WbWrenAbstractResizeManipulator(const WbWrenAbstractResizeManipulator &original);
  WbWrenAbstractResizeManipulator &operator=(const WbWrenAbstractResizeManipulator &original);

  void updateHandlesMaterial();
};

#endif  // WB_WREN_ABSTRACT_RESIZE_MANIPULATOR_HPP
