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

#ifndef WB_PHYSICS_VECTOR_REPRESENTATION_HPP
#define WB_PHYSICS_VECTOR_REPRESENTATION_HPP

//
// Description: class handling the rendering of a force or a torque dragged by the mouse
//

#include <wren/material.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <QtCore/QObject>

// Abstract class //
////////////////////

class WbRotation;
class WbVector3;

class WbPhysicsVectorRepresentation : public QObject {
  Q_OBJECT
public:
  virtual ~WbPhysicsVectorRepresentation();

  void initializeTailAndArrow(const float *materialColor);

  void updatePosition(const WbVector3 &begin, const WbVector3 &end, const WbRotation &orientation);

  void setScale(float scale);

protected:
  WbPhysicsVectorRepresentation() :
    mMaterial(NULL),
    mTailRenderable(NULL),
    mArrowRenderable(NULL),
    mTailMesh(NULL),
    mArrowMesh(NULL),
    mHeadTransform(NULL),
    mTailTransform(NULL){};

  WrMaterial *mMaterial;

  WrRenderable *mTailRenderable;
  WrRenderable *mArrowRenderable;

  WrStaticMesh *mTailMesh;
  WrStaticMesh *mArrowMesh;

  WrTransform *mHeadTransform;
  WrTransform *mTailTransform;
};

// Implemented classes

class WbForceRepresentation : public WbPhysicsVectorRepresentation {
  Q_OBJECT
public:
  WbForceRepresentation();
};

class WbTorqueRepresentation : public WbPhysicsVectorRepresentation {
  Q_OBJECT
public:
  WbTorqueRepresentation();
  virtual ~WbTorqueRepresentation();

private:
  void initializeSpinSymbol();

  WrMaterial *mCoilMaterial;

  WrStaticMesh *mCoilMesh;
  WrRenderable *mCoilRenderable;

  WrRenderable *mCoilArrowRenderable;
  WrTransform *mCoilArrowTransform;
};

#endif
