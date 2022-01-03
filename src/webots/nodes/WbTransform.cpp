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

#include "WbTransform.hpp"
#include "WbBoundingSphere.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSimulationState.hpp"
#include "WbSolid.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/node.h>
#include <wren/transform.h>

void WbTransform::init() {
  mScale = findSFVector3("scale");
}

WbTransform::WbTransform(WbTokenizer *tokenizer) : WbPose("Transform", tokenizer) {
  init();
}

WbTransform::WbTransform(const WbTransform &other) : WbPose(other) {
  init();
}

WbTransform::WbTransform(const WbNode &other) : WbPose(other) {
  init();
}

bool WbTransform::checkScalePositivity(WbVector3 &correctedScale) const {
  const WbVector3 &s = mScale->value();
  const double x = s.x();
  const double y = s.y();
  const double z = s.z();
  correctedScale.setXyz(x, y, z);
  bool b = false;

  if (x <= 0.0) {
    if (x == 0.0) {
      correctedScale.setX(1.0);
      mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be positive: x is set to 1.0."));
    } else {
      correctedScale.setX(fabs(x));
      mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be positive: x is set to abs(x)."));
    }
    b = true;
  }

  if (y <= 0.0) {
    if (y == 0.0) {
      correctedScale.setY(1.0);
      mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be positive: y is set to 1.0."));
    } else {
      correctedScale.setY(fabs(y));
      mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be positive: y is set to abs(y)."));
    }
    b = true;
  }

  if (z <= 0.0) {
    if (z == 0.0) {
      correctedScale.setZ(1.0);
      mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be positive: z is set to 1.0."));
    } else {
      correctedScale.setZ(fabs(z));
      mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be positive: z is set to abs(z)."));
    }
    b = true;
  }

  return b;
}

bool WbTransform::checkScalingPhysicsConstraints(WbVector3 &correctedScale, int constraintType, bool warning) const {
  bool b = false;
  if (constraintType == WbWrenAbstractResizeManipulator::UNIFORM)
    b = checkScaleUniformity(correctedScale);
  else if (constraintType == WbWrenAbstractResizeManipulator::X_EQUAL_Z && mScale->x() != mScale->z()) {
    if (mPreviousXscaleValue == mScale->x())
      correctedScale.setX(mScale->z());
    else
      correctedScale.setZ(mScale->x());
    b = true;
    if (warning)
      mBaseNode->parsingWarn(
        QObject::tr("'scale' were changed so that x = z because of physics constraints inside a 'boundingObject'."));
  }

  return b;
}

bool WbTransform::checkScaleUniformity(WbVector3 &correctedScale, bool warning) const {
  const double x = correctedScale.x();
  const double y = correctedScale.y();
  const double z = correctedScale.z();
  bool b = false;

  if (x != y) {
    if (x == z)
      correctedScale.setXyz(y, y, y);
    else
      correctedScale.setXyz(x, x, x);
    b = true;
  } else if (y != z) {
    correctedScale.setXyz(z, z, z);
    b = true;
  }

  if (b && warning)
    mBaseNode->parsingWarn(QObject::tr("'scale' was made uniform because of physics constraints inside a 'boundingObject'."));

  return b;
}

bool WbTransform::checkScaleUniformity(bool warning) {
  WbVector3 correctedScale;

  if (checkScaleUniformity(correctedScale, warning)) {
    mScale->setValue(correctedScale);
    return true;
  }

  return false;
}

bool WbTransform::checkScale(int constraintType, bool warning) {
  WbVector3 correctedScale;
  bool b = false;

  if (checkScalePositivity(correctedScale))
    b = true;

  if (constraintType > 0 && checkScalingPhysicsConstraints(correctedScale, constraintType, warning))
    b = true;

  if (!mScale->value().almostEquals(WbVector3(1, 1, 1)) &&
      WbNodeUtilities::hasARobotDescendant(dynamic_cast<const WbNode *>(this))) {
    correctedScale.setXyz(1, 1, 1);
    b = true;
    if (warning)
      mBaseNode->parsingWarn(QObject::tr("'scale' cannot be changed if a descendant Robot node is present."));
  }

  if (b)
    mScale->setValue(correctedScale);

  mPreviousXscaleValue = mScale->x();

  return b;
}

void WbTransform::applyToScale() {
  mBaseNode->setMatrixNeedUpdate();
  mBaseNode->setScaleNeedUpdate();

  if (mBaseNode->areWrenObjectsInitialized())
    applyScaleToWren();

  if (mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject() && WbSimulationState::instance()->isRayTracingEnabled())
    mBaseNode->boundingSphere()->setOwnerSizeChanged();

  if (mScaleManipulator && mScaleManipulator->isAttached())
    setResizeManipulatorDimensions();

  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    updateTranslateRotateHandlesSize();
}

void WbTransform::updateScale(bool warning) {
  const int constraint = constraintType();
  if (checkScale(constraint, warning))
    return;

  applyToScale();
}

void WbTransform::createScaleManipulator() {
  const int constraint = constraintType();
  mScaleManipulator = new WbScaleManipulator(mBaseNode->uniqueId(), (WbScaleManipulator::ResizeConstraint)constraint);
}

void WbTransform::createScaleManipulatorIfNeeded() {
  if (!mScaleManipulatorInitialized) {
    assert(hasResizeManipulator());  // otherwise the show resize manipulator option should be disabled
    mScaleManipulatorInitialized = true;
    createScaleManipulator();
    if (mScaleManipulator)
      mScaleManipulator->attachTo(baseNode()->wrenNode());
  }
}