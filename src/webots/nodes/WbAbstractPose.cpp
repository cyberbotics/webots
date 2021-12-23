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

#include "WbAbstractPose.hpp"

#include "WbBaseNode.hpp"
#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbMatter.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPose.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSimulationState.hpp"
#include "WbTranslateRotateManipulator.hpp"

#include <wren/transform.h>

#include <QtCore/QObject>

void WbAbstractPose::init(WbBaseNode *node) {
  mBaseNode = node;
  assert(mBaseNode);

  mTranslation = node->findSFVector3("translation");
  mRotation = node->findSFRotation("rotation");
  mScale = node->findSFVector3("scale");
  mTranslationStep = node->findSFDouble("translationStep");
  mRotationStep = node->findSFDouble("rotationStep");

  mMatrix = NULL;
  mMatrixNeedUpdate = true;
  mVrmlMatrixNeedUpdate = true;
  mAbsoluteScaleNeedUpdate = true;
  mPreviousXscaleValue = 1.0;
  mIsTopTransform = false;
  mHasSearchedTopTransform = false;
  mScaleManipulator = NULL;
  mTranslateRotateManipulator = NULL;

  mTranslateRotateManipulatorInitialized = false;
  mScaleManipulatorInitialized = false;

  mIsTranslationFieldVisible = true;
  mIsRotationFieldVisible = true;
  mIsTranslationFieldVisibleReady = false;
  mIsRotationFieldVisibleReady = false;
  mCanBeTranslated = false;
  mCanBeRotated = false;
}

WbAbstractPose::~WbAbstractPose() {
  deleteWrenObjects();
  delete mMatrix;
}

void WbAbstractPose::deleteWrenObjects() {
  delete mScaleManipulator;
  mScaleManipulator = NULL;

  delete mTranslateRotateManipulator;
  mTranslateRotateManipulator = NULL;
  mTranslateRotateManipulatorInitialized = false;
}

void WbAbstractPose::updateTranslation() {
  mBaseNode->setMatrixNeedUpdate();

  if (mBaseNode->areWrenObjectsInitialized())
    applyTranslationToWren();

  if (WbSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();
}

void WbAbstractPose::updateRotation() {
  mBaseNode->setMatrixNeedUpdate();

  if (mBaseNode->areWrenObjectsInitialized())
    applyRotationToWren();

  if (WbSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();

  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    updateTranslateRotateHandlesSize();
}

void WbAbstractPose::updateTranslationAndRotation() {
  mBaseNode->setMatrixNeedUpdate();

  if (mBaseNode->areWrenObjectsInitialized())
    applyTranslationAndRotationToWren();

  if (WbSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();

  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    updateTranslateRotateHandlesSize();
}

bool WbAbstractPose::checkScalePositivity(WbVector3 &correctedScale) const {
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

bool WbAbstractPose::checkScalingPhysicsConstraints(WbVector3 &correctedScale, int constraintType, bool warning) const {
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

bool WbAbstractPose::checkScaleUniformity(WbVector3 &correctedScale, bool warning) const {
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

bool WbAbstractPose::checkScaleUniformity(bool warning) {
  WbVector3 correctedScale;

  if (checkScaleUniformity(correctedScale, warning)) {
    mScale->setValue(correctedScale);
    return true;
  }

  return false;
}

bool WbAbstractPose::checkScale(int constraintType, bool warning) {
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

void WbAbstractPose::applyToScale() {
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

void WbAbstractPose::updateScale(bool warning) {
  const int constraint = constraintType();
  if (checkScale(constraint, warning))
    return;

  applyToScale();
}

void WbAbstractPose::updateTranslationFieldVisibility() const {
  if (mIsTranslationFieldVisibleReady)
    return;
  mIsTranslationFieldVisible = WbNodeUtilities::isVisible(mBaseNode->findField("translation", true));
  mCanBeTranslated =
    !WbNodeUtilities::isTemplateRegeneratorField(mBaseNode->findField("translation", true)) && mIsTranslationFieldVisible;
  mIsTranslationFieldVisibleReady = true;
}

void WbAbstractPose::updateRotationFieldVisibility() const {
  if (mIsRotationFieldVisibleReady)
    return;
  mIsRotationFieldVisible = WbNodeUtilities::isVisible(mBaseNode->findField("rotation", true));
  mCanBeRotated =
    !WbNodeUtilities::isTemplateRegeneratorField(mBaseNode->findField("rotation", true)) && mIsRotationFieldVisible;
  mIsRotationFieldVisibleReady = true;
}

bool WbAbstractPose::isTranslationFieldVisible() const {
  updateTranslationFieldVisibility();
  return mIsTranslationFieldVisible;
}

bool WbAbstractPose::isRotationFieldVisible() const {
  updateRotationFieldVisibility();
  return mIsRotationFieldVisible;
}

bool WbAbstractPose::canBeTranslated() const {
  updateTranslationFieldVisibility();
  return mCanBeTranslated;
}

bool WbAbstractPose::canBeRotated() const {
  updateRotationFieldVisibility();
  return mCanBeRotated;
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void WbAbstractPose::createScaleManipulator() {
  const int constraint = constraintType();
  mScaleManipulator = new WbScaleManipulator(mBaseNode->uniqueId(), (WbScaleManipulator::ResizeConstraint)constraint);
}

void WbAbstractPose::createScaleManipulatorIfNeeded() {
  if (!mScaleManipulatorInitialized) {
    assert(hasResizeManipulator());  // otherwise the show resize manipulator option should be disabled
    mScaleManipulatorInitialized = true;
    createScaleManipulator();
    if (mScaleManipulator)
      mScaleManipulator->attachTo(baseNode()->wrenNode());
  }
}

void WbAbstractPose::createTranslateRotateManipulatorIfNeeded() {
  if (!mTranslateRotateManipulatorInitialized) {
    mTranslateRotateManipulatorInitialized = true;
    if (!canBeTranslated() && !canBeRotated())
      return;
    mTranslateRotateManipulator = new WbTranslateRotateManipulator(canBeTranslated(), canBeRotated());
    if (mTranslateRotateManipulator) {
      mTranslateRotateManipulator->attachTo(baseNode()->wrenNode());
      updateTranslateRotateHandlesSize();
    }
  }
}

void WbAbstractPose::updateConstrainedHandleMaterials() {
  mScaleManipulator->setResizeConstraint((WbScaleManipulator::ResizeConstraint)constraintType());
}

// Apply to WREN

void WbAbstractPose::applyTranslationToWren() {
  float translation[3];
  mTranslation->value().toFloatArray(translation);
  wr_transform_set_position(mBaseNode->wrenNode(), translation);
}

void WbAbstractPose::applyRotationToWren() {
  float rotation[4];
  mRotation->value().toFloatArray(rotation);
  wr_transform_set_orientation(mBaseNode->wrenNode(), rotation);
}

void WbAbstractPose::applyScaleToWren() {
  float scale[3];
  mScale->value().toFloatArray(scale);
  wr_transform_set_scale(mBaseNode->wrenNode(), scale);
}

void WbAbstractPose::applyTranslationAndRotationToWren() {  // for performance optimization
  float translation[3];
  mTranslation->value().toFloatArray(translation);
  float rotation[4];
  mRotation->value().toFloatArray(rotation);
  wr_transform_set_position_and_orientation(mBaseNode->wrenNode(), translation, rotation);
}

// Matrix 4-by-4

const WbMatrix4 &WbAbstractPose::matrix() const {
  // ensure that the matrix value is not used and computed before the node is finalized
  // because in some cases field values are only set after the node construction
  // (for example in case of PROTO instances)
  assert(mBaseNode->isPreFinalizedCalled());

  if (!mMatrix) {
    mMatrix = new WbMatrix4();
    updateMatrix();
  } else if (mMatrixNeedUpdate)
    updateMatrix();

  return *mMatrix;
}

void WbAbstractPose::updateMatrix() const {
  assert(mMatrix);

  mMatrix->fromVrml(mTranslation->x(), mTranslation->y(), mTranslation->z(), mRotation->x(), mRotation->y(), mRotation->z(),
                    mRotation->angle(), mScale->x(), mScale->y(), mScale->z());

  // multiply with upper matrix if any
  WbAbstractPose *transform = mBaseNode->upperTransform();
  if (transform)
    *mMatrix = transform->matrix() * *mMatrix;
  mMatrixNeedUpdate = false;
}

void WbAbstractPose::setMatrixNeedUpdateFlag() const {
  mMatrixNeedUpdate = true;
  mVrmlMatrixNeedUpdate = true;
}

const WbMatrix4 &WbAbstractPose::vrmlMatrix() const {
  if (mVrmlMatrixNeedUpdate) {
    mVrmlMatrix.fromVrml(translation(), rotation(), scale());
    mVrmlMatrixNeedUpdate = false;
  }

  return mVrmlMatrix;
}

// Absolute scale 3D-vector

const WbVector3 &WbAbstractPose::absoluteScale() const {
  if (mAbsoluteScaleNeedUpdate)
    updateAbsoluteScale();

  return mAbsoluteScale;
}

bool WbAbstractPose::isTopTransform() const {
  if (!mHasSearchedTopTransform) {
    mIsTopTransform = mBaseNode->upperTransform() == NULL;
    mHasSearchedTopTransform = true;
  }
  return mIsTopTransform;
}

void WbAbstractPose::updateAbsoluteScale() const {
  mAbsoluteScale = mScale->value();
  // multiply with upper transform scale if any
  const WbPose *const ut = mBaseNode->upperTransform();
  if (ut)
    mAbsoluteScale *= ut->absoluteScale();

  mAbsoluteScaleNeedUpdate = false;
}

void WbAbstractPose::setScaleNeedUpdateFlag() const {
  // optimisation: it's useless to call the function recursively if scalarScaleNeedUpdate is true,
  // because all the children's scalarNeedUpdate are already true.
  if (mAbsoluteScaleNeedUpdate)
    return;

  mAbsoluteScaleNeedUpdate = true;
}

// Position and orientation setters

void WbAbstractPose::setTranslationAndRotation(double tx, double ty, double tz, double rx, double ry, double rz, double angle) {
  mTranslation->setValue(tx, ty, tz);
  mRotation->setValue(rx, ry, rz, angle);
}

void WbAbstractPose::setTranslationAndRotation(const WbVector3 &v, const WbRotation &r) {
  mTranslation->setValue(v);
  mRotation->setValue(r);
}

void WbAbstractPose::setTranslation(double tx, double ty, double tz) {
  mTranslation->setValue(tx, ty, tz);
}

void WbAbstractPose::rotate(const WbVector3 &v) {
  WbMatrix3 rotation(v.normalized(), v.length());
  WbRotation newRotation = WbRotation(rotation * WbMatrix3(mRotation->value()));
  newRotation.normalize();
  setRotation(newRotation);
}

void WbAbstractPose::setRotation(double x, double y, double z, double angle) {
  mRotation->setValue(x, y, z, angle);
}

void WbAbstractPose::setRotationAngle(double angle) {
  mRotation->setAngle(angle);
}

void WbAbstractPose::setRotation(const WbRotation &r) {
  mRotation->setValue(r);
}

///////////////////////////////////////////////////////
//  WREN methods related to WbPose manipulators //
///////////////////////////////////////////////////////

void WbAbstractPose::showResizeManipulator(bool enabled) {
  if (enabled) {
    detachTranslateRotateManipulator();
    attachResizeManipulator();
    setUniformConstraintForResizeHandles(false);
  } else {
    detachResizeManipulator();
    attachTranslateRotateManipulator();
  }
}

void WbAbstractPose::updateResizeHandlesSize() {
  if (mScaleManipulator) {
    mScaleManipulator->updateHandleScale(matrix().scale().ptr());
    mScaleManipulator->computeHandleScaleFromViewportSize();
  }
}

void WbAbstractPose::setResizeManipulatorDimensions() {
  updateResizeHandlesSize();
}

void WbAbstractPose::attachResizeManipulator() {
  createScaleManipulatorIfNeeded();

  if (mScaleManipulator && !mScaleManipulator->isAttached()) {
    setResizeManipulatorDimensions();
    mScaleManipulator->show();
  }
}

void WbAbstractPose::detachResizeManipulator() const {
  if (mScaleManipulator && mScaleManipulator->isAttached())
    mScaleManipulator->hide();
}

bool WbAbstractPose::hasResizeManipulator() const {
  const WbField *const sf = mBaseNode->findField("scale", true);
  return WbNodeUtilities::isVisible(sf) && !WbNodeUtilities::isTemplateRegeneratorField(sf);
}

void WbAbstractPose::setUniformConstraintForResizeHandles(bool enabled) {
  createScaleManipulatorIfNeeded();

  if (!mScaleManipulator || !mScaleManipulator->isAttached())
    return;

  if (enabled)
    mScaleManipulator->setResizeConstraint(WbScaleManipulator::UNIFORM);
  else
    updateConstrainedHandleMaterials();
}

void WbAbstractPose::attachTranslateRotateManipulator() {
  createTranslateRotateManipulatorIfNeeded();

  if (mTranslateRotateManipulator) {
    if (!mTranslateRotateManipulator->isAttached()) {
      // get size
      WbBaseNode *node = baseNode();
      if (node->isInBoundingObject())
        node = WbNodeUtilities::findBoundingObjectAncestor(node);
      WbBoundingSphere *bs = node->boundingSphere();
      if (bs)
        bs->recomputeIfNeeded(false);

      mTranslateRotateManipulator->show();
    }
  }
}

void WbAbstractPose::detachTranslateRotateManipulator() {
  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    mTranslateRotateManipulator->hide();
}

void WbAbstractPose::updateTranslateRotateHandlesSize() {
  createTranslateRotateManipulatorIfNeeded();
  if (!mTranslateRotateManipulator)
    return;

  mTranslateRotateManipulator->updateHandleScale(absoluteScale().ptr());

  if (mTranslateRotateManipulator && !WbNodeUtilities::isNodeOrAncestorLocked(mBaseNode))
    mTranslateRotateManipulator->computeHandleScaleFromViewportSize();
}
