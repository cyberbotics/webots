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

#include "WbAbstractTransform.hpp"

#include "WbBaseNode.hpp"
#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbMatter.hpp"
#include "WbNodeUtilities.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSimulationState.hpp"
#include "WbTransform.hpp"
#include "WbTranslateRotateManipulator.hpp"

#include <wren/transform.h>

#include <QtCore/QObject>

void WbAbstractTransform::init(WbBaseNode *node) {
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

WbAbstractTransform::~WbAbstractTransform() {
  deleteWrenObjects();
  delete mMatrix;
}

void WbAbstractTransform::deleteWrenObjects() {
  delete mScaleManipulator;
  mScaleManipulator = NULL;

  delete mTranslateRotateManipulator;
  mTranslateRotateManipulator = NULL;
  mTranslateRotateManipulatorInitialized = false;
}

void WbAbstractTransform::updateTranslation() {
  mBaseNode->setMatrixNeedUpdate();

  if (mBaseNode->areWrenObjectsInitialized())
    applyTranslationToWren();

  if (WbSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();
}

void WbAbstractTransform::updateRotation() {
  mBaseNode->setMatrixNeedUpdate();

  if (mBaseNode->areWrenObjectsInitialized())
    applyRotationToWren();

  if (WbSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();

  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    updateTranslateRotateHandlesSize();
}

void WbAbstractTransform::updateTranslationAndRotation() {
  mBaseNode->setMatrixNeedUpdate();

  if (mBaseNode->areWrenObjectsInitialized())
    applyTranslationAndRotationToWren();

  if (WbSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();

  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    updateTranslateRotateHandlesSize();
}

bool WbAbstractTransform::checkScaleZeroValues(WbVector3 &correctedScale) const {
  const WbVector3 &s = mScale->value();
  const double x = s.x();
  const double y = s.y();
  const double z = s.z();
  correctedScale.setXyz(x, y, z);
  bool b = false;

  if (x == 0.0) {
    correctedScale.setX(1.0);
    mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be non-zero: x is set to 1.0."));
    b = true;
  }

  if (y == 0.0) {
    correctedScale.setY(1.0);
    mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be non-zero: y is set to 1.0."));
    b = true;
  }

  if (z == 0.0) {
    correctedScale.setZ(1.0);
    mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be non-zero: z is set to 1.0."));
    b = true;
  }

  return b;
}

bool WbAbstractTransform::checkScalingPhysicsConstraints(WbVector3 &correctedScale, int constraintType, bool warning) const {
  bool b = false;
  if (constraintType == WbWrenAbstractResizeManipulator::UNIFORM)
    b = checkScaleUniformity(correctedScale);
  else if (constraintType == WbWrenAbstractResizeManipulator::X_EQUAL_Y && mScale->x() != mScale->y()) {
    if (mPreviousXscaleValue == mScale->x())
      correctedScale.setX(mScale->y());
    else
      correctedScale.setY(mScale->x());
    b = true;
    if (warning)
      mBaseNode->parsingWarn(
        QObject::tr("'scale' were changed so that x = y because of physics constraints inside a 'boundingObject'."));
  }

  return b;
}

bool WbAbstractTransform::checkScaleUniformity(WbVector3 &correctedScale, bool warning) const {
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

bool WbAbstractTransform::checkScaleUniformity(bool warning) {
  WbVector3 correctedScale;

  if (checkScaleUniformity(correctedScale, warning)) {
    mScale->setValue(correctedScale);
    return true;
  }

  return false;
}

bool WbAbstractTransform::checkScale(int constraintType, bool warning) {
  WbVector3 correctedScale;
  bool b = false;

  if (checkScaleZeroValues(correctedScale))
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

void WbAbstractTransform::applyToScale() {
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

void WbAbstractTransform::updateScale(bool warning) {
  const int constraint = constraintType();
  if (checkScale(constraint, warning))
    return;

  applyToScale();
}

void WbAbstractTransform::updateTranslationFieldVisibility() const {
  if (mIsTranslationFieldVisibleReady)
    return;
  mIsTranslationFieldVisible = WbNodeUtilities::isVisible(mBaseNode->findField("translation", true));
  mCanBeTranslated =
    !WbNodeUtilities::isTemplateRegeneratorField(mBaseNode->findField("translation", true)) && mIsTranslationFieldVisible;
  mIsTranslationFieldVisibleReady = true;
}

void WbAbstractTransform::updateRotationFieldVisibility() const {
  if (mIsRotationFieldVisibleReady)
    return;
  mIsRotationFieldVisible = WbNodeUtilities::isVisible(mBaseNode->findField("rotation", true));
  mCanBeRotated =
    !WbNodeUtilities::isTemplateRegeneratorField(mBaseNode->findField("rotation", true)) && mIsRotationFieldVisible;
  mIsRotationFieldVisibleReady = true;
}

bool WbAbstractTransform::isTranslationFieldVisible() const {
  updateTranslationFieldVisibility();
  return mIsTranslationFieldVisible;
}

bool WbAbstractTransform::isRotationFieldVisible() const {
  updateRotationFieldVisibility();
  return mIsRotationFieldVisible;
}

bool WbAbstractTransform::canBeTranslated() const {
  updateTranslationFieldVisibility();
  return mCanBeTranslated;
}

bool WbAbstractTransform::canBeRotated() const {
  updateRotationFieldVisibility();
  return mCanBeRotated;
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void WbAbstractTransform::createScaleManipulator() {
  const int constraint = constraintType();
  mScaleManipulator = new WbScaleManipulator(mBaseNode->uniqueId(), (WbScaleManipulator::ResizeConstraint)constraint);
}

void WbAbstractTransform::createScaleManipulatorIfNeeded() {
  if (!mScaleManipulatorInitialized) {
    assert(hasResizeManipulator());  // otherwise the show resize manipulator option should be disabled
    mScaleManipulatorInitialized = true;
    createScaleManipulator();
    if (mScaleManipulator)
      mScaleManipulator->attachTo(baseNode()->wrenNode());
  }
}

void WbAbstractTransform::createTranslateRotateManipulatorIfNeeded() {
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

void WbAbstractTransform::updateConstrainedHandleMaterials() {
  mScaleManipulator->setResizeConstraint((WbScaleManipulator::ResizeConstraint)constraintType());
}

// Apply to WREN

void WbAbstractTransform::applyTranslationToWren() {
  float newTranslation[3];
  mTranslation->value().toFloatArray(newTranslation);
  wr_transform_set_position(mBaseNode->wrenNode(), newTranslation);
}

void WbAbstractTransform::applyRotationToWren() {
  float newRotation[4];
  mRotation->value().toFloatArray(newRotation);
  wr_transform_set_orientation(mBaseNode->wrenNode(), newRotation);
}

void WbAbstractTransform::applyScaleToWren() {
  float newScale[3];
  mScale->value().toFloatArray(newScale);
  wr_transform_set_scale(mBaseNode->wrenNode(), newScale);
}

void WbAbstractTransform::applyTranslationAndRotationToWren() {  // for performance optimization
  float newTranslation[3];
  mTranslation->value().toFloatArray(newTranslation);
  float newRotation[4];
  mRotation->value().toFloatArray(newRotation);
  wr_transform_set_position_and_orientation(mBaseNode->wrenNode(), newTranslation, newRotation);
}

// Matrix 4-by-4

const WbMatrix4 &WbAbstractTransform::matrix() const {
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

void WbAbstractTransform::updateMatrix() const {
  assert(mMatrix);

  mMatrix->fromVrml(mTranslation->x(), mTranslation->y(), mTranslation->z(), mRotation->x(), mRotation->y(), mRotation->z(),
                    mRotation->angle(), mScale->x(), mScale->y(), mScale->z());

  // multiply with upper matrix if any
  WbAbstractTransform *transform = mBaseNode->upperTransform();
  if (transform)
    *mMatrix = transform->matrix() * *mMatrix;
  mMatrixNeedUpdate = false;
}

void WbAbstractTransform::setMatrixNeedUpdateFlag() const {
  mMatrixNeedUpdate = true;
  mVrmlMatrixNeedUpdate = true;
}

const WbMatrix4 &WbAbstractTransform::vrmlMatrix() const {
  if (mVrmlMatrixNeedUpdate) {
    mVrmlMatrix.fromVrml(translation(), rotation(), scale());
    mVrmlMatrixNeedUpdate = false;
  }

  return mVrmlMatrix;
}

// Absolute scale 3D-vector

const WbVector3 &WbAbstractTransform::absoluteScale() const {
  if (mAbsoluteScaleNeedUpdate)
    updateAbsoluteScale();

  return mAbsoluteScale;
}

bool WbAbstractTransform::isTopTransform() const {
  if (!mHasSearchedTopTransform) {
    mIsTopTransform = mBaseNode->upperTransform() == NULL;
    mHasSearchedTopTransform = true;
  }
  return mIsTopTransform;
}

void WbAbstractTransform::updateAbsoluteScale() const {
  mAbsoluteScale = mScale->value();
  // multiply with upper transform scale if any
  const WbTransform *const ut = mBaseNode->upperTransform();
  if (ut)
    mAbsoluteScale *= ut->absoluteScale();

  mAbsoluteScaleNeedUpdate = false;
}

void WbAbstractTransform::setScaleNeedUpdateFlag() const {
  // optimisation: it's useless to call the function recursively if scalarScaleNeedUpdate is true,
  // because all the children's scalarNeedUpdate are already true.
  if (mAbsoluteScaleNeedUpdate)
    return;

  mAbsoluteScaleNeedUpdate = true;
}

// Position and orientation setters

void WbAbstractTransform::setTranslationAndRotation(double tx, double ty, double tz, double rx, double ry, double rz,
                                                    double angle) {
  mTranslation->setValue(tx, ty, tz);
  mRotation->setValue(rx, ry, rz, angle);
}

void WbAbstractTransform::setTranslationAndRotation(const WbVector3 &v, const WbRotation &r) {
  mTranslation->setValue(v);
  mRotation->setValue(r);
}

void WbAbstractTransform::setTranslation(double tx, double ty, double tz) {
  mTranslation->setValue(tx, ty, tz);
}

void WbAbstractTransform::rotate(const WbVector3 &v) {
  WbMatrix3 rotation(v.normalized(), v.length());
  WbRotation newRotation = WbRotation(rotation * WbMatrix3(mRotation->value()));
  newRotation.normalize();
  setRotation(newRotation);
}

void WbAbstractTransform::setRotation(double x, double y, double z, double angle) {
  mRotation->setValue(x, y, z, angle);
}

void WbAbstractTransform::setRotationAngle(double angle) {
  mRotation->setAngle(angle);
}

void WbAbstractTransform::setRotation(const WbRotation &r) {
  mRotation->setValue(r);
}

///////////////////////////////////////////////////////
//  WREN methods related to WbTransform manipulators //
///////////////////////////////////////////////////////

void WbAbstractTransform::showResizeManipulator(bool enabled) {
  if (enabled) {
    detachTranslateRotateManipulator();
    attachResizeManipulator();
    setUniformConstraintForResizeHandles(false);
  } else {
    detachResizeManipulator();
    attachTranslateRotateManipulator();
  }
}

void WbAbstractTransform::updateResizeHandlesSize() {
  if (mScaleManipulator) {
    mScaleManipulator->updateHandleScale(absoluteScale().ptr());
    mScaleManipulator->computeHandleScaleFromViewportSize();
  }
}

void WbAbstractTransform::setResizeManipulatorDimensions() {
  updateResizeHandlesSize();
}

bool WbAbstractTransform::isScaleManipulatorAttached() const {
  return mScaleManipulator ? mScaleManipulator->isAttached() : false;
}

void WbAbstractTransform::attachResizeManipulator() {
  createScaleManipulatorIfNeeded();

  if (mScaleManipulator && !mScaleManipulator->isAttached()) {
    setResizeManipulatorDimensions();
    mScaleManipulator->show();
  }
}

void WbAbstractTransform::detachResizeManipulator() const {
  if (mScaleManipulator && mScaleManipulator->isAttached())
    mScaleManipulator->hide();
}

bool WbAbstractTransform::hasResizeManipulator() const {
  const WbField *const sf = mBaseNode->findField("scale", true);
  return WbNodeUtilities::isVisible(sf) && !WbNodeUtilities::isTemplateRegeneratorField(sf);
}

void WbAbstractTransform::setUniformConstraintForResizeHandles(bool enabled) {
  createScaleManipulatorIfNeeded();

  if (!mScaleManipulator || !mScaleManipulator->isAttached())
    return;

  if (enabled)
    mScaleManipulator->setResizeConstraint(WbScaleManipulator::UNIFORM);
  else
    updateConstrainedHandleMaterials();
}

void WbAbstractTransform::attachTranslateRotateManipulator() {
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

void WbAbstractTransform::detachTranslateRotateManipulator() {
  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    mTranslateRotateManipulator->hide();
}

void WbAbstractTransform::updateTranslateRotateHandlesSize() {
  createTranslateRotateManipulatorIfNeeded();
  if (!mTranslateRotateManipulator)
    return;

  mTranslateRotateManipulator->updateHandleScale(absoluteScale().ptr());

  if (!WbNodeUtilities::isNodeOrAncestorLocked(mBaseNode))
    mTranslateRotateManipulator->computeHandleScaleFromViewportSize();
}
