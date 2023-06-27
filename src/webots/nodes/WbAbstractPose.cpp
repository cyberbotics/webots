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

#include "WbAbstractPose.hpp"

#include "WbBaseNode.hpp"
#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbMatter.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPose.hpp"
#include "WbSimulationState.hpp"
#include "WbTransform.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbVrmlNodeUtilities.hpp"

#include <wren/transform.h>

void WbAbstractPose::init(WbBaseNode *node) {
  mBaseNode = node;
  assert(mBaseNode);

  mTranslation = node->findSFVector3("translation");
  mRotation = node->findSFRotation("rotation");
  mTranslationStep = node->findSFDouble("translationStep");
  mRotationStep = node->findSFDouble("rotationStep");

  mMatrix = NULL;
  mMatrixNeedUpdate = true;
  mVrmlMatrixNeedUpdate = true;
  mTranslateRotateManipulator = NULL;

  mTranslateRotateManipulatorInitialized = false;

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

void WbAbstractPose::updateTranslationFieldVisibility() const {
  if (mIsTranslationFieldVisibleReady)
    return;
  mIsTranslationFieldVisible = WbVrmlNodeUtilities::isVisible(mBaseNode->findField("translation", true));
  mCanBeTranslated =
    !WbNodeUtilities::isTemplateRegeneratorField(mBaseNode->findField("translation", true)) && mIsTranslationFieldVisible;
  mIsTranslationFieldVisibleReady = true;
}

void WbAbstractPose::updateRotationFieldVisibility() const {
  if (mIsRotationFieldVisibleReady)
    return;
  mIsRotationFieldVisible = WbVrmlNodeUtilities::isVisible(mBaseNode->findField("rotation", true));
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

// Apply to WREN

void WbAbstractPose::applyTranslationToWren() {
  float newTranslation[3];
  mTranslation->value().toFloatArray(newTranslation);
  wr_transform_set_position(mBaseNode->wrenNode(), newTranslation);
}

void WbAbstractPose::applyRotationToWren() {
  float newRotation[4];
  mRotation->value().toFloatArray(newRotation);
  wr_transform_set_orientation(mBaseNode->wrenNode(), newRotation);
}

void WbAbstractPose::applyTranslationAndRotationToWren() {  // for performance optimization
  float newTranslation[3];
  mTranslation->value().toFloatArray(newTranslation);
  float newRotation[4];
  mRotation->value().toFloatArray(newRotation);
  wr_transform_set_position_and_orientation(mBaseNode->wrenNode(), newTranslation, newRotation);
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
                    mRotation->angle(), 1.0, 1.0, 1.0);

  // multiply with upper matrix if any
  const WbPose *pose = mBaseNode->upperPose();
  if (pose) {
    const WbTransform *const transform = dynamic_cast<const WbTransform *const>(pose);
    *mMatrix = transform ? transform->matrix() * *mMatrix : pose->matrix() * *mMatrix;
  }
  mMatrixNeedUpdate = false;
}

void WbAbstractPose::setMatrixNeedUpdateFlag() const {
  mMatrixNeedUpdate = true;
  mVrmlMatrixNeedUpdate = true;
}

const WbMatrix4 &WbAbstractPose::vrmlMatrix() const {
  if (mVrmlMatrixNeedUpdate) {
    mVrmlMatrix.fromVrml(translation(), rotation(), WbVector3(1, 1, 1));
    mVrmlMatrixNeedUpdate = false;
  }

  return mVrmlMatrix;
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

  const WbTransform *transform = dynamic_cast<WbTransform *>(mBaseNode);
  if (!transform)
    transform = mBaseNode->upperTransform();
  if (transform)
    mTranslateRotateManipulator->updateHandleScale(transform->absoluteScale().ptr());

  if (!WbNodeUtilities::isNodeOrAncestorLocked(mBaseNode))
    mTranslateRotateManipulator->computeHandleScaleFromViewportSize();
}
