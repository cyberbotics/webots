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

#include "WbTransform.hpp"

#include "WbBoundingSphere.hpp"
#include "WbNodeUtilities.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSimulationState.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbVrmlNodeUtilities.hpp"

void WbTransform::init() {
  mScale = findSFVector3("scale");

  mAbsoluteScaleNeedUpdate = true;
  mPreviousXscaleValue = 1.0;

  mScaleManipulator = NULL;
  mScaleManipulatorInitialized = false;
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

WbTransform::~WbTransform() {
  disconnect(childrenField(), &WbMFNode::changed, this, &WbTransform::updateConstrainedHandleMaterials);
}

void WbTransform::preFinalize() {
  WbPose::preFinalize();

  checkScale(0, true);
}

void WbTransform::postFinalize() {
  WbPose::postFinalize();

  connect(mScale, SIGNAL(changed()), this, SLOT(updateScale()));
}

void WbTransform::deleteWrenObjects() {
  WbPose::deleteWrenObjects();

  delete mScaleManipulator;
  mScaleManipulator = NULL;
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

  // TODO: needed? transform can't be in BO -> cleanup
  if (isInBoundingObject() && isAValidBoundingObject())
    applyToOdeScale();
}

void WbTransform::updateScale(bool warning) {
  const int constraint = constraintType();
  if (checkScale(constraint, warning))
    return;

  applyToScale();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

bool WbTransform::checkScale(int constraintType, bool warning) {
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

int WbTransform::constraintType() const {
  static const int CONSTRAINT = WbWrenAbstractResizeManipulator::NO_CONSTRAINT;
  const WbGeometry *const g = geometry();

  if (g && (nodeUse() & WbNode::BOUNDING_OBJECT_USE))
    return g->constraintType();
  return CONSTRAINT;
}

void WbTransform::applyToOdeScale() {
  geometry()->applyToOdeData();
}

QStringList WbTransform::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "scale" << WbPose::fieldsToSynchronizeWithX3D();
  return fields;
}

void WbTransform::updateAbsoluteScale() const {
  mAbsoluteScale = mScale->value();
  // multiply with upper transform scale if any
  const WbTransform *const ut = dynamic_cast<const WbTransform *const>(mBaseNode->upperPose());
  if (ut)
    mAbsoluteScale *= ut->absoluteScale();

  mAbsoluteScaleNeedUpdate = false;
}

bool WbTransform::checkScaleZeroValues(WbVector3 &correctedScale) const {
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

bool WbTransform::checkScalingPhysicsConstraints(WbVector3 &correctedScale, int constraintType, bool warning) const {
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

// Absolute scale 3D-vector

const WbVector3 &WbTransform::absoluteScale() const {
  if (mAbsoluteScaleNeedUpdate)
    updateAbsoluteScale();

  return mAbsoluteScale;
}

WbMatrix3 WbTransform::rotationMatrix() const {
  const WbVector3 &s = absoluteScale();
  WbMatrix3 m = matrix().extracted3x3Matrix();
  m.scale(1.0 / s.x(), 1.0 / s.y(), 1.0 / s.z());
  return m;
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void WbTransform::createWrenObjects() {
  WbPose::createWrenObjects();

  applyScaleToWren();
}

void WbTransform::createScaleManipulator() {
  const int constraint = constraintType();
  mScaleManipulator = new WbScaleManipulator(uniqueId(), (WbScaleManipulator::ResizeConstraint)constraint);
  if (constraint) {
    connect(childrenField(), &WbMFNode::destroyed, mScaleManipulator, &WbScaleManipulator::hide);
    connect(childrenField(), &WbMFNode::changed, this, &WbTransform::updateConstrainedHandleMaterials);
  }
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

void WbTransform::setScaleNeedUpdate() {
  setScaleNeedUpdateFlag();
  WbGroup::setScaleNeedUpdate();
}

void WbTransform::updateConstrainedHandleMaterials() {
  mScaleManipulator->setResizeConstraint((WbScaleManipulator::ResizeConstraint)constraintType());
}

void WbTransform::applyScaleToWren() {
  float newScale[3];
  mScale->value().toFloatArray(newScale);
  wr_transform_set_scale(mBaseNode->wrenNode(), newScale);
}

///////////////////////////////////////////////////////
//  WREN methods related to WbTransform manipulators //
///////////////////////////////////////////////////////

void WbTransform::setScaleNeedUpdateFlag() const {
  // optimisation: it's useless to call the function recursively if scalarScaleNeedUpdate is true,
  // because all the children's scalarNeedUpdate are already true.
  if (mAbsoluteScaleNeedUpdate)
    return;

  mAbsoluteScaleNeedUpdate = true;
}

void WbTransform::showResizeManipulator(bool enabled) {
  if (enabled) {
    detachTranslateRotateManipulator();
    attachResizeManipulator();
    setUniformConstraintForResizeHandles(false);
  } else {
    detachResizeManipulator();
    attachTranslateRotateManipulator();
  }
}

void WbTransform::updateResizeHandlesSize() {
  if (mScaleManipulator) {
    mScaleManipulator->updateHandleScale(absoluteScale().ptr());
    mScaleManipulator->computeHandleScaleFromViewportSize();
  }
}

void WbTransform::setResizeManipulatorDimensions() {
  updateResizeHandlesSize();
}

bool WbTransform::isScaleManipulatorAttached() const {
  return mScaleManipulator ? mScaleManipulator->isAttached() : false;
}

void WbTransform::attachResizeManipulator() {
  createScaleManipulatorIfNeeded();

  if (mScaleManipulator && !mScaleManipulator->isAttached()) {
    setResizeManipulatorDimensions();
    mScaleManipulator->show();
  }
}

void WbTransform::detachResizeManipulator() const {
  if (mScaleManipulator && mScaleManipulator->isAttached())
    mScaleManipulator->hide();
}

bool WbTransform::hasResizeManipulator() const {
  const WbField *const sf = mBaseNode->findField("scale", true);
  return WbVrmlNodeUtilities::isVisible(sf) && !WbNodeUtilities::isTemplateRegeneratorField(sf);
}

void WbTransform::setUniformConstraintForResizeHandles(bool enabled) {
  createScaleManipulatorIfNeeded();

  if (!mScaleManipulator || !mScaleManipulator->isAttached())
    return;

  if (enabled)
    mScaleManipulator->setResizeConstraint(WbScaleManipulator::UNIFORM);
  else
    updateConstrainedHandleMaterials();
}
