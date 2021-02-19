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
#include "WbTransmissionJoint.hpp"

#include "WbBrake.hpp"
#include "WbFieldChecker.hpp"
#include "WbHingeJointParameters.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeUtilities.hpp"
#include "WbRotationalMotor.hpp"
#include "WbSolid.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <cassert>

// Constructors

void WbTransmissionJoint::init() {
  mJoint2 = NULL;
  mTransmission = NULL;
  mTransmissionMode = -1;
  mParameters2 = findSFNode("jointParameters2");
  mStartPoint = findSFNode("startPoint");
  mBacklash = findSFDouble("backlash");
  mMultiplier = findSFDouble("multiplier");
  mStartPoint = findSFNode("startPoint");

  // hidden field
  mPosition2 = findSFDouble("position2")->value();
  mOdePositionOffset2 = mPosition2;
  mInitialPosition2 = mPosition2;
}

WbTransmissionJoint::WbTransmissionJoint(const QString &modelName, WbTokenizer *tokenizer) : WbJoint(modelName, tokenizer) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(WbTokenizer *tokenizer) : WbJoint("TransmissionJoint", tokenizer) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(const WbTransmissionJoint &other) : WbJoint(other) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(const WbNode &other) : WbJoint(other) {
  init();
}

WbTransmissionJoint::~WbTransmissionJoint() {
}

WbHingeJointParameters *WbTransmissionJoint::hingeJointParameters() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters->value());
}

WbHingeJointParameters *WbTransmissionJoint::hingeJointParameters2() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters2->value());
}

WbVector3 WbTransmissionJoint::anchor() const {
  static const WbVector3 DEFAULT_ANCHOR(-1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p = hingeJointParameters();
  return p ? p->anchor() : DEFAULT_ANCHOR;
}

WbVector3 WbTransmissionJoint::anchor2() const {
  static const WbVector3 DEFAULT_ANCHOR(1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p2 = hingeJointParameters();
  return p2 ? p2->anchor() : DEFAULT_ANCHOR;
}

WbVector3 WbTransmissionJoint::axis() const {
  static const WbVector3 DEFAULT_AXIS(0.0, 1.0, 0.0);
  const WbHingeJointParameters *const p = hingeJointParameters();
  return p ? p->axis() : DEFAULT_AXIS;
}

WbVector3 WbTransmissionJoint::axis2() const {
  static const WbVector3 DEFAULT_AXIS(0.0, 1.0, 0.0);
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  return p2 ? p2->axis() : DEFAULT_AXIS;
}

void WbTransmissionJoint::applyToOdeAxis() {
  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  WbVector3 a = m4.sub3x3MatrixDot(axis());
  dJointSetHingeAxis(mJoint, a.x(), a.y(), a.z());
}

void WbTransmissionJoint::applyToOdeAxis2() {
  // TODO
}

void WbTransmissionJoint::applyToOdeAnchor() {
  assert(mJoint);

  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &t = m4 * anchor();
  dJointSetHingeAnchor(mJoint, t.x(), t.y(), t.z());
}

void WbTransmissionJoint::applyToOdeAnchor2() {
  // TODO
}

void WbTransmissionJoint::updateAxis() {
  printf("updateAxis\n");
  // update the current endPoint pose based on the new axis value
  updatePosition();

  if (mJoint)
    applyToOdeAxis();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  inferTransmissionMode();
}

void WbTransmissionJoint::updateAxis2() {
  printf("updateAxis2\n");
  // update the current startPoint pose based on the new axis value
  updatePosition2();

  if (mJoint2)
    applyToOdeAxis2();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  inferTransmissionMode();
}

void WbTransmissionJoint::updateAnchor() {
  printf("updateAnchor\n");
  // update the current endPoint pose based on the new anchor value
  updatePosition();

  if (mJoint)
    applyToOdeAnchor();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  inferTransmissionMode();
}

void WbTransmissionJoint::updateAnchor2() {
  printf("updateAnchor2\n");
  // update the current startPoint pose based on the new anchor value
  updatePosition2();

  if (mJoint2)
    applyToOdeAnchor2();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  inferTransmissionMode();
}

double WbTransmissionJoint::position(int index) const {
  switch (index) {
    case 1:
      return mPosition;
    case 2:
      return mPosition2;
    default:
      return NAN;
  }
}

double WbTransmissionJoint::initialPosition(int index) const {
  switch (index) {
    case 1:
      return mInitialPosition;
    case 2:
      return mInitialPosition2;
    default:
      return NAN;
  }
}

// Updates

void WbTransmissionJoint::updateParameters() {
  printf("updateParameters\n");
  const WbHingeJointParameters *const p = hingeJointParameters();
  if (p) {
    mOdePositionOffset = p->position();
    mPosition = mOdePositionOffset;
    connect(p, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor, Qt::UniqueConnection);
  }
}

void WbTransmissionJoint::updateParameters2() {
  printf("updateParameters2\n");
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (p2) {
    mOdePositionOffset = p2->position();
    mPosition2 = mOdePositionOffset2;
    connect(p2, SIGNAL(positionChanged()), this, SLOT(updatePosition2()), Qt::UniqueConnection);
    connect(p2, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis2, Qt::UniqueConnection);
    connect(p2, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor2, Qt::UniqueConnection);
  }
}

void WbTransmissionJoint::updatePosition() {
  const WbHingeJointParameters *const p = hingeJointParameters();

  if (solidReference() == NULL && solidEndPoint())
    updatePosition(p ? p->position() : mPosition);

  emit updateMuscleStretch(0.0, true, 0);
}

void WbTransmissionJoint::updatePosition2() {
  // TODO
}

void WbTransmissionJoint::updatePosition(double position) {
  WbSolid *const s = solidEndPoint();
  assert(s);
  // called after an artificial move
  mPosition = position;
  WbMotor *m = motor();
  if (m && !m->isConfigureDone())
    m->setTargetPosition(position);
  WbVector3 translation;
  WbRotation rotation;
  computeEndPointSolidPositionFromParameters(translation, rotation);
  if (!translation.almostEquals(s->translation()) || !rotation.almostEquals(s->rotation())) {
    mIsEndPointPositionChangedByJoint = true;
    s->setTranslationAndRotation(translation, rotation);
    s->resetPhysics();
    mIsEndPointPositionChangedByJoint = false;
  }
}

void WbTransmissionJoint::updateEndPointZeroTranslationAndRotation() {
  if (solidEndPoint() == NULL)
    return;

  WbRotation ir;
  WbVector3 it;
  retrieveEndPointSolidTranslationAndRotation(it, ir);

  WbQuaternion qMinus;
  const double angle = mPosition;
  if (WbMathsUtilities::isZeroAngle(angle)) {
    // In case of a zero angle, the quaternion axis is undefined, so we keep track of the original one
    mEndPointZeroRotation = ir;
  } else {
    const WbVector3 &ax = axis().normalized();
    qMinus = WbQuaternion(ax, -angle);
    const WbQuaternion &q = ir.toQuaternion();
    WbQuaternion qNormalized = qMinus * q;
    if (qNormalized.w() != 1.0)
      qNormalized.normalize();
    mEndPointZeroRotation = WbRotation(qNormalized);
    if (mEndPointZeroRotation.angle() == 0.0)
      mEndPointZeroRotation = WbRotation(ax.x(), ax.y(), ax.z(), 0.0);
  }
  const WbVector3 &an = anchor();
  mEndPointZeroTranslation = qMinus * (it - an) + an;
}

void WbTransmissionJoint::computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const {
  const WbVector3 &ax = axis().normalized();
  const WbQuaternion q(ax, mPosition);
  const WbQuaternion iq(mEndPointZeroRotation.toQuaternion());
  WbQuaternion qp(q * iq);
  if (qp.w() != 1.0)
    qp.normalize();
  rotation.fromQuaternion(qp);
  if (rotation.angle() == 0.0)
    rotation = WbRotation(ax.x(), ax.y(), ax.z(), 0.0);
  const WbVector3 &a = anchor();
  translation = q * (mEndPointZeroTranslation - a) + a;
}

void WbTransmissionJoint::updateBacklash() {
  WbFieldChecker::resetDoubleIfNegative(this, mBacklash, 0.0);
  printf("new backlash %f\n", mBacklash->value());
}

void WbTransmissionJoint::updateMultiplier() {
  if (mMultiplier->isZero()) {
    mMultiplier->setValue(1);
    parsingWarn(tr("'multiplier' must be different from zero, setting it back to 1."));
  }

  printf("new multiplier = %f\n", mMultiplier->value());

  inferTransmissionMode();
}

void WbTransmissionJoint::inferTransmissionMode() {
  mTransmissionMode = -1;
  const bool isCodirectional = axis().normalized().almostEquals(axis2().normalized());
  if (mMultiplier->value() < 0.0 && isCodirectional)
    mTransmissionMode = dTransmissionParallelAxes;
  else if (mMultiplier->value() > 0.0 && isCodirectional)
    mTransmissionMode = dTransmissionChainDrive;
  else {
    // determine if they intersect
    const bool isCoplanar = fabs(axis().cross(axis2()).dot(anchor() - anchor2())) < 1e-10;
    const bool isParallel = fabs(axis().cross(axis2()).length2()) < 1e-10;
    if (isCoplanar && !isParallel)
      mTransmissionMode = dTransmissionIntersectingAxes;
  }

  switch (mTransmissionMode) {
    case dTransmissionParallelAxes:
      printf("geartype = CLASSIC GEAR\n");
      break;
    case dTransmissionChainDrive:
      printf("geartype = CHAIN DRIVE\n");
      break;
    case dTransmissionIntersectingAxes:
      printf("geartype = BEVEL GEAR\n");
      break;
    default:
      printf("geartype = UNDEFINED\n");
  }

  setupTransmission();
}

void WbTransmissionJoint::setupTransmission() {
  if (mTransmissionMode == -1) {
    printf("undefined transmission\n");
    return;
  }

  if (!mTransmission)
    mTransmission = dJointCreateTransmission(WbOdeContext::instance()->world(), 0);

  const WbVector3 &ax1 = axis();
  const WbVector3 &ax2 = axis2();
  const WbVector3 &an1 = anchor();
  const WbVector3 &an2 = anchor2();

  printf("%f %f %f\n", an1.x(), an1.y(), an1.z());
  dJointSetTransmissionAnchor1(mTransmission, an1.x(), an1.y(), an1.z());
  dJointSetTransmissionAnchor2(mTransmission, an2.x(), an2.y(), an2.z());

  dJointSetTransmissionMode(mTransmission, mTransmissionMode);
  dJointSetTransmissionBacklash(mTransmission, mBacklash->value());

  if (mTransmissionMode == dTransmissionParallelAxes) {
    dJointSetTransmissionRatio(mTransmission, mMultiplier->value());
    dJointSetTransmissionAxis(mTransmission, ax1.x(), ax1.y(), ax1.z());
  }
  if (mTransmissionMode == dTransmissionChainDrive) {
    dJointSetTransmissionRadius1(mTransmission, 1.0);
    dJointSetTransmissionRadius2(mTransmission, mMultiplier->value());
    dJointSetTransmissionAxis(mTransmission, ax1.x(), ax1.y(), ax1.z());
  }
  if (mTransmissionMode == dTransmissionIntersectingAxes) {
    dJointSetTransmissionAxis1(mTransmission, ax1.x(), ax1.y(), ax1.z());
    dJointSetTransmissionAxis2(mTransmission, ax2.x(), ax2.y(), ax2.z());
  }
}

void WbTransmissionJoint::applyToOdeMinAndMaxStop() {
}

void WbTransmissionJoint::applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) {
}
