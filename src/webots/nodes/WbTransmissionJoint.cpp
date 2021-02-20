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

#include <wren/config.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

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

void WbTransmissionJoint::preFinalize() {
  WbJoint::preFinalize();

  WbBaseNode *const p2 = dynamic_cast<WbBaseNode *>(mParameters2->value());
  if (p2 && !p2->isPreFinalizedCalled())
    p2->preFinalize();

  updateParameters2();
  mInitialPosition2 = mPosition2;
}

void WbTransmissionJoint::postFinalize() {
  WbJoint::postFinalize();

  WbBaseNode *const p2 = dynamic_cast<WbBaseNode *>(mParameters2->value());
  if (p2 && !p2->isPostFinalizedCalled())
    p2->postFinalize();

  connect(mParameters2, &WbSFNode::changed, this, &WbTransmissionJoint::updateParameters2);
  connect(mBacklash, &WbSFDouble::changed, this, &WbTransmissionJoint::updateBacklash);
  connect(mMultiplier, &WbSFDouble::changed, this, &WbTransmissionJoint::updateMultiplier);
}

void WbTransmissionJoint::prePhysicsStep(double ms) {
  // endPoint
  assert(solidEndPoint());
  WbRotationalMotor *const rm = rotationalMotor();
  WbJointParameters *const p = parameters();
  if (isEnabled()) {
    if (rm && rm->userControl()) {
      // user-defined torque
      const double torque = rm->rawInput();
      dJointAddHingeTorque(mJoint, torque);
      if (rm->hasMuscles())
        // force is directly applied to the bodies and not included in joint motor feedback
        emit updateMuscleStretch(torque / rm->maxForceOrTorque(), false, 1);
    } else {
      // ODE motor torque (user velocity/position control)
      const double currentVelocity = rm ? rm->computeCurrentDynamicVelocity(ms, mPosition) : 0.0;
      const double fMax = qMax(p ? p->staticFriction() : 0.0, rm ? rm->torque() : 0.0);
      const double s = upperTransform()->absoluteScale().x();
      double s4 = s * s;
      s4 *= s4;
      dJointSetHingeParam(mJoint, dParamFMax, s * s4 * fMax);
      dJointSetHingeParam(mJoint, dParamVel, currentVelocity);
    }
  } else if (rm && rm->runKinematicControl(ms, mPosition)) {  // kinematic mode
    if (p)
      p->setPosition(mPosition);
    else
      updatePosition(mPosition);
    if (rm->hasMuscles()) {
      double velocityPercentage = rm->currentVelocity() / rm->maxVelocity();
      if (rm->kinematicVelocitySign() == -1)
        velocityPercentage = -velocityPercentage;
      emit updateMuscleStretch(velocityPercentage, true, 1);
    }
  }
  mTimeStep = ms;
}

void WbTransmissionJoint::postPhysicsStep() {
  // endPoint
  assert(mJoint);
  WbRotationalMotor *const rm = rotationalMotor();
  if (rm && rm->isPIDPositionControl()) {  // if controlling in position we update position using directly the angle feedback
    double angle = dJointGetHingeAngle(mJoint);
    mPosition = WbMathsUtilities::normalizeAngle(angle + mOdePositionOffset, mPosition);
  } else {
    // if not controlling in position we use the angle rate feedback to update position (because at high speed angle feedback is
    // under-estimated)
    double angleRate = dJointGetHingeAngleRate(mJoint);
    mPosition -= angleRate * mTimeStep / 1000.0;
  }
  WbJointParameters *const p = parameters();
  if (p)
    p->setPositionFromOde(mPosition);

  if (isEnabled() && rm && rm->hasMuscles() && !rm->userControl())
    // dynamic position or velocity control
    emit updateMuscleStretch(rm->computeFeedback() / rm->maxForceOrTorque(), false, 1);
  // transmission
  /* // only needed for drawing th contact point
  dVector3 c_1, c_2, a_1, a_2;
  dJointGetTransmissionContactPoint1(mTransmission, c_1);
  dJointGetTransmissionContactPoint2(mTransmission, c_2);
  dJointGetTransmissionAnchor1(mTransmission, a_1);
  dJointGetTransmissionAnchor2(mTransmission, a_2);
  */
}

WbHingeJointParameters *WbTransmissionJoint::hingeJointParameters() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters->value());
}

WbHingeJointParameters *WbTransmissionJoint::hingeJointParameters2() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters2->value());
}

WbRotationalMotor *WbTransmissionJoint::rotationalMotor() const {
  WbRotationalMotor *motor = NULL;
  for (int i = 0; i < mDevice->size(); ++i) {
    motor = dynamic_cast<WbRotationalMotor *>(mDevice->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

WbVector3 WbTransmissionJoint::anchor() const {
  static const WbVector3 DEFAULT_ANCHOR(-1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p = hingeJointParameters();
  return p ? p->anchor() : DEFAULT_ANCHOR;
}

WbVector3 WbTransmissionJoint::anchor2() const {
  static const WbVector3 DEFAULT_ANCHOR(1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
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

bool WbTransmissionJoint::setJoint() {
  if (!WbBasicJoint::setJoint())
    return false;

  if (mJoint == NULL)
    mJoint = dJointCreateHinge(WbOdeContext::instance()->world(), 0);

  const WbSolid *const s = solidEndPoint();
  setOdeJoint(s ? s->body() : NULL, upperSolid()->bodyMerger());

  return true;
}

bool WbTransmissionJoint::setJoint2() {
  // TODO

  return true;
}

void WbTransmissionJoint::setOdeJoint(dBodyID body, dBodyID parentBody) {
  WbJoint::setOdeJoint(body, parentBody);
  // compute and set the anchor point
  applyToOdeAnchor();
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
  assert(mJoint2);

  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &t = m4 * anchor2();
  dJointSetHingeAnchor(mJoint2, t.x(), t.y(), t.z());
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
  WbHingeJointParameters *const p = hingeJointParameters();
  if (p) {
    // p->setAxis(WbVector3(0.0, 1.0, 0.0));
    // p->setAnchor(WbVector3(-0.1, 0.0, 0.0));
    connect(p, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor, Qt::UniqueConnection);
  }
}

void WbTransmissionJoint::updateParameters2() {
  printf("updateParameters2\n");
  WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (p2) {
    // p2->setAxis(WbVector3(0.0, 1.0, 0.0));
    // p2->setAnchor(WbVector3(0.1, 0.0, 0.0));
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

void WbTransmissionJoint::updatePosition2(double position) {
  // TODO
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

  if (mParameters->value() == NULL || mParameters2->value() == NULL) {
    printf("jointParameters or jointParameters2 not defined\n");
    return;
  }

  if (!mTransmission)
    mTransmission = dJointCreateTransmission(WbOdeContext::instance()->world(), 0);

  const WbVector3 &ax1 = axis();
  const WbVector3 &ax2 = axis2();
  const WbVector3 &an1 = anchor();
  const WbVector3 &an2 = anchor2();

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

  printf("transmission set up\n");
}

void WbTransmissionJoint::updateJointAxisRepresentation() {
  if (!areWrenObjectsInitialized())
    return;

  wr_static_mesh_delete(mMesh);

  float vertices[12];
  const double scaling = 0.5f * wr_config_get_line_scale();
  const WbVector3 &anchorVector = anchor();
  const WbVector3 &axisVector = scaling * axis();
  const WbVector3 &anchorVector2 = anchor2();
  const WbVector3 &axisVector2 = scaling * axis2();

  // joint on endPoint side
  WbVector3 vertex(anchorVector - axisVector);
  vertex.toFloatArray(vertices);

  vertex = anchorVector + axisVector;
  vertex.toFloatArray(vertices + 3);

  // joint on startPoint side
  vertex = anchorVector2 - axisVector2;
  vertex.toFloatArray(vertices + 6);

  vertex = anchorVector2 + axisVector2;
  vertex.toFloatArray(vertices + 9);

  mMesh = wr_static_mesh_line_set_new(4, vertices, NULL);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
}

void WbTransmissionJoint::applyToOdeMinAndMaxStop() {
}

void WbTransmissionJoint::applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) {
}
