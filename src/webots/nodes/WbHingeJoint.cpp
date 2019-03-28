// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbHingeJoint.hpp"
#include "WbBrake.hpp"
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

WbHingeJoint::WbHingeJoint(const QString &modelName, WbTokenizer *tokenizer) : WbJoint(modelName, tokenizer) {
}

WbHingeJoint::WbHingeJoint(WbTokenizer *tokenizer) : WbJoint("HingeJoint", tokenizer) {
}

WbHingeJoint::WbHingeJoint(const WbHingeJoint &other) : WbJoint(other) {
}

WbHingeJoint::WbHingeJoint(const WbNode &other) : WbJoint(other) {
}

WbHingeJoint::~WbHingeJoint() {
}

WbHingeJointParameters *WbHingeJoint::hingeJointParameters() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters->value());
}

WbRotationalMotor *WbHingeJoint::rotationalMotor() const {
  WbRotationalMotor *motor = NULL;
  for (int i = 0; i < mDevice->size(); ++i) {
    motor = dynamic_cast<WbRotationalMotor *>(mDevice->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

void WbHingeJoint::updateEndPointZeroTranslationAndRotation() {
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

void WbHingeJoint::computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const {
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

bool WbHingeJoint::setJoint() {
  if (!WbBasicJoint::setJoint())
    return false;

  if (mJoint == NULL)
    mJoint = dJointCreateHinge(WbOdeContext::instance()->world(), 0);

  const WbSolid *const s = solidEndPoint();
  setOdeJoint(s ? s->body() : NULL, upperSolid()->bodyMerger());

  return true;
}

void WbHingeJoint::setOdeJoint(dBodyID body, dBodyID parentBody) {
  WbJoint::setOdeJoint(body, parentBody);
  // compute and set the anchor point and suspension
  applyToOdeAnchor();
  applyToOdeSuspension();
}

void WbHingeJoint::applyToOdeMinAndMaxStop() {
  assert(mJoint);
  const WbJointParameters *const p = parameters();
  const double m = p ? p->minStop() : 0.0;
  const double M = p ? p->maxStop() : 0.0;
  if (m != M) {
    double min = -M + mOdePositionOffset;
    double max = -m + mOdePositionOffset;
    WbMathsUtilities::clampAngles(min, max);
    dJointSetHingeParam(mJoint, dParamLoStop, min);
    dJointSetHingeParam(mJoint, dParamHiStop, max);
  }
}

void WbHingeJoint::applyToOdeAxis() {
  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  WbVector3 a = m4.sub3x3MatrixDot(axis());
  if (mIsReverseJoint)
    a = -a;  // the axis should be inverted when the upper solid has no physics node
  dJointSetHingeAxis(mJoint, a.x(), a.y(), a.z());
  if (mSpringAndDamperMotor)
    dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, a.x(), a.y(), a.z());
}

void WbHingeJoint::applyToOdeSuspensionAxis() {
  // suspension along the suspension axis
  const WbHingeJointParameters *const hp = hingeJointParameters();
  if (hp == NULL)
    return;
  const WbMatrix4 &m4 = upperTransform()->matrix();
  WbVector3 a = m4.sub3x3MatrixDot(hp->suspensionAxis());
  if (mIsReverseJoint)
    a = -a;  // the axis should be inverted when the upper solid has no physics node
  a.normalize();
  if (nodeType() == WB_NODE_HINGE_2_JOINT)
    dJointSetHinge2SuspensionAxis(mJoint, a.x(), a.y(), a.z());
  else
    dJointSetHingeSuspensionAxis(mJoint, a.x(), a.y(), a.z());
}

void WbHingeJoint::applyToOdeAnchor() {
  assert(mJoint);

  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &t = m4 * anchor();
  if (nodeType() == WB_NODE_HINGE_2_JOINT)
    dJointSetHinge2Anchor(mJoint, t.x(), t.y(), t.z());
  else
    dJointSetHingeAnchor(mJoint, t.x(), t.y(), t.z());
}

void WbHingeJoint::applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) {
  const WbJointParameters *const p = parameters();
  const double brakingDampingConstant = brake() ? brake()->getBrakingDampingConstant() : 0.0;

  if ((p == NULL && brakingDampingConstant == 0.0) || (body == NULL && parentBody == NULL)) {
    if (mSpringAndDamperMotor) {
      dJointDestroy(mSpringAndDamperMotor);
      mSpringAndDamperMotor = NULL;
    }
    return;
  }

  assert((body || parentBody) && (p || brake()));

  double d = brakingDampingConstant;
  double s = 0.0;
  if (p) {
    d += p->dampingConstant();
    s += p->springConstant();
  }

  if (s == 0.0 && d == 0.0) {
    if (mSpringAndDamperMotor) {
      dJointDestroy(mSpringAndDamperMotor);
      mSpringAndDamperMotor = NULL;
    }
    return;
  }

  // Handles scale
  const double scale = upperTransform()->absoluteScale().x();
  double s4 = scale * scale;
  s4 *= scale;
  s *= s4;
  d *= s4;

  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  double cfm, erp;
  WbOdeUtilities::convertSpringAndDampingConstants(s, d, wi->basicTimeStep() * 0.001, cfm, erp);

  const dWorldID world = parentBody ? dBodyGetWorld(parentBody) : dBodyGetWorld(body);
  if (mSpringAndDamperMotor == NULL)
    mSpringAndDamperMotor = dJointCreateAMotor(world, 0);

  dJointAttach(mSpringAndDamperMotor, parentBody, body);

  dJointSetAMotorNumAxes(mSpringAndDamperMotor, 1);
  dJointSetAMotorMode(mSpringAndDamperMotor, dAMotorUser);

  // Axis setting
  const WbMatrix4 &m4 = upperTransform()->matrix();
  WbVector3 a = m4.sub3x3MatrixDot(axis());
  if (mIsReverseJoint)
    a = -a;
  dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, a.x(), a.y(), a.z());

  // Stops
  const double clamped = WbMathsUtilities::normalizeAngle(mOdePositionOffset);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop, clamped);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop, clamped);

  // Spring and damper through LCP
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM, cfm);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP, erp);

  // Initial angle
  dJointSetAMotorAngle(mSpringAndDamperMotor, 0, 0.0);
}

void WbHingeJoint::applyToOdeSuspension() {
  // suspension along the suspension axis
  const WbHingeJointParameters *const hp = hingeJointParameters();
  if (hp == NULL)
    return;

  const WbSolid *const solid = solidEndPoint();
  double s2 = solid == NULL ? 1.0 : solid->absoluteScale().x();
  s2 *= s2;
  const double s = s2 * hp->suspensionSpringConstant();
  const double d = s2 * hp->suspensionDampingConstant();
  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  double erp = 0.2, cfm = 0.0;
  if (s == 0.0 && d == 0.0) {
    erp = wi->erp();
    cfm = wi->cfm();
  } else
    WbOdeUtilities::convertSpringAndDampingConstants(s, d, wi->basicTimeStep() * 0.001, cfm, erp);

  if (nodeType() == WB_NODE_HINGE_2_JOINT) {
    dJointSetHinge2Param(mJoint, dParamSuspensionERP, erp);
    dJointSetHinge2Param(mJoint, dParamSuspensionCFM, cfm);
  } else {
    dJointSetHingeParam(mJoint, dParamSuspensionERP, erp);
    dJointSetHingeParam(mJoint, dParamSuspensionCFM, cfm);
  }
  applyToOdeSuspensionAxis();
}

void WbHingeJoint::prePhysicsStep(double ms) {
  assert(solidEndPoint());
  WbRotationalMotor *const rm = rotationalMotor();
  WbJointParameters *const p = parameters();
  if (isEnabled()) {
    if (rm && rm->userControl()) {
      // user-defined torque
      const double torque = rm->rawInput();
      dJointAddHingeTorque(mJoint, mIsReverseJoint ? torque : -torque);
      if (rm->hasMuscles())
        // force is directly applied to the bodies and not included in joint motor feedback
        emit updateMuscleStretch(torque / rm->maxForceOrTorque(), false);
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
    // eventually add spring and damping forces
    if (mSpringAndDamperMotor) {
      double angle = dJointGetHingeAngle(mJoint);
      if (mIsReverseJoint)
        angle = -angle;
      dJointSetAMotorAngle(mSpringAndDamperMotor, 0, angle);
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
      emit updateMuscleStretch(velocityPercentage, true);
    }
  }
  mTimeStep = ms;
}

void WbHingeJoint::postPhysicsStep() {
  assert(mJoint);
  WbRotationalMotor *const rm = rotationalMotor();
  if (rm && rm->isPIDPositionControl()) {  // if controlling in position we update position using directly the angle feedback
    double angle = dJointGetHingeAngle(mJoint);
    if (!mIsReverseJoint)
      angle = -angle;
    mPosition = WbMathsUtilities::normalizeAngle(angle + mOdePositionOffset, mPosition);
  } else {
    // if not controlling in position we use the angle rate feedback to update position (because at high speed angle feedback is
    // under-estimated)
    double angleRate = dJointGetHingeAngleRate(mJoint);
    if (mIsReverseJoint)
      angleRate = -angleRate;
    mPosition -= angleRate * mTimeStep / 1000.0;
  }
  WbJointParameters *const p = parameters();
  if (p)
    p->setPositionFromOde(mPosition);

  if (isEnabled() && rm && rm->hasMuscles() && !rm->userControl())
    // dynamic position or velocity control
    emit updateMuscleStretch(rm->computeFeedback() / rm->maxForceOrTorque(), false);
}

void WbHingeJoint::updatePosition() {
  // Update triggered by an artificial move, i.e. a move caused by the user or a Supervisor
  const WbJointParameters *const p = parameters();
  assert(p);
  if (solidReference() == NULL && solidEndPoint())
    updatePosition(p->position());

  emit updateMuscleStretch(0.0, true);
}

void WbHingeJoint::updatePosition(double position) {
  WbSolid *const s = solidEndPoint();
  assert(s);
  // called after an artificial move
  mPosition = position;
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

// Updates

void WbHingeJoint::updateParameters() {
  WbJoint::updateParameters();
  const WbHingeJointParameters *const p = hingeJointParameters();
  if (p) {
    connect(p, &WbHingeJointParameters::anchorChanged, this, &WbHingeJoint::updateAnchor, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::suspensionChanged, this, &WbHingeJoint::updateSuspension, Qt::UniqueConnection);
  }
}

void WbHingeJoint::updateSuspension() {
  if (isEnabled())
    applyToOdeSuspension();
}

void WbHingeJoint::updateMinAndMaxStop(double min, double max) {
  const WbJointParameters *const p = dynamic_cast<WbJointParameters *>(sender());
  if (min <= -M_PI)
    p->warn(tr("HingeJoint 'minStop' must be greater than -pi to be effective."));

  if (max >= M_PI)
    p->warn(tr("HingeJoint 'maxStop' must be less than pi to be effective."));

  WbRotationalMotor *const rm = rotationalMotor();
  if (rm) {
    const double minPos = rm->minPosition();
    const double maxPos = rm->maxPosition();
    if (min != max && minPos != maxPos) {
      if (minPos < min)
        p->warn(tr("HingeJoint 'minStop' must be less or equal to RotationalMotor 'minPosition'."));

      if (maxPos > max)
        p->warn(tr("HingeJoint 'maxStop' must be greater or equal to RotationalMotor 'maxPosition'."));
    }
  }

  if (mJoint)
    applyToOdeMinAndMaxStop();
}

void WbHingeJoint::updateAnchor() {
  // update the current endPoint pose based on the new anchor value
  // but do not modify the initial endPoint pose
  updatePosition();

  if (mJoint)
    applyToOdeAnchor();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();
}

WbVector3 WbHingeJoint::axis() const {
  static const WbVector3 DEFAULT_AXIS(1.0, 0.0, 0.0);
  const WbJointParameters *const p = parameters();
  return p ? p->axis() : DEFAULT_AXIS;
}

WbVector3 WbHingeJoint::anchor() const {
  const WbHingeJointParameters *const p = hingeJointParameters();
  return p ? p->anchor() : WbBasicJoint::anchor();
}

void WbHingeJoint::updateOdeWorldCoordinates() {
  if (!mJoint || !isPostFinalizedCalled())
    return;

  WbHingeJointParameters *p = hingeJointParameters();
  if (p && (p->suspensionSpringConstant() != 0.0 || p->suspensionDampingConstant() != 0.0))
    // remove suspension effect by resetting the endPoint solid position
    updatePosition(mPosition);

  applyToOdeAxis();
  applyToOdeAnchor();
  if (p)
    applyToOdeSuspensionAxis();
}
