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
//#include "WbJointParameters.hpp"
#include "WbBoundingSphere.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeUtilities.hpp"
#include "WbPositionSensor.hpp"
#include "WbRobot.hpp"
#include "WbRotationalMotor.hpp"
#include "WbSlot.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <ode/ode.h>
#include <wren/config.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

// Constructors

void WbTransmissionJoint::init() {
  mJoint2 = NULL;
  mParameters2 = findSFNode("jointParameters2");
  mStartPoint = findSFNode("startPoint");
  mBacklash = findSFDouble("backlash");
  mMultiplier = findSFDouble("multiplier");
  mGearType = UNDEFINED;

  mIsStartPointPositionChangedByJoint = false;

  // spring and dampingConstant
  mSpringAndDampingConstantsAxis1On = false;
  mSpringAndDampingConstantsAxis2On = false;

  // hidden field
  mPosition2 = findSFDouble("position2")->value();
  mOdePositionOffset2 = mPosition2;
  mInitialPosition2 = mPosition2;
}

WbTransmissionJoint::WbTransmissionJoint(const QString &modelName, WbTokenizer *tokenizer) :
  WbHingeJoint(modelName, tokenizer) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(WbTokenizer *tokenizer) : WbHingeJoint("TransmissionJoint", tokenizer) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(const WbTransmissionJoint &other) : WbHingeJoint(other) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(const WbNode &other) : WbHingeJoint(other) {
  init();
}

WbTransmissionJoint::~WbTransmissionJoint() {
}

void WbTransmissionJoint::preFinalize() {
  WbBaseNode::preFinalize();
  // set endPoint initial position
  updateParameters();
  updateEndPointZeroTranslationAndRotation();

  WbBaseNode *const p = dynamic_cast<WbBaseNode *>(mParameters->value());
  WbBaseNode *const e = dynamic_cast<WbBaseNode *>(mEndPoint->value());
  if (p && !p->isPreFinalizedCalled())
    p->preFinalize();
  if (e && !e->isPreFinalizedCalled())
    e->preFinalize();

  mInitialPosition = mPosition;

  WbBaseNode *const p2 = dynamic_cast<WbBaseNode *>(mParameters2->value());
  if (p2 && !p2->isPreFinalizedCalled())
    p2->preFinalize();

  updateParameters();
  updateParameters2();

  mInitialPosition2 = mPosition2;
}

void WbTransmissionJoint::postFinalize() {
  WbHingeJoint::postFinalize();

  WbBaseNode *const p2 = dynamic_cast<WbBaseNode *>(mParameters2->value());
  if (p2 && !p2->isPostFinalizedCalled())
    p2->postFinalize();

  connect(mParameters2, &WbSFNode::changed, this, &WbTransmissionJoint::updateParameters);
  connect(mBacklash, &WbSFDouble::changed, this, &WbTransmissionJoint::updateBacklash);
  connect(mMultiplier, &WbSFDouble::changed, this, &WbTransmissionJoint::updateMultiplier);
}

// This method is overritten with the creation of the startPoint instead of
// the endPoint as it is in parents of this class. The reason is to maintain
// consistency. (axis2, anchor2, ...)
bool WbTransmissionJoint::setJoint() {
  if (!WbBasicJoint::setJoint())
    return false;

  if (mJoint == NULL)
    mJoint = dJointCreateHinge(WbOdeContext::instance()->world(), 0);

  const WbSolid *const s = solidStartPoint();
  dBodyID body = s ? s->body() : NULL;
  dBodyID parentBody = upperSolid()->bodyMerger();
  if (body && parentBody)
    setOdeJoint(body, parentBody);
  else {
    parsingWarn(tr("TransmissionJoint nodes can only connect Solid nodes that have a Physics node."));
    return false;
  }

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  return true;
}

bool WbTransmissionJoint::setJoint2() {
  if (!WbBasicJoint::setJoint())
    return false;

  if (mJoint2 == NULL)
    mJoint2 = dJointCreateHinge(WbOdeContext::instance()->world(), 0);

  const WbSolid *const s = solidEndPoint();
  dBodyID body = s ? s->body() : NULL;
  dBodyID parentBody = upperSolid()->bodyMerger();
  if (body && parentBody)
    setOdeJoint(body, parentBody);
  else {
    parsingWarn(tr("TransmissionJoint nodes can only connect Solid nodes that have a Physics node."));
    return false;
  }

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  return true;
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

void WbTransmissionJoint::setPosition(double position, int index) {
  if (index == 1) {
    mPosition = position;
    mOdePositionOffset = position;
    WbHingeJointParameters *const p = hingeJointParameters();
    if (p)
      p->setPosition(mPosition);

    WbMotor *const m = motor();
    if (m)
      m->setTargetPosition(position);
    return;
  }

  assert(index == 2);

  mPosition2 = position;
  mOdePositionOffset2 = position;
  WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (p2)
    p2->setPosition(mPosition2);
}

bool WbTransmissionJoint::resetJointPositions() {
  mOdePositionOffset2 = 0.0;
  return WbJoint::resetJointPositions();
}

void WbTransmissionJoint::updateOdePositionOffset() {
  double newValue = position();
  if (mOdePositionOffset == newValue)
    return;

  mOdePositionOffset = newValue;
}

void WbTransmissionJoint::updateOdePositionOffset2() {
  double newValue = position();
  if (mOdePositionOffset2 == newValue)
    return;

  mOdePositionOffset2 = newValue;
}

void WbTransmissionJoint::applyToOdeAxis() {
  assert(mJoint);

  updateOdePositionOffset();

  // compute orientation of rotation axis
  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &a1 = m4.sub3x3MatrixDot(axis());
  dJointSetHinge2Axis1(mJoint, a1.x(), a1.y(), a1.z());
}

void WbTransmissionJoint::applyToOdeAxis2() {
  assert(mJoint2);

  updateOdePositionOffset2();

  // compute orientation of rotation axis
  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &a2 = m4.sub3x3MatrixDot(axis());
  dJointSetHinge2Axis1(mJoint, a2.x(), a2.y(), a2.z());
}

void WbTransmissionJoint::applyToOdeAnchor2() {
  assert(mJoint2);

  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &t = m4 * anchor2();
  if (nodeType() == WB_NODE_HINGE_2_JOINT)
    dJointSetHinge2Anchor(mJoint, t.x(), t.y(), t.z());
}

void WbTransmissionJoint::applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) {
  const WbHingeJointParameters *const p = hingeJointParameters();
  const WbHingeJointParameters *const p2 = hingeJointParameters2();

  const double brakingDampingConstant = brake() ? brake()->getBrakingDampingConstant() : 0.0;

  if ((p == NULL && p2 == NULL && brakingDampingConstant == 0.0) || (body == NULL && parentBody == NULL)) {
    if (mSpringAndDamperMotor) {
      dJointDestroy(mSpringAndDamperMotor);
      mSpringAndDamperMotor = NULL;
    }
    return;
  }
  assert((body || parentBody) && (p || p2 || brake()));

  double s = p ? p->springConstant() : 0.0;
  double d = p ? p->dampingConstant() : 0.0;
  double s2 = p2 ? p2->springConstant() : 0.0;
  double d2 = p2 ? p2->dampingConstant() : 0.0;

  d += brakingDampingConstant;

  mSpringAndDampingConstantsAxis1On = s != 0.0 || d != 0.0;
  mSpringAndDampingConstantsAxis2On = s2 != 0.0 || d2 != 0.0;

  if (!mSpringAndDampingConstantsAxis1On && !mSpringAndDampingConstantsAxis2On) {
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
  s2 *= s4;
  d2 *= s4;

  double cfm, erp, cfm2, erp2;
  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  const double t = wi->basicTimeStep() * 0.001;
  WbOdeUtilities::convertSpringAndDampingConstants(s, d, t, cfm, erp);
  WbOdeUtilities::convertSpringAndDampingConstants(s2, d2, t, cfm2, erp2);

  const dWorldID world = parentBody ? dBodyGetWorld(parentBody) : dBodyGetWorld(body);
  if (mSpringAndDamperMotor == NULL)
    mSpringAndDamperMotor = dJointCreateAMotor(world, 0);

  dJointAttach(mSpringAndDamperMotor, parentBody, body);

  const bool bothAxes = mSpringAndDampingConstantsAxis1On && mSpringAndDampingConstantsAxis2On;
  const int numberOfAxes = bothAxes ? 2 : 1;

  dJointSetAMotorNumAxes(mSpringAndDamperMotor, numberOfAxes);
  dJointSetAMotorMode(mSpringAndDamperMotor, dAMotorUser);

  // Axis dependent settings
  const WbMatrix4 &m4 = upperTransform()->matrix();
  if (mSpringAndDampingConstantsAxis1On) {
    const double clamped = WbMathsUtilities::normalizeAngle(mOdePositionOffset);
    const WbVector3 &a1 = m4.sub3x3MatrixDot(axis());
    dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, a1.x(), a1.y(), a1.z());
    dJointSetAMotorAngle(mSpringAndDamperMotor, 0, 0.0);
    dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop, clamped);
    dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop, clamped);
    dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM, cfm);
    dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP, erp);
  }

  if (mSpringAndDampingConstantsAxis2On) {
    const double clamped2 = WbMathsUtilities::normalizeAngle(mOdePositionOffset2);
    const WbVector3 &a2 = m4.sub3x3MatrixDot(axis2());
    if (bothAxes) {  // axes 0 and 1 of the AMotorAngle are enabled
      dJointSetAMotorAxis(mSpringAndDamperMotor, 1, 1, a2.x(), a2.y(), a2.z());
      dJointSetAMotorAngle(mSpringAndDamperMotor, 1, 0.0);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop2, clamped2);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop2, clamped2);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM2, cfm2);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP2, erp2);
    } else {  // only axis 0 of the AMotorAngle is enabled
      dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, a2.x(), a2.y(), a2.z());
      dJointSetAMotorAngle(mSpringAndDamperMotor, 0, 0.0);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop, clamped2);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop, clamped2);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM, cfm2);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP, erp2);
    }
  }
}

void WbTransmissionJoint::prePhysicsStep(double ms) {
  assert(solidEndPoint());
  WbRotationalMotor *const rm = rotationalMotor();
  WbHingeJointParameters *const p = hingeJointParameters();

  if (isEnabled()) {
    const double s = upperTransform()->absoluteScale().x();
    double s5 = s * s;
    s5 *= s5 * s;

    if (rm && rm->userControl()) {
      // user-defined torque
      dJointAddHinge2Torques(mJoint, -rm->rawInput(), 0.0);
      if (rm->hasMuscles())
        // force is directly applied to the bodies and not included in joint motor feedback
        emit updateMuscleStretch(rm->rawInput() / rm->maxForceOrTorque(), false, 1);
    } else {
      // ODE motor torque (user velocity/position control)
      const double currentVelocity = rm ? rm->computeCurrentDynamicVelocity(ms, mPosition) : 0.0;
      const double fMax = qMax(p ? p->staticFriction() : 0.0, rm ? rm->torque() : 0.0);
      dJointSetHinge2Param(mJoint, dParamFMax, s5 * fMax);
      dJointSetHinge2Param(mJoint, dParamVel, currentVelocity);
    }

    // eventually add spring and damping forces
    if (mSpringAndDamperMotor) {
      if (mSpringAndDampingConstantsAxis1On && mSpringAndDampingConstantsAxis2On) {
        // axes 0 and 1 of the AMotorAngle are enabled
        dJointSetAMotorAngle(mSpringAndDamperMotor, 0, dJointGetHinge2Angle1(mJoint));
        dJointSetAMotorAngle(mSpringAndDamperMotor, 1, -dJointGetHinge2Angle2(mJoint));
      } else if (mSpringAndDampingConstantsAxis1On) {
        // only axis 0 of the AMotorAngle is enabled
        dJointSetAMotorAngle(mSpringAndDamperMotor, 0, dJointGetHinge2Angle1(mJoint));
      } else
        dJointSetAMotorAngle(mSpringAndDamperMotor, 0, -dJointGetHinge2Angle2(mJoint));
    }
  } else {
    const bool run1 = rm && rm->runKinematicControl(ms, mPosition);
    if (run1) {
      if (p)
        p->setPosition(mPosition);
      if (rm->hasMuscles()) {
        double velocityPercentage = rm->currentVelocity() / rm->maxVelocity();
        if (rm->kinematicVelocitySign() == -1)
          velocityPercentage = -velocityPercentage;
        emit updateMuscleStretch(velocityPercentage, true, 1);
      }

      // updatePositions(mPosition, mPosition2);
    }
  }
  mTimeStep = ms;
}

void WbTransmissionJoint::postPhysicsStep() {
  assert(mJoint);
  WbRotationalMotor *const rm = rotationalMotor();
  if (rm && rm->isPIDPositionControl()) {
    // if controlling in position we update position using directly the angle feedback
    mPosition = WbMathsUtilities::normalizeAngle(-dJointGetHinge2Angle1(mJoint) + mOdePositionOffset, mPosition);
  } else {
    // if not controlling in position we use the angle rate feedback to update position (because at high speed angle feedback is
    // under-estimated)
    mPosition -= dJointGetHinge2Angle1Rate(mJoint) * mTimeStep / 1000.0;
  }
  WbHingeJointParameters *const p = hingeJointParameters();
  if (p)
    p->setPositionFromOde(mPosition);
  if (isEnabled() && rm && rm->hasMuscles() && !rm->userControl())
    // dynamic position or velocity control
    emit updateMuscleStretch(rm->computeFeedback() / rm->maxForceOrTorque(), false, 1);

  mPosition2 -= dJointGetHinge2Angle2Rate(mJoint) * mTimeStep / 1000.0;
  WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (p2)
    p2->setPositionFromOde(mPosition2);
}

void WbTransmissionJoint::reset() {
  WbJoint::reset();

  WbNode *const p = mParameters2->value();
  if (p)
    p->reset();

  setPosition(mInitialPosition2, 2);
}

void WbTransmissionJoint::resetPhysics() {
  WbJoint::resetPhysics();
}

void WbTransmissionJoint::save() {
  WbJoint::save();

  WbNode *const p = mParameters2->value();
  if (p)
    p->save();

  mInitialPosition2 = mPosition2;
}

void WbTransmissionJoint::updateStartPoint() {
  WbSolidReference *const r = solidReference();
  if (r)
    r->updateName();

  WbSolid *const s = solidStartPoint();
  if (s) {
    connect(s, &WbSolid::positionChangedArtificially, this, &WbTransmissionJoint::updateStartPointPosition,
            Qt::UniqueConnection);
    s->appendJointParent(this);
  }

  updateStartPointPosition();

  if (r)
    connect(r, &WbSolidReference::changed, this, &WbTransmissionJoint::setJoint, Qt::UniqueConnection);

  if (s == NULL || s->isPostFinalizedCalled()) {
    emit startPointChanged(s);
    if (s != NULL && isPostFinalizedCalled())
      WbBoundingSphere::addSubBoundingSphereToParentNode(this);
  } else {
    connect(s, &WbTransmissionJoint::finalizationCompleted, this, &WbTransmissionJoint::startPointChanged);
    connect(s, &WbTransmissionJoint::finalizationCompleted, this, &WbTransmissionJoint::updateBoundingSphere);
  }
}

void WbTransmissionJoint::updateEndPointZeroTranslationAndRotation() {
  if (solidEndPoint() == NULL)
    return;

  WbRotation ir;
  WbVector3 it;
  retrieveEndPointSolidTranslationAndRotation(it, ir);

  WbQuaternion qp;
  if (WbMathsUtilities::isZeroAngle(mPosition) && WbMathsUtilities::isZeroAngle(mPosition2))
    mEndPointZeroRotation = ir;  // Keeps track of the original axis if the angle is zero as it defines the second DoF axis
  else {
    const WbQuaternion q(axis(), -mPosition);
    const WbQuaternion q2(axis2(), -mPosition2);
    qp = q2 * q;
    const WbQuaternion &iq = ir.toQuaternion();
    WbQuaternion qr = qp * iq;
    qr.normalize();
    mEndPointZeroRotation = WbRotation(qr);
  }
  const WbVector3 &a = anchor();
  const WbVector3 t(it - a);
  mEndPointZeroTranslation = qp * t + a;
}

void WbTransmissionJoint::updateStartPointZeroTranslationAndRotation() {
  if (solidStartPoint() == NULL)
    return;

  WbRotation ir;
  WbVector3 it;
  retrieveStartPointSolidTranslationAndRotation(it, ir);

  WbQuaternion qp;
  if (WbMathsUtilities::isZeroAngle(mPosition) && WbMathsUtilities::isZeroAngle(mPosition2))
    mEndPointZeroRotation = ir;  // Keeps track of the original axis if the angle is zero as it defines the second DoF axis
  else {
    const WbQuaternion q(axis(), -mPosition);
    const WbQuaternion q2(axis2(), -mPosition2);
    qp = q2 * q;
    const WbQuaternion &iq = ir.toQuaternion();
    WbQuaternion qr = qp * iq;
    qr.normalize();
    mStartPointZeroRotation = WbRotation(qr);
  }
  const WbVector3 &a = anchor();
  const WbVector3 t(it - a);
  mStartPointZeroTranslation = qp * t + a;
}

void WbTransmissionJoint::updatePosition() {
  // Update triggered by an artificial move, i.e. a move caused by the user or a Supervisor
  const WbHingeJointParameters *const p = hingeJointParameters();

  if (solidReference() == NULL && solidEndPoint())
    updatePosition(p ? p->position() : mPosition);
  emit updateMuscleStretch(0.0, true, 1);
}

void WbTransmissionJoint::updatePosition2() {
  // Update triggered by an artificial move, i.e. a move caused by the user or a Supervisor
  const WbHingeJointParameters *const p2 = hingeJointParameters();

  if (solidReference() == NULL && solidStartPoint())
    updatePosition2(p2 ? p2->position() : mPosition2);
  emit updateMuscleStretch(0.0, true, 2);
}

void WbTransmissionJoint::updatePosition(double position) {
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

void WbTransmissionJoint::updatePosition2(double position2) {
  WbSolid *const s = solidStartPoint();
  assert(s);
  // called after an artificial move
  mPosition2 = position2;
  WbMotor *m = motor();
  if (m && !m->isConfigureDone())
    m->setTargetPosition(position2);
  WbVector3 translation;
  WbRotation rotation;
  computeStartPointSolidPositionFromParameters(translation, rotation);
  if (!translation.almostEquals(s->translation()) || !rotation.almostEquals(s->rotation())) {
    mIsStartPointPositionChangedByJoint = true;
    s->setTranslationAndRotation(translation, rotation);
    s->resetPhysics();
    mIsStartPointPositionChangedByJoint = false;
  }
}

void WbTransmissionJoint::computeStartPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const {
  const WbVector3 &ax2 = axis2().normalized();
  const WbQuaternion q(ax2, mPosition2);
  const WbQuaternion iq(mStartPointZeroRotation.toQuaternion());
  WbQuaternion qp(q * iq);
  if (qp.w() != 1.0)
    qp.normalize();
  rotation.fromQuaternion(qp);
  if (rotation.angle() == 0.0)
    rotation = WbRotation(ax2.x(), ax2.y(), ax2.z(), 0.0);
  const WbVector3 &a = anchor2();
  translation = q * (mStartPointZeroTranslation - a) + a;
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

  inferGearType();
}

void WbTransmissionJoint::inferGearType() {
  mGearType = UNDEFINED;
  const bool isCodirectional = axis().normalized().almostEquals(axis2().normalized());
  if (mMultiplier->value() < 0.0 && isCodirectional)
    mGearType = CLASSIC_GEAR;
  else if (mMultiplier->value() > 0.0 && isCodirectional)
    mGearType = CHAIN_DRIVE;
  else {
    // determine if they intersect
    const bool isCoplanar = fabs(axis().cross(axis2()).dot(anchor() - anchor2())) < 1e-10;
    const bool isParallel = fabs(axis().cross(axis2()).length2()) < 1e-10;
    if (isCoplanar && !isParallel)
      mGearType = BEVEL_GEAR;
  }

  switch (mGearType) {
    case CLASSIC_GEAR:
      printf("geartype = CLASSIC GEAR\n");
      break;
    case CHAIN_DRIVE:
      printf("geartype = CHAIN DRIVE\n");
      break;
    case BEVEL_GEAR:
      printf("geartype = BEVEL GEAR\n");
      break;
    default:
      printf("geartype = UNDEFINED\n");
  }
}

// Update methods

void WbTransmissionJoint::updateParameters() {
  WbJoint::updateParameters();
  const WbHingeJointParameters *const p = hingeJointParameters();
  if (p) {
    connect(p, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor, Qt::UniqueConnection);
    connect(p, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
  }
}

void WbTransmissionJoint::updateParameters2() {
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (p2) {
    mOdePositionOffset2 = p2->position();
    mPosition2 = mOdePositionOffset2;
    connect(p2, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis2, Qt::UniqueConnection);
    connect(p2, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor2, Qt::UniqueConnection);
    connect(p2, SIGNAL(positionChanged()), this, SLOT(updatePosition2()), Qt::UniqueConnection);
  }
}

WbHingeJointParameters *WbTransmissionJoint::hingeJointParameters2() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters2->value());
}

WbVector3 WbTransmissionJoint::axis2() const {
  static const WbVector3 DEFAULT_AXIS_2(1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  return p2 ? p2->axis() : DEFAULT_AXIS_2;
}

WbVector3 WbTransmissionJoint::anchor2() const {
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  return p2 ? p2->anchor() : WbBasicJoint::anchor();
}

void WbTransmissionJoint::updateAxis() {
  WbHingeJoint::updateAxis();
  inferGearType();
}

void WbTransmissionJoint::updateAxis2() {
  WbHingeJoint::updateAxis();
  inferGearType();
}

void WbTransmissionJoint::updateAnchor() {
  WbHingeJoint::updateAnchor();
  inferGearType();
}

void WbTransmissionJoint::updateAnchor2() {
  updatePosition2();

  if (mJoint2)
    applyToOdeAnchor2();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  inferGearType();
}

QVector<WbLogicalDevice *> WbTransmissionJoint::devices() const {
  QVector<WbLogicalDevice *> devices;
  int i = 0;
  for (i = 0; i < devicesNumber(); ++i)
    devices.append(device(i));

  return devices;
}

void WbTransmissionJoint::updateBoundingSphere(WbBaseNode *subNode) {
  disconnect(subNode, &WbBaseNode::finalizationCompleted, this, &WbTransmissionJoint::updateBoundingSphere);
  WbBoundingSphere::addSubBoundingSphereToParentNode(this);
}

void WbTransmissionJoint::updateStartPointPosition() {
  if (mIsStartPointPositionChangedByJoint)
    return;

  WbSolid *const s = solidStartPoint();
  if (s)
    updateStartPointZeroTranslationAndRotation();

  if (WbBaseNode::areOdeObjectsCreated())
    setJoint();  // here setjoint1 ?
}

void WbTransmissionJoint::setSolidStartPoint(WbSolid *solid) {
  mStartPoint->removeValue();
  mStartPoint->setValue(solid);
  updateStartPoint();
}

void WbTransmissionJoint::setSolidStartPoint(WbSolidReference *solid) {
  mStartPoint->removeValue();
  mStartPoint->setValue(solid);
  updateStartPoint();
}

void WbTransmissionJoint::setSolidStartPoint(WbSlot *slot) {
  mStartPoint->removeValue();
  mStartPoint->setValue(slot);
  updateStartPoint();
}

WbSolid *WbTransmissionJoint::solidStartPoint() const {
  WbSlot *slot = dynamic_cast<WbSlot *>(mStartPoint->value());
  if (slot) {
    WbSlot *childrenSlot = slot->slotEndPoint();
    if (childrenSlot) {
      WbSolid *solid = childrenSlot->solidEndPoint();
      if (solid)
        return solid;

      WbSolidReference *solidReference = childrenSlot->solidReferenceEndPoint();
      if (solidReference)
        return solidReference->solid();
    }
  } else {
    WbSolid *solid = dynamic_cast<WbSolid *>(mStartPoint->value());
    if (solid)
      return solid;

    const WbSolidReference *const solidReference = dynamic_cast<WbSolidReference *>(mStartPoint->value());
    if (solidReference)
      return solidReference->solid();
  }

  return NULL;
}

WbSolidReference *WbTransmissionJoint::solidReference() const {
  WbSlot *slot = dynamic_cast<WbSlot *>(mStartPoint->value());
  if (slot) {
    WbSlot *childrenSlot = slot->slotEndPoint();
    if (childrenSlot)
      return childrenSlot->solidReferenceEndPoint();
    else
      return NULL;
  } else
    return dynamic_cast<WbSolidReference *>(mStartPoint->value());
}

void WbTransmissionJoint::retrieveStartPointSolidTranslationAndRotation(WbVector3 &it, WbRotation &ir) const {
  const WbSolid *const s = solidStartPoint();
  assert(s);

  if (solidReference()) {
    const WbTransform *const ut = upperTransform();
    WbMatrix4 m = ut->matrix().pseudoInversed() * s->matrix();
    double scale = 1.0 / ut->absoluteScale().x();
    scale *= scale;
    m *= scale;
    ir = WbRotation(m.extracted3x3Matrix());
    it = m.translation();
  } else {
    ir = s->rotation();
    it = s->translation();
  }
}

//////////
// WREN //
//////////

void WbTransmissionJoint::createWrenObjects() {
  WbJoint::createWrenObjects();
}

void WbTransmissionJoint::updateJointAxisRepresentation() {
  if (!areWrenObjectsInitialized())
    return;

  wr_static_mesh_delete(mMesh);

  const float scaling = 0.5f * wr_config_get_line_scale();

  float vertices[12];
  const WbVector3 &anchorVector = anchor();
  const WbVector3 &axisVector = scaling * axis();

  WbVector3 vertex(anchorVector - axisVector);
  vertex.toFloatArray(vertices);

  vertex = anchorVector + axisVector;
  vertex.toFloatArray(vertices + 3);

  const WbVector3 &axisVector2 = scaling * axis2();
  vertex = anchorVector - axisVector2;
  vertex.toFloatArray(vertices + 6);

  vertex = anchorVector + axisVector2;
  vertex.toFloatArray(vertices + 9);

  mMesh = wr_static_mesh_line_set_new(4, vertices, NULL);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
}

void WbTransmissionJoint::writeExport(WbVrmlWriter &writer) const {
  if (writer.isUrdf() && solidEndPoint()) {
    warn(tr("Exporting 'Hinge2Joint' nodes to URDF is currently not supported"));
    return;
  }
  WbBasicJoint::writeExport(writer);
}
