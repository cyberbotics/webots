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
#include "WbWrenShaders.hpp"

#include <wren/config.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <cassert>

void WbHingeJoint::init() {
  mTransformSuspension = NULL;
  mRenderableSuspension = NULL;
  mMeshSuspension = NULL;
  mMaterialSuspension = NULL;
}

// Constructors

WbHingeJoint::WbHingeJoint(const QString &modelName, WbTokenizer *tokenizer) : WbJoint(modelName, tokenizer) {
  init();
}

WbHingeJoint::WbHingeJoint(WbTokenizer *tokenizer) : WbJoint("HingeJoint", tokenizer) {
  init();
}

WbHingeJoint::WbHingeJoint(const WbHingeJoint &other) : WbJoint(other) {
  init();
}

WbHingeJoint::WbHingeJoint(const WbNode &other) : WbJoint(other) {
  init();
}

WbHingeJoint::~WbHingeJoint() {
  if (areWrenObjectsInitialized()) {
    wr_static_mesh_delete(mMeshSuspension);
    wr_material_delete(mMaterialSuspension);
    wr_node_delete(WR_NODE(mRenderableSuspension));
    wr_node_delete(WR_NODE(mTransformSuspension));
  }
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

void WbHingeJoint::applyToOdeStopErp() {
  assert(mJoint);

  const WbHingeJointParameters *const p = hingeJointParameters();
  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  const double erp = (p && p->stopErp() != -1) ? p->stopErp() : wi->erp();

  if (nodeType() == WB_NODE_HINGE_JOINT)
    dJointSetHingeParam(mJoint, dParamStopERP, erp);
  if (nodeType() == WB_NODE_HINGE_2_JOINT)
    dJointSetHinge2Param(mJoint, dParamStopERP, erp);
}

void WbHingeJoint::applyToOdeStopCfm() {
  assert(mJoint);

  const WbHingeJointParameters *const p = hingeJointParameters();
  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  const double cfm = (p && p->stopCfm() != -1) ? p->stopCfm() : wi->cfm();

  if (nodeType() == WB_NODE_HINGE_JOINT)
    dJointSetHingeParam(mJoint, dParamStopCFM, cfm);
  if (nodeType() == WB_NODE_HINGE_2_JOINT)
    dJointSetHinge2Param(mJoint, dParamStopCFM, cfm);
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
  // cppcheck-suppress knownConditionTrueFalse
  else if (nodeType() == WB_NODE_BALL_JOINT)
    dJointSetBallAnchor(mJoint, t.x(), t.y(), t.z());
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
      emit updateMuscleStretch(velocityPercentage, true, 1);
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
    // if not controlling in position we use the angle rate feedback to update position (because at high speed angle feedback
    // is under-estimated)
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
    emit updateMuscleStretch(rm->computeFeedback() / rm->maxForceOrTorque(), false, 1);
}

void WbHingeJoint::updatePosition() {
  // Update triggered by an artificial move, i.e. a move caused by the user or a Supervisor
  const WbJointParameters *const p = parameters();

  if (solidReference() == NULL && solidEndPoint())
    updatePosition(p ? p->position() : mPosition);

  emit updateMuscleStretch(0.0, true, 1);
}

void WbHingeJoint::updatePosition(double position) {
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

// Updates

void WbHingeJoint::updateParameters() {
  WbJoint::updateParameters();
  const WbHingeJointParameters *const p = hingeJointParameters();
  if (p) {
    connect(p, &WbHingeJointParameters::anchorChanged, this, &WbHingeJoint::updateAnchor, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::suspensionChanged, this, &WbHingeJoint::updateSuspension, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::stopErpChanged, this, &WbHingeJoint::updateStopErp, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::stopCfmChanged, this, &WbHingeJoint::updateStopCfm, Qt::UniqueConnection);
  }
}

void WbHingeJoint::updateSuspension() {
  if (isEnabled())
    applyToOdeSuspension();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateSuspensionAxisRepresentation();
}

void WbHingeJoint::updateMinAndMaxStop(double min, double max) {
  const WbJointParameters *const p = dynamic_cast<WbJointParameters *>(sender());
  if (min <= -M_PI)
    p->parsingWarn(tr("HingeJoint 'minStop' must be greater than -pi to be effective."));

  if (max >= M_PI)
    p->parsingWarn(tr("HingeJoint 'maxStop' must be less than pi to be effective."));

  WbRotationalMotor *const rm = rotationalMotor();
  if (rm) {
    const double minPos = rm->minPosition();
    const double maxPos = rm->maxPosition();
    if (min != max && minPos != maxPos) {
      if (minPos < min)
        p->parsingWarn(tr("HingeJoint 'minStop' must be less or equal to RotationalMotor 'minPosition'."));

      if (maxPos > max)
        p->parsingWarn(tr("HingeJoint 'maxStop' must be greater or equal to RotationalMotor 'maxPosition'."));
    }
  }

  if (mJoint)
    applyToOdeMinAndMaxStop();
}

void WbHingeJoint::updateStopErp() {
  if (mJoint)
    applyToOdeStopErp();
}

void WbHingeJoint::updateStopCfm() {
  if (mJoint)
    applyToOdeStopCfm();
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

void WbHingeJoint::createWrenObjects() {
  WbJoint::createWrenObjects();

  const float color[3] = {0.0f, 0.0f, 0.0f};
  mMaterialSuspension = wr_phong_material_new();
  wr_phong_material_set_color(mMaterialSuspension, color);
  wr_material_set_default_program(mMaterialSuspension, WbWrenShaders::lineSetShader());

  mRenderableSuspension = wr_renderable_new();
  wr_renderable_set_cast_shadows(mRenderableSuspension, false);
  wr_renderable_set_receive_shadows(mRenderableSuspension, false);
  wr_renderable_set_material(mRenderableSuspension, mMaterialSuspension, NULL);
  wr_renderable_set_visibility_flags(mRenderableSuspension, WbWrenRenderingContext::VF_JOINT_AXES);
  wr_renderable_set_drawing_mode(mRenderableSuspension, WR_RENDERABLE_DRAWING_MODE_LINES);

  mTransformSuspension = wr_transform_new();
  wr_node_set_visible(WR_NODE(mTransformSuspension), false);
  wr_transform_attach_child(mTransformSuspension, WR_NODE(mRenderableSuspension));
  wr_transform_attach_child(wrenNode(), WR_NODE(mTransformSuspension));

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    wr_node_set_visible(WR_NODE(mTransformSuspension), true);

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::optionalRenderingChanged, this,
          &WbHingeJoint::updateOptionalRendering);

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this,
          &WbHingeJoint::updateSuspensionAxisRepresentation);

  updateSuspensionAxisRepresentation();
}

void WbHingeJoint::updateOptionalRendering(int option) {
  WbJoint::updateOptionalRendering(option);

  if (option == WbWrenRenderingContext::VF_JOINT_AXES) {
    if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(option)) {
      updateSuspensionAxisRepresentation();
    } else
      wr_node_set_visible(WR_NODE(mTransformSuspension), false);
  }
}

void WbHingeJoint::updateSuspensionAxisRepresentation() {
  if (!areWrenObjectsInitialized())
    return;

  const WbHingeJointParameters *const p = hingeJointParameters();

  if (!p)
    return;

  const bool hasSuspensionSpring = p->suspensionSpringConstant() > 0;
  const bool hasSuspensionDamper = p->suspensionDampingConstant() > 0;

  if (!hasSuspensionDamper && !hasSuspensionSpring) {
    wr_node_set_visible(WR_NODE(mTransformSuspension), false);
    return;
  }

  if (!wr_node_is_visible(WR_NODE(mTransformSuspension)))
    wr_node_set_visible(WR_NODE(mTransformSuspension), true);

  wr_static_mesh_delete(mMeshSuspension);

  const WbVector3 &suspensionAxis = p->suspensionAxis();
  const WbVector3 &anchorVector = anchor();

  int nbVertices = 2;  // always have at least the suspension axis
  const int steps = 128;
  const int sides = 16;

  if (hasSuspensionSpring)
    nbVertices += steps * 2;

  if (hasSuspensionDamper)
    nbVertices += (sides * 2) + (sides * 2) + (sides * 2) + (sides * 2);  // bottom & top circle & height

  // TODO: check if works without either spring or damper

  float vertices[nbVertices * 3];
  int offset = 0;
  // create vertices of the coil of the suspension spring
  if (hasSuspensionSpring) {
    const double coilHeight = wr_config_get_line_scale() / steps;
    const double coilRadius = 0.01f * (1 + wr_config_get_line_scale());
    const double revolutions = 8.0f * (2 * M_PI);
    const double position_offset = 0.5f * wr_config_get_line_scale();

    WbVector3 vertex;
    for (int i = 0; i < steps; ++i) {
      vertex = WbVector3(coilRadius * sin(i * (revolutions / steps)), position_offset + coilHeight * i,
                         coilRadius * cos(i * (revolutions / steps)));
      vertex.toFloatArray(vertices + offset);
      offset += 3;

      vertex = WbVector3(coilRadius * sin((i + 1) * (revolutions / steps)), position_offset + coilHeight * (i + 1),
                         coilRadius * cos((i + 1) * (revolutions / steps)));
      vertex.toFloatArray(vertices + offset);
      offset += 3;
    }
  }

  // create vertices of the cylinder of the suspension damper
  if (hasSuspensionDamper) {
    const double cylinderHeight = wr_config_get_line_scale();
    const double cylinderRadius = 0.01f * (1 + wr_config_get_line_scale()) * 0.75;

    const double lower_height = 0.5f * wr_config_get_line_scale() + 0.33 * cylinderHeight;
    const double upper_height = 0.5f * wr_config_get_line_scale() + 0.66 * cylinderHeight;

    WbVector3 vertex;
    for (int i = 0; i < sides; ++i) {
      const double coordinate_x = cylinderRadius * sin(i * 2 * M_PI / sides);
      const double coordinate_z = cylinderRadius * cos(i * 2 * M_PI / sides);

      const double next_coordinate_x = cylinderRadius * sin((i + 1) * 2 * M_PI / sides);
      const double next_coordinate_z = cylinderRadius * cos((i + 1) * 2 * M_PI / sides);

      // bottom circle
      vertex = WbVector3(coordinate_x, lower_height, coordinate_z);
      vertex.toFloatArray(vertices + offset);
      offset += 3;

      vertex = WbVector3(next_coordinate_x, lower_height, next_coordinate_z);
      vertex.toFloatArray(vertices + offset);
      offset += 3;

      // top circle
      vertex = WbVector3(coordinate_x, upper_height, coordinate_z);
      vertex.toFloatArray(vertices + offset);
      offset += 3;

      vertex = WbVector3(next_coordinate_x, upper_height, next_coordinate_z);
      vertex.toFloatArray(vertices + offset);
      offset += 3;

      // connectors between bottom and top circles
      vertex = WbVector3(coordinate_x, lower_height, coordinate_z);
      vertex.toFloatArray(vertices + offset);
      offset += 3;

      vertex = WbVector3(coordinate_x, upper_height, coordinate_z);
      vertex.toFloatArray(vertices + offset);
      offset += 3;

      if (!(i % 2)) {
        // bottom face
        vertex = WbVector3(coordinate_x, lower_height, coordinate_z);
        vertex.toFloatArray(vertices + offset);
        offset += 3;

        vertex = WbVector3(-coordinate_x, lower_height, -coordinate_z);
        vertex.toFloatArray(vertices + offset);
        offset += 3;

        // top face
        vertex = WbVector3(coordinate_x, upper_height, coordinate_z);
        vertex.toFloatArray(vertices + offset);
        offset += 3;

        vertex = WbVector3(-coordinate_x, upper_height, -coordinate_z);
        vertex.toFloatArray(vertices + offset);
        offset += 3;
      }
    }
  }

  // create mesh for suspension axis
  anchorVector.toFloatArray(vertices + offset);
  offset += 3;
  WbVector3 vertex(0, 2 * wr_config_get_line_scale(), 0);  // we orient everything together later
  vertex.toFloatArray(vertices + offset);

  mMeshSuspension = wr_static_mesh_line_set_new(nbVertices, vertices, NULL);

  // orient mesh based on direction of suspension axis
  WbVector3 baseX, baseY, baseZ;
  baseY = suspensionAxis.normalized();
  baseX = fabs(baseY.dot(WbVector3(1, 0, 0))) < 1e-6 ? baseY.cross(WbVector3(1, 0, 0)) : baseY.cross(WbVector3(0, 1, 0));
  baseZ = baseX.cross(baseY).normalized();

  WbRotation rotation(baseX, baseY, baseZ);
  rotation.normalize();

  float rotationArray[4];
  rotation.toFloatArray(rotationArray);

  float tail[6];
  anchorVector.toFloatArray(tail);
  suspensionAxis.toFloatArray(tail + 3);

  wr_transform_set_position(mTransformSuspension, tail);
  wr_transform_set_orientation(mTransformSuspension, rotationArray);

  wr_renderable_set_mesh(mRenderableSuspension, WR_MESH(mMeshSuspension));
}