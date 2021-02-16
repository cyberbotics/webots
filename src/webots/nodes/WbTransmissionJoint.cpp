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
#include "WbJointParameters.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeUtilities.hpp"
#include "WbPositionSensor.hpp"
#include "WbRobot.hpp"
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
  mParameters2 = findSFNode("jointParameters2");
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
  WbHingeJoint::preFinalize();

  WbBaseNode *const p2 = dynamic_cast<WbBaseNode *>(mParameters2->value());
  if (p2 && !p2->isPreFinalizedCalled())
    p2->preFinalize();

  updateParameters2();

  mInitialPosition2 = mPosition2;
}

void WbTransmissionJoint::postFinalize() {
  WbHingeJoint::postFinalize();

  WbBaseNode *const p2 = dynamic_cast<WbBaseNode *>(mParameters2->value());
  if (p2 && !p2->isPostFinalizedCalled())
    p2->postFinalize();

  connect(mParameters2, &WbSFNode::changed, this, &WbTransmissionJoint::updateParameters);
}

bool WbTransmissionJoint::setJoint() {
  if (!WbBasicJoint::setJoint())
    return false;

  if (mJoint == NULL)
    mJoint = dJointCreateHinge2(WbOdeContext::instance()->world(), 0);

  const WbSolid *const s = solidEndPoint();
  dBodyID body = s ? s->body() : NULL;
  dBodyID parentBody = upperSolid()->bodyMerger();
  if (body && parentBody)
    setOdeJoint(body, parentBody);
  else {
    parsingWarn(tr("Hinge2Joint nodes can only connect Solid nodes that have a Physics node."));
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
    WbJointParameters *const p = parameters();
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
  WbJointParameters *const p2 = parameters2();
  if (p2)
    p2->setPosition(mPosition2);
}

bool WbTransmissionJoint::resetJointPositions() {
  mOdePositionOffset2 = 0.0;
  return WbJoint::resetJointPositions();
}

void WbTransmissionJoint::updateOdePositionOffset() {
  WbJoint::updateOdePositionOffset();
  mOdePositionOffset2 = position(2);
}

void WbTransmissionJoint::applyToOdeAxis() {
  assert(mJoint);

  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  // compute orientation of rotation axis
  const WbVector3 &a1 = m4.sub3x3MatrixDot(axis());
  WbVector3 a2;
  if (mPosition == 0.0)
    a2 = m4.sub3x3MatrixDot(axis2());
  else {
    // compute axis2 based on axis1 rotation
    WbMatrix3 a1Matrix(axis(), mPosition);
    a2 = (m4.extracted3x3Matrix() * a1Matrix) * axis2();
  }
  const WbVector3 &c = a1.cross(a2);
  if (!c.isNull()) {
    dJointSetHinge2Axis1(mJoint, a1.x(), a1.y(), a1.z());
    dJointSetHinge2Axis2(mJoint, a2.x(), a2.y(), a2.z());
    if (mSpringAndDamperMotor) {
      if (mSpringAndDampingConstantsAxis1On && mSpringAndDampingConstantsAxis2On) {
        // axes 0 and 1 of the AMotorAngle are enabled
        dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, a1.x(), a1.y(), a1.z());
        dJointSetAMotorAxis(mSpringAndDamperMotor, 1, 1, a2.x(), a2.y(), a2.z());
      } else if (mSpringAndDampingConstantsAxis1On) {
        // only axis 0 of the AMotorAngle is enabled
        dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, a1.x(), a1.y(), a1.z());
      } else
        dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, a2.x(), a2.y(), a2.z());
    }
  } else {
    parsingWarn(tr("Hinge axes are aligned: using x and z axes instead."));
    dJointSetHinge2Axis1(mJoint, 1.0, 0.0, 0.0);
    dJointSetHinge2Axis2(mJoint, 0.0, 0.0, 1.0);
    if (mSpringAndDamperMotor) {
      dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, 1.0, 0.0, 0.0);
      dJointSetAMotorAxis(mSpringAndDamperMotor, 1, 1, 0.0, 0.0, 1.0);
    }
  }
}

void WbTransmissionJoint::applyToOdeMinAndMaxStop() {
  assert(mJoint);
  // place hard stops if defined
  const WbJointParameters *const p = parameters();
  double m = p ? p->minStop() : 0.0;
  double M = p ? p->maxStop() : 0.0;
  if (m != M) {
    double min = -M + mOdePositionOffset;
    double max = -m + mOdePositionOffset;
    WbMathsUtilities::clampAngles(min, max);
    dJointSetHinge2Param(mJoint, dParamLoStop, min);
    dJointSetHinge2Param(mJoint, dParamHiStop, max);
  }

  const WbJointParameters *const p2 = parameters2();
  m = p2 ? p2->minStop() : 0.0;
  M = p2 ? p2->maxStop() : 0.0;
  if (m != M) {
    double min = -M + mOdePositionOffset2;
    double max = -m + mOdePositionOffset2;
    WbMathsUtilities::clampAngles(min, max);
    dJointSetHinge2Param(mJoint, dParamLoStop2, min);
    dJointSetHinge2Param(mJoint, dParamHiStop2, max);
  }
}

void WbTransmissionJoint::applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) {
  const WbJointParameters *const p = parameters();
  const WbJointParameters *const p2 = parameters2();

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
  WbJointParameters *const p = parameters();

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

      updatePositions(mPosition, mPosition2);
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
  WbJointParameters *const p = parameters();
  if (p)
    p->setPositionFromOde(mPosition);
  if (isEnabled() && rm && rm->hasMuscles() && !rm->userControl())
    // dynamic position or velocity control
    emit updateMuscleStretch(rm->computeFeedback() / rm->maxForceOrTorque(), false, 1);

  mPosition2 -= dJointGetHinge2Angle2Rate(mJoint) * mTimeStep / 1000.0;
  WbJointParameters *const p2 = parameters2();
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

void WbTransmissionJoint::computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const {
  WbQuaternion qp;
  const WbQuaternion q(axis(), mPosition);
  const WbQuaternion q2(axis2(), mPosition2);
  WbQuaternion qi = mEndPointZeroRotation.toQuaternion();
  qp = q * q2;
  const WbVector3 &a = anchor();
  const WbVector3 t(mEndPointZeroTranslation - a);
  translation = qp * t + a;
  qp = qp * qi;
  qp.normalize();
  rotation.fromQuaternion(qp);
}

void WbTransmissionJoint::updatePosition() {
  const WbJointParameters *const p = parameters();
  const WbJointParameters *const p2 = parameters2();

  if (solidReference() == NULL && solidEndPoint())
    updatePositions(p ? p->position() : mPosition, p2 ? p2->position() : mPosition2);
  emit updateMuscleStretch(0.0, true, 1);
  emit updateMuscleStretch(0.0, true, 2);
}

void WbTransmissionJoint::updatePositions(double position, double position2) {
  WbSolid *const s = solidEndPoint();
  assert(s);
  // called after an artificial move (user or Supervisor move) or in kinematic mode
  mPosition = position;
  mPosition2 = position2;
  WbMotor *m1 = motor();
  if (m1 && !m1->isConfigureDone())
    m1->setTargetPosition(position);
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

void WbTransmissionJoint::updatePosition(double position) {
  mPosition = position;
  updatePositions(mPosition, mPosition2);
}

void WbTransmissionJoint::updateMinAndMaxStop(double min, double max) {
  const WbJointParameters *const p = dynamic_cast<WbJointParameters *>(sender());

  const WbRotationalMotor *rm = NULL;
  if (p == parameters())
    rm = rotationalMotor();

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

void WbTransmissionJoint::updateParameters() {
  WbHingeJoint::updateParameters();
  updateParameters2();
}

// Update methods

void WbTransmissionJoint::updateParameters2() {
  const WbJointParameters *const p2 = parameters2();
  if (p2) {
    mOdePositionOffset2 = p2->position();
    mPosition2 = mOdePositionOffset2;
    connect(p2, &WbJointParameters::minAndMaxStopChanged, this, &WbTransmissionJoint::updateMinAndMaxStop,
            Qt::UniqueConnection);
    connect(p2, &WbJointParameters::springAndDampingConstantsChanged, this,
            &WbTransmissionJoint::updateSpringAndDampingConstants, Qt::UniqueConnection);
    connect(p2, &WbJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis, Qt::UniqueConnection);
    connect(p2, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
  }
}

WbJointParameters *WbTransmissionJoint::parameters2() const {
  return dynamic_cast<WbJointParameters *>(mParameters2->value());
}

WbVector3 WbTransmissionJoint::axis2() const {
  static const WbVector3 DEFAULT_AXIS_2(0.0, 0.0, 1.0);
  const WbJointParameters *const p2 = parameters2();
  return p2 ? p2->axis() : DEFAULT_AXIS_2;
}

QVector<WbLogicalDevice *> WbTransmissionJoint::devices() const {
  QVector<WbLogicalDevice *> devices;
  int i = 0;
  for (i = 0; i < devicesNumber(); ++i)
    devices.append(device(i));

  return devices;
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
