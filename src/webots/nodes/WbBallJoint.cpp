// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbBallJoint.hpp"
#include "WbBallJointParameters.hpp"
#include "WbMathsUtilities.hpp"
#include "WbPositionSensor.hpp"
#include "WbJointParameters.hpp"
#include "WbBrake.hpp"
#include "WbMotor.hpp"
#include "WbRotationalMotor.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeUtilities.hpp"
#include "WbRobot.hpp"
#include "WbSFNode.hpp"
#include "WbSolid.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <ode/ode.h>

// Constructors

void WbBallJoint::init() {
  mParameters3 = findSFNode("jointParameters3");
  mDevice3 = findMFNode("device3");

  // hidden field
  mPosition3 = findSFDouble("position3")->value();
  mOdePositionOffset3 = mPosition3;
  mInitialPosition3 = mPosition3;
}

WbBallJoint::WbBallJoint(WbTokenizer *tokenizer) : WbHinge2Joint("BallJoint", tokenizer) {
  init();
}

WbBallJoint::WbBallJoint(const WbBallJoint &other) : WbHinge2Joint(other) {
  init();
}

WbBallJoint::WbBallJoint(const WbNode &other) : WbHinge2Joint(other) {
  init();
}

WbBallJoint::~WbBallJoint() {
}

WbAnchorParameter *WbBallJoint::anchorParameter() const {
  return dynamic_cast<WbAnchorParameter *>(mParameters->value());
}

WbBallJointParameters *WbBallJoint::ballJointParameters() const {
  return dynamic_cast<WbBallJointParameters *>(mParameters->value());
}

WbJointParameters *WbBallJoint::parameters3() const {
  return dynamic_cast<WbJointParameters *>(mParameters3->value());
}

WbVector3 WbBallJoint::axis3() const {
  static const WbVector3 DEFAULT_AXIS_3(0.0, 1.0, 0.0);
  const WbJointParameters *const p3 = parameters3();
  return p3 ? p3->axis() : DEFAULT_AXIS_3;
}

WbMotor *WbBallJoint::motor3() const {
  WbMotor *motor = NULL;
  for (int i = 0; i < mDevice3->size(); ++i) {
    motor = dynamic_cast<WbMotor *>(mDevice3->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

WbPositionSensor *WbBallJoint::positionSensor3() const {
  WbPositionSensor *sensor = NULL;
  for (int i = 0; i < mDevice3->size(); ++i) {
    sensor = dynamic_cast<WbPositionSensor *>(mDevice3->item(i));
    if (sensor)
      return sensor;
  }

  return NULL;
}

WbBrake *WbBallJoint::brake3() const {
  WbBrake *brake = NULL;
  for (int i = 0; i < mDevice3->size(); ++i) {
    brake = dynamic_cast<WbBrake *>(mDevice3->item(i));
    if (brake)
      return brake;
  }

  return NULL;
}

WbRotationalMotor *WbBallJoint::rotationalMotor3() const {
  WbRotationalMotor *motor = NULL;
  for (int i = 0; i < mDevice3->size(); ++i) {
    motor = dynamic_cast<WbRotationalMotor *>(mDevice3->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

WbVector3 WbBallJoint::anchor() const {
  static const WbVector3 ZERO(0.0, 0.0, 0.0);
  WbAnchorParameter *const p = anchorParameter();
  return p ? p->anchor() : ZERO;
}

void WbBallJoint::updateParameters() {
  WbHinge2Joint::updateParameters();
  updateParameters3();
}

void WbBallJoint::addDevice3(int index) {
  const WbSolid *const s = upperSolid();
  if (s) {
    WbRobot *const r = s->robot();
    assert(r);
    WbBaseNode *decendant = dynamic_cast<WbBaseNode *>(mDevice3->item(index));
    r->descendantNodeInserted(decendant);
  }
  WbBrake *brake = dynamic_cast<WbBrake *>(mDevice3->item(index));
  if (brake)
    connect(brake, &WbBrake::brakingChanged, this, &WbBallJoint::updateSpringAndDampingConstants, Qt::UniqueConnection);
}

void WbBallJoint::updateParameters3() {
  const WbJointParameters *const p3 = parameters3();
  if (p3) {
    mOdePositionOffset3 = p3->position();
    mPosition3 = mOdePositionOffset3;
    connect(p3, &WbJointParameters::minAndMaxStopChanged, this, &WbBallJoint::updateMinAndMaxStop, Qt::UniqueConnection);
    connect(p3, &WbJointParameters::springAndDampingConstantsChanged, this, &WbBallJoint::updateSpringAndDampingConstants,
            Qt::UniqueConnection);
    connect(p3, &WbJointParameters::axisChanged, this, &WbBallJoint::updateAxis, Qt::UniqueConnection);
    connect(p3, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
  }
}

bool WbBallJoint::setJoint() {
  if (!WbBasicJoint::setJoint())
    return false;

  if (mJoint == NULL)
    mJoint = dJointCreateBall(WbOdeContext::instance()->world(), 0);

  const WbSolid *const s = solidEndPoint();
  setOdeJoint(s ? s->body() : NULL, upperSolid()->bodyMerger());

  return true;
}

void WbBallJoint::preFinalize() {
  WbHinge2Joint::preFinalize();

  WbBallJointParameters *const p = ballJointParameters();
  if (p && !p->isPreFinalizedCalled())
    p->preFinalize();
}

void WbBallJoint::postFinalize() {
  WbHinge2Joint::postFinalize();

  WbBallJointParameters *const p = ballJointParameters();

  if (p && !p->isPostFinalizedCalled())
    p->postFinalize();
}

void WbBallJoint::applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) {
  const WbBallJointParameters *const p = ballJointParameters();
  const WbJointParameters *const p2 = parameters2();
  const WbJointParameters *const p3 = parameters3();

  const double brakingDampingConstant = brake() ? brake()->getBrakingDampingConstant() : 0.0;
  const double brakingDampingConstant2 = brake2() ? brake2()->getBrakingDampingConstant() : 0.0;
  const double brakingDampingConstant3 = brake3() ? brake3()->getBrakingDampingConstant() : 0.0;

  if ((p == NULL && p2 == NULL && p3 == NULL && brakingDampingConstant == 0.0 && brakingDampingConstant2 == 0.0 && brakingDampingConstant3 == 0.0) || (body == NULL && parentBody == NULL)) {
    if (mSpringAndDamperMotor) {
      dJointDestroy(mSpringAndDamperMotor);
      mSpringAndDamperMotor = NULL;
    }
    return;
  }
  assert((body || parentBody) && (p || p2 || p3 || brake() || brake2() || brake3()));

  double s = p ? p->springConstant() : 0.0;
  double d = p ? p->dampingConstant() : 0.0;
  double s2 = p2 ? p2->springConstant() : 0.0;
  double d2 = p2 ? p2->dampingConstant() : 0.0;
  double s3 = p3 ? p3->springConstant() : 0.0;
  double d3 = p3 ? p3->dampingConstant() : 0.0;

  d += brakingDampingConstant;
  d2 += brakingDampingConstant2;
  d3 += brakingDampingConstant3;

  mSpringAndDampingConstantsAxis1On = s != 0.0 || d != 0.0;
  mSpringAndDampingConstantsAxis2On = s2 != 0.0 || d2 != 0.0;
  mSpringAndDampingConstantsAxis3On = s3 != 0.0 || d3 != 0.0;

  if (!mSpringAndDampingConstantsAxis1On && !mSpringAndDampingConstantsAxis2On && !mSpringAndDampingConstantsAxis3On) {
    if (mSpringAndDamperMotor) {
      dJointDestroy(mSpringAndDamperMotor);
      mSpringAndDamperMotor = NULL;
    }
    return;
  }

  // Handles scale
  const WbTransform *const ut = upperTransform();
  const double scale = ut->absoluteScale().x();
  double s4 = scale * scale;
  s4 *= scale;
  s *= s4;
  d *= s4;

  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  double cfm, erp, cfm2, erp2, cfm3, erp3;
  WbOdeUtilities::convertSpringAndDampingConstants(s, d, wi->basicTimeStep() * 0.001, cfm, erp);
  WbOdeUtilities::convertSpringAndDampingConstants(s2, d2, wi->basicTimeStep() * 0.001, cfm2, erp2);
  WbOdeUtilities::convertSpringAndDampingConstants(s3, d3, wi->basicTimeStep() * 0.001, cfm3, erp3);

  const dWorldID world = parentBody ? dBodyGetWorld(parentBody) : dBodyGetWorld(body);
  if (mSpringAndDamperMotor == NULL)
    mSpringAndDamperMotor = dJointCreateAMotor(world, 0);

  dJointAttach(mSpringAndDamperMotor, parentBody, body);

  int numberOfAxes =  0;
  if (mSpringAndDampingConstantsAxis1On)
    numberOfAxes++;
  if (mSpringAndDampingConstantsAxis2On)
    numberOfAxes++;
  if (mSpringAndDampingConstantsAxis3On)
    numberOfAxes++;

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
    if (mSpringAndDampingConstantsAxis1On) {  // axes 0 and 1 of the AMotorAngle are enabled
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

  if (mSpringAndDampingConstantsAxis3On) {
    const double clamped3 = WbMathsUtilities::normalizeAngle(mOdePositionOffset3);
    const WbVector3 &a3 = m4.sub3x3MatrixDot(axis3());
    if (mSpringAndDampingConstantsAxis1On || mSpringAndDampingConstantsAxis1On) {
      if (mSpringAndDampingConstantsAxis1On && mSpringAndDampingConstantsAxis1On) {  // axes 0, 1 and 2 of the AMotorAngle are enabled
        dJointSetAMotorAxis(mSpringAndDamperMotor, 2, 1, a3.x(), a3.y(), a3.z());
        dJointSetAMotorAngle(mSpringAndDamperMotor, 2, 0.0);
        dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop3, clamped3);
        dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop3, clamped3);
        dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM3, cfm3);
        dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP3, erp3);
      } else {  // axes 0 and 1 of the AMotorAngle are enabled
        dJointSetAMotorAxis(mSpringAndDamperMotor, 1, 1, a3.x(), a3.y(), a3.z());
        dJointSetAMotorAngle(mSpringAndDamperMotor, 1, 0.0);
        dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop2, clamped3);
        dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop2, clamped3);
        dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM2, cfm3);
        dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP2, erp3);
      }
    } else {  // only axis 0 of the AMotorAngle is enabled
      dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, a3.x(), a3.y(), a3.z());
      dJointSetAMotorAngle(mSpringAndDamperMotor, 0, 0.0);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop, clamped3);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop, clamped3);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM, cfm3);
      dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP, erp3);
    }
  }
}

void WbBallJoint::applyToOdeAxis() {
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
  // TODO: axis 3
  if (!c.isNull()) {
    /*dJointSetHinge2Axis1(mJoint, a1.x(), a1.y(), a1.z());
    dJointSetHinge2Axis2(mJoint, a2.x(), a2.y(), a2.z());*/
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
    warn(tr("Hinge axes are aligned: using x and z axes instead."));
    /*dJointSetHinge2Axis1(mJoint, 1.0, 0.0, 0.0);
    dJointSetHinge2Axis2(mJoint, 0.0, 0.0, 1.0);
    dJointSetHinge2Axis2(mJoint, 0.0, 1.0, 0.0);*/
    if (mSpringAndDamperMotor) {
      dJointSetAMotorAxis(mSpringAndDamperMotor, 0, 1, 1.0, 0.0, 0.0);
      dJointSetAMotorAxis(mSpringAndDamperMotor, 1, 1, 0.0, 0.0, 1.0);
      dJointSetAMotorAxis(mSpringAndDamperMotor, 1, 1, 0.0, 1.0, 0.0);
    }
  }
}

void WbBallJoint::applyToOdeMinAndMaxStop() {
  assert(mJoint);
  // place hard stops if defined
  const WbJointParameters *const p = parameters();
  double m = p ? p->minStop() : 0.0;
  double M = p ? p->maxStop() : 0.0;
  if (m != M) {
    double min = -M + mOdePositionOffset;
    double max = -m + mOdePositionOffset;
    WbMathsUtilities::clampAngles(min, max);
    dJointSetBallParam(mJoint, dParamLoStop, min);
    dJointSetBallParam(mJoint, dParamHiStop, max);
  }

  const WbJointParameters *const p2 = parameters2();
  m = p2 ? p2->minStop() : 0.0;
  M = p2 ? p2->maxStop() : 0.0;
  if (m != M) {
    double min = -M + mOdePositionOffset2;
    double max = -m + mOdePositionOffset2;
    WbMathsUtilities::clampAngles(min, max);
    dJointSetBallParam(mJoint, dParamLoStop2, min);
    dJointSetBallParam(mJoint, dParamHiStop2, max);
  }

  const WbJointParameters *const p3 = parameters3();
  m = p3 ? p3->minStop() : 0.0;
  M = p3 ? p3->maxStop() : 0.0;
  if (m != M) {
    double min = -M + mOdePositionOffset3;
    double max = -m + mOdePositionOffset3;
    WbMathsUtilities::clampAngles(min, max);
    dJointSetBallParam(mJoint, dParamLoStop3, min);
    dJointSetBallParam(mJoint, dParamHiStop3, max);
  }
}

//////////
// WREN //
//////////

void WbBallJoint::createWrenObjects() {
  WbHinge2Joint::createWrenObjects();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    wr_node_set_visible(WR_NODE(mTransform), true);

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this,
          &WbBallJoint::updateJointAxisRepresentation);

  updateJointAxisRepresentation();
}

void WbBallJoint::updateJointAxisRepresentation() {
  if (!areWrenObjectsInitialized())
    return;

  wr_static_mesh_delete(mMesh);

  float anchorArray[3];
  anchor().toFloatArray(anchorArray);

  const float scaling = 0.5f * wr_config_get_line_scale();
  const float vertices[18] = {
    anchorArray[0] - scaling, anchorArray[1], anchorArray[2],           anchorArray[0] + scaling, anchorArray[1],
    anchorArray[2],           anchorArray[0], anchorArray[1] - scaling, anchorArray[2],           anchorArray[0],
    anchorArray[1] + scaling, anchorArray[2], anchorArray[0],           anchorArray[1],           anchorArray[2] - scaling,
    anchorArray[0],           anchorArray[1], anchorArray[2] + scaling};
  mMesh = wr_static_mesh_line_set_new(6, vertices, NULL);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
}
