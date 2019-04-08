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

#include "WbBallJoint.hpp"
#include "WbBallJointParameters.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeUtilities.hpp"
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

WbBallJoint::WbBallJoint(WbTokenizer *tokenizer) : WbBasicJoint("BallJoint", tokenizer) {
}

WbBallJoint::WbBallJoint(const WbBallJoint &other) : WbBasicJoint(other) {
}

WbBallJoint::WbBallJoint(const WbNode &other) : WbBasicJoint(other) {
}

WbBallJoint::~WbBallJoint() {
}

WbAnchorParameter *WbBallJoint::anchorParameter() const {
  return dynamic_cast<WbAnchorParameter *>(mParameters->value());
}

WbBallJointParameters *WbBallJoint::ballJointParameters() const {
  return dynamic_cast<WbBallJointParameters *>(mParameters->value());
}

WbVector3 WbBallJoint::anchor() const {
  static const WbVector3 ZERO(0.0, 0.0, 0.0);
  WbAnchorParameter *const p = anchorParameter();
  return p ? p->anchor() : ZERO;
}

void WbBallJoint::updateParameters() {
  const WbBallJointParameters *const p = ballJointParameters();
  if (p) {
    connect(p, &WbBallJointParameters::anchorChanged, this, &WbBallJoint::updateAnchor, Qt::UniqueConnection);
    connect(p, &WbBallJointParameters::springAndDampingConstantsChanged, this, &WbBallJoint::updateSpringAndDampingConstants,
            Qt::UniqueConnection);
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

void WbBallJoint::setOdeJoint(dBodyID body, dBodyID parentBody) {
  WbBasicJoint::setOdeJoint(body, parentBody);
  applyToOdeAnchor();
}

void WbBallJoint::preFinalize() {
  WbBasicJoint::preFinalize();

  WbBallJointParameters *const p = ballJointParameters();
  if (p && !p->isPreFinalizedCalled())
    p->preFinalize();
}

void WbBallJoint::postFinalize() {
  WbBasicJoint::postFinalize();

  WbBallJointParameters *const p = ballJointParameters();

  if (p && !p->isPostFinalizedCalled())
    p->postFinalize();
}

void WbBallJoint::updateAnchor() {
  if (mJoint)
    applyToOdeAnchor();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();
}

void WbBallJoint::applyToOdeAnchor() {
  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &t = m4 * anchor();
  dJointSetBallAnchor(mJoint, t.x(), t.y(), t.z());
}

void WbBallJoint::applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) {
  const WbBallJointParameters *const p = ballJointParameters();
  if (p == NULL || (body == NULL && parentBody == NULL)) {
    if (mSpringAndDamperMotor) {
      dJointDestroy(mSpringAndDamperMotor);
      mSpringAndDamperMotor = NULL;
    }
    return;
  }

  assert((body || parentBody) && p);

  double d = p->dampingConstant();
  double s = p->springConstant();

  if (s == 0.0 && d == 0.0) {
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
  double cfm, erp;
  WbOdeUtilities::convertSpringAndDampingConstants(s, d, wi->basicTimeStep() * 0.001, cfm, erp);

  const dWorldID world = parentBody ? dBodyGetWorld(parentBody) : dBodyGetWorld(body);
  if (mSpringAndDamperMotor == NULL)
    mSpringAndDamperMotor = dJointCreateAMotor(world, 0);

  dJointAttach(mSpringAndDamperMotor, parentBody, body);
  dJointSetAMotorMode(mSpringAndDamperMotor, dAMotorEuler);

  // Axes settings
  const double invS = 1.0 / scale;
  const WbMatrix4 &m4 = ut->matrix();
  const WbVector3 &a0 = invS * m4.xAxis();
  const WbVector3 &a2 = invS * m4.zAxis();
  // in dAMotorEuler mode axis 0 must be anchored to body 1 and
  // axis 2 must be anchored to body 2
  // this mode doesn't automatically manage the dJOINT_REVERSE case,
  // thus the 'rel' body value must be manually switched
  dJointSetAMotorAxis(mSpringAndDamperMotor, 0, mIsReverseJoint ? 2 : 1, a0.x(), a0.y(), a0.z());
  dJointSetAMotorAxis(mSpringAndDamperMotor, 2, mIsReverseJoint ? 1 : 2, a2.x(), a2.y(), a2.z());
  // Stops
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop, 0.0);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop, 0.0);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop2, 0.0);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop2, 0.0);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamLoStop3, 0.0);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamHiStop3, 0.0);

  // Spring and damper through LCP
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM, cfm);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP, erp);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM2, cfm);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP2, erp);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopCFM3, cfm);
  dJointSetAMotorParam(mSpringAndDamperMotor, dParamStopERP3, erp);

  // Initial angles
  dJointSetAMotorAngle(mSpringAndDamperMotor, 0, 0.0);
  dJointSetAMotorAngle(mSpringAndDamperMotor, 1, 0.0);
  dJointSetAMotorAngle(mSpringAndDamperMotor, 2, 0.0);
}

//////////
// WREN //
//////////

void WbBallJoint::createWrenObjects() {
  WbBasicJoint::createWrenObjects();

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

void WbBallJoint::updateOdeWorldCoordinates() {
  if (!mJoint || !isPostFinalizedCalled())
    return;
  applyToOdeAnchor();

  if (mSpringAndDamperMotor) {
    // recompute axis world coordinates
    const WbTransform *const ut = upperTransform();
    const double invS = 1.0 / ut->absoluteScale().x();
    const WbMatrix4 &m4 = ut->matrix();
    const WbVector3 &a0 = invS * m4.xAxis();
    const WbVector3 &a2 = invS * m4.zAxis();
    dJointSetAMotorAxis(mSpringAndDamperMotor, 0, mIsReverseJoint ? 2 : 1, a0.x(), a0.y(), a0.z());
    dJointSetAMotorAxis(mSpringAndDamperMotor, 2, mIsReverseJoint ? 1 : 2, a2.x(), a2.y(), a2.z());
  }
}
