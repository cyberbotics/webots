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

//
//  WbRotationalMotor.cpp
//

#include "WbRotationalMotor.hpp"

#include "WbJoint.hpp"
#include "WbJointParameters.hpp"
#include "WbMathsUtilities.hpp"
#include "WbPropeller.hpp"
#include "WbSolid.hpp"

#include <ode/ode.h>

#include <cassert>

void WbRotationalMotor::init() {
  mMaxForceOrTorque = findSFDouble("maxTorque");
}

WbRotationalMotor::WbRotationalMotor(WbTokenizer *tokenizer) : WbMotor("RotationalMotor", tokenizer) {
  init();
}

WbRotationalMotor::WbRotationalMotor(const WbRotationalMotor &other) : WbMotor(other) {
  init();
}

WbRotationalMotor::WbRotationalMotor(const WbNode &other) : WbMotor(other) {
  init();
}

WbRotationalMotor::~WbRotationalMotor() {
}

void WbRotationalMotor::turnOffMotor() {
  const WbJoint *const j = joint();
  if (j == NULL)  // this can occur for example in a Propeller.
    return;
  dJointID jID = j->jointID();
  if (jID == NULL)
    return;

  if (j->nodeType() == WB_NODE_HINGE_JOINT) {
    dJointSetHingeParam(jID, dParamVel, 0.0);
    const WbJointParameters *const p = j->parameters();
    dJointSetHingeParam(jID, dParamFMax, p ? p->staticFriction() : 0.0);
  } else if (j->nodeType() == WB_NODE_HINGE_2_JOINT) {
    if (this == j->motor()) {
      const WbJointParameters *const p = j->parameters();
      dJointSetHinge2Param(jID, dParamFMax, p ? p->staticFriction() : 0.0);
    } else if (this == j->motor2()) {
      const WbJointParameters *const p2 = j->parameters2();
      dJointSetHinge2Param(jID, dParamFMax2, p2 ? p2->staticFriction() : 0.0);
    } else
      assert(false);
  } else if (j->nodeType() == WB_NODE_BALL_JOINT) {
    if (this == j->motor()) {
      const WbJointParameters *const p = j->parameters();
      dJointSetAMotorParam(jID, dParamFMax, p ? p->staticFriction() : 0.0);
    } else if (this == j->motor2()) {
      const WbJointParameters *const p2 = j->parameters2();
      dJointSetAMotorParam(jID, dParamFMax2, p2 ? p2->staticFriction() : 0.0);
    } else if (this == j->motor3()) {
      const WbJointParameters *const p3 = j->parameters3();
      dJointSetAMotorParam(jID, dParamFMax3, p3 ? p3->staticFriction() : 0.0);
    } else
      assert(false);
  } else
    assert(false);
}

double WbRotationalMotor::computeFeedback() const {
  const WbPropeller *propeller = dynamic_cast<WbPropeller *>(parentNode());
  if (propeller)
    // RotationalMotor usually returns only torque feedback,
    // but the propeller case is different since the motor produces both a torque and a force (thrust).
    // We therefore estimate the feedback by the sum of the force and torque,
    // so that the consumption computation takes both into account.
    return fabs(propeller->currentThrust()) + fabs(propeller->currentTorque());

  const WbJoint *j = joint();
  if (j == NULL) {  // function available for motorized joints only
    warn(tr("Feedback is available for motorized joints only"));
    return 0.0;
  }
  const dJointID jID = j->jointID();
  if (!jID)  // we need physics enabled to compute torque feedback
    return 0.0;

  const bool hinge = j->nodeType() == WB_NODE_HINGE_JOINT;
  const bool hinge2 = j->nodeType() == WB_NODE_HINGE_2_JOINT;
  assert(hinge || hinge2);
  if (hinge2 && dJointGetNumBodies(jID) == 0) {
    // invalid hinge2 linked to static environment
    warn(tr("Hinge2Joint is invalid: feedback is not available."));
    return 0.0;
  }

  assert(j->solidEndPoint());
  const dBodyID b = j->solidEndPoint()->bodyMerger();
  assert(b);

  const dJointFeedback *const fb = dJointGetFeedback(jID);
  assert(fb);

  // qDebug() << deviceName();
  // using namespace WbMathsUtilities;
  // printVector3("f1", fb->f1);
  // printVector3("t1", fb->t1);
  // printVector3("f2", fb->f2);
  // printVector3("t2", fb->t2);

  // ODE dJointFeedback contains torque applied at body CoMs
  // we need to compute torque at hinge anchor instead
  // see explanations here:
  //   http://www.ode.org/old_list_archives/2005-January/014948.html
  // (all calculations are in global coordinate system)
  dVector3 anchor, sub, t2;
  if (hinge)
    dJointGetHingeAnchor2(jID, anchor);
  else
    dJointGetHinge2Anchor2(jID, anchor);

  const dReal *const p2 = dBodyGetPosition(b);
  dSubtractVectors3(sub, anchor, p2);
  dCopyVector3(t2, fb->t2);
  dAddVectorCross3(t2, fb->f2, sub);

  // project torque onto hinge axis:
  // a positive torque makes the RotationalMotor rotate in the positive direction
  // (this assumes that ODE returns a normalized axis, i.e., with length = 1.0)
  // Note: projection on the hinge axis is not giving the actual driving torque: see latest reference about dJointFeedback in
  // http://ode-wiki.org/wiki/index.php?title=Manual:_Joint_Types_and_Functions
  dVector3 axis;
  if (hinge)
    dJointGetHingeAxis(jID, axis);
  else {
    const QVector<WbLogicalDevice *> &devices = j->devices();
    if (this == devices.at(0))
      dJointGetHinge2Axis1(jID, axis);
    else
      dJointGetHinge2Axis2(jID, axis);
  }
  return dCalcVectorDot3(axis, t2);
}
