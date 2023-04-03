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

#include "WbLinearMotor.hpp"

#include "WbJoint.hpp"
#include "WbJointParameters.hpp"
#include "WbSolid.hpp"
#include "WbTrack.hpp"

#include <ode/ode.h>
#include <cassert>

void WbLinearMotor::init() {
  mMaxForceOrTorque = findSFDouble("maxForce");
}

WbLinearMotor::WbLinearMotor(WbTokenizer *tokenizer) : WbMotor("LinearMotor", tokenizer) {
  init();
}

WbLinearMotor::WbLinearMotor(const WbLinearMotor &other) : WbMotor(other) {
  init();
}

WbLinearMotor::WbLinearMotor(const WbNode &other) : WbMotor(other) {
  init();
}

WbLinearMotor::~WbLinearMotor() {
}

void WbLinearMotor::turnOffMotor() {
  WbJoint *j = joint();
  if (j == NULL)
    // function available for motorized joints only
    return;

  dJointID jID = j->jointID();
  if (jID) {
    dJointSetSliderParam(jID, dParamVel, 0.0);
    const WbJointParameters *p = j->parameters();
    dJointSetSliderParam(jID, dParamFMax, p ? p->staticFriction() : 0.0);
  }
}

double WbLinearMotor::computeFeedback() const {
  if (dynamic_cast<WbTrack *>(parentNode())) {
    warn(tr("Force feedback is not available for a LinearMotor node inside a Track node."));
    return 0.0;
  }

  const WbJoint *j = joint();
  if (j == NULL) {  // function available for motorized joints only
    warn(tr("Force feedback is available for motorized joints only"));
    return 0.0;
  }
  const dJointID jID = j->jointID();
  if (!jID)  // we need physics enabled to compute force feedback
    return 0.0;
  assert(j->solidEndPoint());
  assert(j->solidEndPoint()->bodyMerger());

  const dJointFeedback *fb = dJointGetFeedback(jID);
  assert(fb);

  /*printf("\n name: %ls\n", (const wchar_t *)name());
  printVector3("f1", fb->f1);
  printVector3("t1", fb->t1);
  printVector3("f2", fb->f2);
  printVector3("t2", fb->t2);*/

  // measure the force along the slider axis
  dVector3 axis;
  dJointGetSliderAxis(jID, axis);
  return dCalcVectorDot3(axis, fb->f1);
}
