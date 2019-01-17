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

#include "WbBallJointParameters.hpp"

void WbBallJointParameters::init() {
  mPosition = findSFDouble("position");
  mMinStop = findSFDouble("minStop");
  mMaxStop = findSFDouble("maxStop");
  mSpringConstant = findSFDouble("springConstant");
  mDampingConstant = findSFDouble("dampingConstant");
  mStaticFriction = findSFDouble("staticFriction");
}

WbBallJointParameters::WbBallJointParameters(WbTokenizer *tokenizer) : WbAnchorParameter("BallJointParameters", tokenizer) {
  init();
}

WbBallJointParameters::WbBallJointParameters(const WbBallJointParameters &other) : WbAnchorParameter(other) {
  init();
}

WbBallJointParameters::WbBallJointParameters(const WbNode &other) : WbAnchorParameter(other) {
  init();
}

WbBallJointParameters::~WbBallJointParameters() {
}

void WbBallJointParameters::preFinalize() {
  WbAnchorParameter::preFinalize();

  updateMinAndMaxStop();
  updateSpringConstant();
  updateDampingConstant();
}

void WbBallJointParameters::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mPosition, &WbSFDouble::changed, this, &WbBallJointParameters::positionChanged);
  connect(mMinStop, &WbSFDouble::changed, this, &WbBallJointParameters::updateMinAndMaxStop);
  connect(mMaxStop, &WbSFDouble::changed, this, &WbBallJointParameters::updateMinAndMaxStop);
  connect(mSpringConstant, &WbSFDouble::changed, this, &WbBallJointParameters::updateSpringConstant);
  connect(mDampingConstant, &WbSFDouble::changed, this, &WbBallJointParameters::updateDampingConstant);
  connect(mStaticFriction, &WbSFDouble::changed, this, &WbBallJointParameters::updateStaticFriction);
  disconnectFieldNotification(mPosition);
}

// Update methods: they check validity and correct if necessary

void WbBallJointParameters::updateSpringConstant() {
  if (mSpringConstant->value() < 0.0) {
    warn(tr("'springConstant' must be greater than or equal to zero."));
    mSpringConstant->makeAbsolute();
    return;
  }

  emit springAndDampingConstantsChanged();
}

void WbBallJointParameters::updateDampingConstant() {
  if (mDampingConstant->value() < 0.0) {
    warn(tr("'dampingConstant' must be greater than or equal to zero."));
    mDampingConstant->makeAbsolute();
    return;
  }

  emit springAndDampingConstantsChanged();
}


void WbBallJointParameters::updateStaticFriction() {
  if (mStaticFriction->value() < 0.0) {
    warn(tr("'staticFriction' must be greater than or equal to zero."));
    mStaticFriction->makeAbsolute();
    return;
  }
}

void WbBallJointParameters::updateMinAndMaxStop() {
  const double m = mMinStop->value();
  const double M = mMaxStop->value();
  const double p = mPosition->value();

  if (M != m) {
    emit minAndMaxStopChanged(m, M);

    // the joint's position must lie between minStop and maxStop
    // otherwise the Joint will be irrealistically pushed away from the stops when
    // the simulation starts which may cause instabilities

    if (p < m)  // in case of equality, some instabilities may be detected
      warn(tr("'minStop' must be less than or equal to 'position'."));

    if (p > M)  // in case of equality, some instabilities may be detected
      warn(tr("'maxStop' must be greater than or equal to 'position'."));
  }
}
