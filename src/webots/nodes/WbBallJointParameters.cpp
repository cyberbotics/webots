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

#include "WbBallJointParameters.hpp"

void WbBallJointParameters::init() {
  mSpringConstant = findSFDouble("springConstant");
  mDampingConstant = findSFDouble("dampingConstant");
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

  updateSpringConstant();
  updateDampingConstant();
}

void WbBallJointParameters::postFinalize() {
  WbAnchorParameter::postFinalize();
  connect(mSpringConstant, &WbSFDouble::changed, this, &WbBallJointParameters::updateSpringConstant);
  connect(mDampingConstant, &WbSFDouble::changed, this, &WbBallJointParameters::updateDampingConstant);
}

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
