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

#include "WbBallJointParameters.hpp"

void WbBallJointParameters::init() {
  mAnchor = findSFVector3("anchor");
}

WbBallJointParameters::WbBallJointParameters(WbTokenizer *tokenizer) : WbJointParameters("BallJointParameters", tokenizer) {
  init();
}

WbBallJointParameters::WbBallJointParameters(const WbBallJointParameters &other) : WbJointParameters(other) {
  init();
}

WbBallJointParameters::WbBallJointParameters(const WbNode &other) : WbJointParameters(other) {
  init();
}

WbBallJointParameters::~WbBallJointParameters() {
}

void WbBallJointParameters::preFinalize() {
  WbJointParameters::preFinalize();
}

void WbBallJointParameters::postFinalize() {
  WbJointParameters::postFinalize();

  connect(mAnchor, &WbSFVector3::changed, this, &WbBallJointParameters::anchorChanged);
}
