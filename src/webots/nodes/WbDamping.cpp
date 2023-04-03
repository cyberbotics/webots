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

#include "WbDamping.hpp"
#include "WbWorld.hpp"

void WbDamping::init() {
  mLinear = findSFDouble("linear");
  mAngular = findSFDouble("angular");
}

WbDamping::WbDamping(WbTokenizer *tokenizer) : WbBaseNode("Damping", tokenizer) {
  init();
}

WbDamping::WbDamping(const WbDamping &other) : WbBaseNode(other) {
  init();
}

WbDamping::WbDamping(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbDamping::~WbDamping() {
}

void WbDamping::preFinalize() {
  WbBaseNode::preFinalize();

  updateLinear();
  updateAngular();
}

void WbDamping::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mLinear, &WbSFDouble::changed, this, &WbDamping::updateLinear);
  connect(mAngular, &WbSFDouble::changed, this, &WbDamping::updateAngular);
}

void WbDamping::updateLinear() {
  if (mLinear->value() < 0.0) {
    parsingWarn(tr("'linear' must be greater than or equal to zero."));
    mLinear->setValue(0.0);
    return;
  }

  if (mLinear->value() > 1.0) {
    parsingWarn(tr("'linear' must be less than or equal to one."));
    mLinear->setValue(1.0);
    return;
  }

  emit changed();
}

void WbDamping::updateAngular() {
  if (mAngular->value() < 0.0) {
    parsingWarn(tr("'angular' must be greater than or equal to zero."));
    mAngular->setValue(0.0);
    return;
  }

  if (mAngular->value() > 1.0) {
    parsingWarn(tr("'angular' must be less than or equal to one."));
    mAngular->setValue(1.0);
    return;
  }
  emit changed();
}
