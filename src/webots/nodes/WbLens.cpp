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

#include "WbLens.hpp"

void WbLens::init() {
  mCenter = findSFVector2("center");
  mRadialCoefficients = findSFVector2("radialCoefficients");
  mTangentialCoefficients = findSFVector2("tangentialCoefficients");
}

WbLens::WbLens(WbTokenizer *tokenizer) : WbBaseNode("Lens", tokenizer) {
  init();
}

WbLens::WbLens(const WbLens &other) : WbBaseNode(other) {
  init();
}

WbLens::WbLens(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbLens::~WbLens() {
}

void WbLens::preFinalize() {
  WbBaseNode::preFinalize();

  updateCenter();
  updateRadialCoefficients();
  updateTangentialCoefficients();
}

void WbLens::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mCenter, &WbSFVector2::changed, this, &WbLens::updateCenter);
  connect(mRadialCoefficients, &WbSFVector2::changed, this, &WbLens::updateRadialCoefficients);
  connect(mTangentialCoefficients, &WbSFVector2::changed, this, &WbLens::updateTangentialCoefficients);
}

void WbLens::updateCenter() {
  if (mCenter->value().x() < 0.0) {
    parsingWarn(tr("Invalid 'center.x' changed to 0. The value should be in the range [0;1]."));
    mCenter->setX(0.0);
  }
  if (mCenter->value().y() < 0.0) {
    parsingWarn(tr("Invalid 'center.y' changed to 0. The value should be in the range [0;1]."));
    mCenter->setY(0.0);
  }
  if (mCenter->value().x() > 1.0) {
    parsingWarn(tr("Invalid 'center.x' changed to 1. The value should be in the range [0;1]."));
    mCenter->setX(1.0);
  }
  if (mCenter->value().y() > 1.0) {
    parsingWarn(tr("Invalid 'center.y' changed to 1. The value should be in the range [0;1]."));
    mCenter->setX(1.0);
  }

  emit centerChanged();
}

void WbLens::updateRadialCoefficients() {
  emit radialCoefficientsChanged();
}

void WbLens::updateTangentialCoefficients() {
  emit tangentialCoefficientsChanged();
}
