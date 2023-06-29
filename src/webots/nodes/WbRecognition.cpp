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

#include "WbRecognition.hpp"

#include "WbFieldChecker.hpp"

void WbRecognition::init() {
  mMaxRange = findSFDouble("maxRange");
  mMaxObjects = findSFInt("maxObjects");
  mOcclusion = findSFInt("occlusion");
  mFrameColor = findSFColor("frameColor");
  mFrameThickness = findSFInt("frameThickness");
  mSegmentation = findSFBool("segmentation");
}

WbRecognition::WbRecognition(WbTokenizer *tokenizer) : WbBaseNode("Recognition", tokenizer) {
  init();
}

WbRecognition::WbRecognition(const WbRecognition &other) : WbBaseNode(other) {
  init();
}

WbRecognition::WbRecognition(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbRecognition::~WbRecognition() {
}

void WbRecognition::preFinalize() {
  WbBaseNode::preFinalize();

  updateMaxRange();
  updateMaxObjects();
  updateOcclusion();
}

void WbRecognition::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mMaxRange, &WbSFDouble::changed, this, &WbRecognition::updateMaxRange);
  connect(mMaxObjects, &WbSFInt::changed, this, &WbRecognition::updateMaxObjects);
  connect(mOcclusion, &WbSFInt::changed, this, &WbRecognition::updateOcclusion);
  connect(mFrameThickness, &WbSFInt::changed, this, &WbRecognition::updateFrameThickness);
  connect(mSegmentation, &WbSFBool::changed, this, &WbRecognition::segmentationChanged);
}

void WbRecognition::updateMaxRange() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mMaxRange, 100.0))
    return;
}

void WbRecognition::updateMaxObjects() {
  if (WbFieldChecker::resetIntIfNonPositiveAndNotDisabled(this, mMaxObjects, -1, -1))
    return;
}

void WbRecognition::updateOcclusion() {
  if (mOcclusion->value() < 0 || mOcclusion->value() > 2) {
    parsingWarn(tr("Invalid 'occlusion' changed to 1. The value should be 0, 1 or 2."));
    mOcclusion->setValue(1);
  }
}

void WbRecognition::updateFrameThickness() {
  if (WbFieldChecker::resetIntIfNegative(this, mFrameThickness, 0))
    return;
}
