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

#include "WbFocus.hpp"

#include "WbFieldChecker.hpp"

void WbFocus::init() {
  mFocalDistance = findSFDouble("focalDistance");
  mFocalLength = findSFDouble("focalLength");
  mMaxFocalDistance = findSFDouble("maxFocalDistance");
  mMinFocalDistance = findSFDouble("minFocalDistance");
}

WbFocus::WbFocus(WbTokenizer *tokenizer) : WbBaseNode("Focus", tokenizer) {
  init();
}

WbFocus::WbFocus(const WbFocus &other) : WbBaseNode(other) {
  init();
}

WbFocus::WbFocus(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbFocus::~WbFocus() {
}

void WbFocus::preFinalize() {
  WbBaseNode::preFinalize();

  updateFocalDistance();
  updateFocalLength();
  updateMinFocalDistance();
  updateMaxFocalDistance();
}

void WbFocus::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mFocalDistance, &WbSFDouble::changed, this, &WbFocus::updateFocalDistance);
  connect(mFocalLength, &WbSFDouble::changed, this, &WbFocus::updateFocalLength);
  connect(mMinFocalDistance, &WbSFDouble::changed, this, &WbFocus::updateMinFocalDistance);
  connect(mMaxFocalDistance, &WbSFDouble::changed, this, &WbFocus::updateMaxFocalDistance);
}

void WbFocus::updateFocalDistance() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mFocalDistance, 0.0))
    return;
  emit focusSettingsChanged();
}

void WbFocus::updateFocalLength() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mFocalLength, 0.0))
    return;
  emit focusSettingsChanged();
}

void WbFocus::updateMinFocalDistance() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mMinFocalDistance, 0.0))
    return;
  if (mMinFocalDistance->value() > mMaxFocalDistance->value()) {
    parsingWarn(tr("Invalid 'minFocalDistance' changed to %1. The value should be smaller or equal to 'maxFocalDistance'.")
                  .arg(mMaxFocalDistance->value()));
    mMinFocalDistance->setValue(mMaxFocalDistance->value());
    return;
  }
}

void WbFocus::updateMaxFocalDistance() {
  if (mMaxFocalDistance->value() < mMinFocalDistance->value()) {
    parsingWarn(tr("Invalid 'maxFocalDistance' changed to %1. The value should be bigger or equal to 'minFocalDistance'.")
                  .arg(mMinFocalDistance->value() + 0.1));
    mMaxFocalDistance->setValue(mMinFocalDistance->value() + 0.1);
    return;
  }
}
