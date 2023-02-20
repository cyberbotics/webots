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

#include "WbZoom.hpp"

#include "WbFieldChecker.hpp"

void WbZoom::init() {
  mMaxFieldOfView = findSFDouble("maxFieldOfView");
  mMinFieldOfView = findSFDouble("minFieldOfView");
}

WbZoom::WbZoom(WbTokenizer *tokenizer) : WbBaseNode("Zoom", tokenizer) {
  init();
}

WbZoom::WbZoom(const WbZoom &other) : WbBaseNode(other) {
  init();
}

WbZoom::WbZoom(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbZoom::~WbZoom() {
}

void WbZoom::preFinalize() {
  WbBaseNode::preFinalize();

  updateMinFieldOfView();
  updateMaxFieldOfView();
}

void WbZoom::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mMinFieldOfView, &WbSFDouble::changed, this, &WbZoom::updateMinFieldOfView);
  connect(mMaxFieldOfView, &WbSFDouble::changed, this, &WbZoom::updateMaxFieldOfView);
}

void WbZoom::updateMinFieldOfView() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mMinFieldOfView, 0.0))
    return;
  if (mMinFieldOfView->value() > mMaxFieldOfView->value()) {
    parsingWarn(tr("Invalid 'minFieldOfView' changed to %1. The value should be smaller or equal to 'maxFieldOfView'.")
                  .arg(mMaxFieldOfView->value()));
    mMinFieldOfView->setValue(mMaxFieldOfView->value());
    return;
  }
}

void WbZoom::updateMaxFieldOfView() {
  if (mMaxFieldOfView->value() < mMinFieldOfView->value()) {
    parsingWarn(tr("Invalid 'maxFieldOfView' changed to %1. The value should be bigger or equal to 'minFieldOfView'.")
                  .arg(mMinFieldOfView->value() + 0.1));
    mMaxFieldOfView->setValue(mMinFieldOfView->value() + 0.1);
    return;
  }
}
