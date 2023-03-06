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

#include "WbColor.hpp"

#include "WbFieldChecker.hpp"
#include "WbMFColor.hpp"

void WbColor::init() {
  mColor = findMFColor("color");
}

WbColor::WbColor(WbTokenizer *tokenizer) : WbBaseNode("Color", tokenizer) {
  init();
}

WbColor::WbColor(const WbColor &other) : WbBaseNode(other) {
  init();
}

WbColor::WbColor(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbColor::~WbColor() {
}

void WbColor::preFinalize() {
  WbBaseNode::preFinalize();

  updateColor();
}

void WbColor::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mColor, &WbMFColor::changed, this, &WbColor::updateColor);
}

void WbColor::updateColor() {
  if (WbFieldChecker::resetMultipleColorIfInvalid(this, mColor))
    return;
  emit changed();
}

void WbColor::copyValuesToArray(double array[][3]) const {
  int nColors = mColor->size();
  for (int i = 0; i < nColors; i++) {
    WbRgb r = mColor->item(i);
    array[i][0] = r.red();
    array[i][1] = r.green();
    array[i][2] = r.blue();
  }
}

QStringList WbColor::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "color";
  return fields;
}
