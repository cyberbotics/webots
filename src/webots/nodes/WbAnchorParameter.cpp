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

#include "WbAnchorParameter.hpp"

void WbAnchorParameter::init() {
  mAnchor = findSFVector3("anchor");
}

// Constructors

WbAnchorParameter::WbAnchorParameter(const QString &modelName, WbTokenizer *tokenizer) : WbBaseNode(modelName, tokenizer) {
  init();
}

WbAnchorParameter::WbAnchorParameter(const WbAnchorParameter &other) : WbBaseNode(other) {
  init();
}

WbAnchorParameter::WbAnchorParameter(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbAnchorParameter::~WbAnchorParameter() {
}

void WbAnchorParameter::preFinalize() {
  WbBaseNode::preFinalize();
}

void WbAnchorParameter::postFinalize() {
  WbBaseNode::postFinalize();
  connect(mAnchor, &WbSFVector3::changed, this, &WbAnchorParameter::anchorChanged);
}
