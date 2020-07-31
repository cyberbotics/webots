// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "WbJointLink.hpp"

void WbJointLink::init() {
}

// Constructors

WbJointLink::WbJointLink(const QString &modelName, WbTokenizer *tokenizer) : WbBaseNode(modelName, tokenizer) {
  init();
}

WbJointLink::WbJointLink(WbTokenizer *tokenizer) : WbBaseNode("JointLink", tokenizer) {
  init();
}

WbJointLink::WbJointLink(const WbJointLink &other) : WbBaseNode(other) {
  init();
}

WbJointLink::WbJointLink(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbJointLink::~WbJointLink() {
}

void WbJointLink::preFinalize() {
  WbBaseNode::preFinalize();

  updateMinAndMaxStop();
  updateSpringConstant();
  updateDampingConstant();
  updateAxis();
}

void WbJointParameters::postFinalize() {
  WbBaseNode::postFinalize();
}

// Update methods: they check validity and correct if necessary
