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

#include "WbSolidReference.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSolid.hpp"

#include <cassert>

const QString WbSolidReference::STATIC_ENVIRONMENT = QString("<static environment>");

void WbSolidReference::init() {
  mSolid = NULL;
  mName = findSFString("solidName");
}

// Constructors

WbSolidReference::WbSolidReference(WbTokenizer *tokenizer) : WbBaseNode("SolidReference", tokenizer) {
  init();
}

WbSolidReference::WbSolidReference(const WbSolidReference &other) : WbBaseNode(other) {
  init();
}

WbSolidReference::WbSolidReference(const WbNode &other) : WbBaseNode(other) {
  init();
}

// Destructor
WbSolidReference::~WbSolidReference() {
}

void WbSolidReference::preFinalize() {
  WbBaseNode::preFinalize();
}

void WbSolidReference::postFinalize() {
  WbBaseNode::postFinalize();
  connect(mName, &WbSFString::changed, this, &WbSolidReference::changed);
}

void WbSolidReference::updateName() {
  WbSolid *const ts = topSolid();
  assert(ts);
  const QString &name = mName->value();
  const bool linkToStaticEnvironment = name == "<static environment>";
  if (!linkToStaticEnvironment)
    mSolid = QPointer<WbSolid>(ts->findSolid(name, upperSolid()));
  if (!name.isEmpty() && !linkToStaticEnvironment && mSolid.isNull())
    warn(tr("SolidReference has an invalid '%1' name or refers to its closest upper solid, which is prohibited.").arg(name));
}

bool WbSolidReference::isClosedLoop() const {
  if (!mSolid)
    return false;

  WbNode *parentNode = parent();
  while (parentNode && !parentNode->isWorldRoot()) {
    if (parentNode == mSolid)
      return true;
    parentNode = parentNode->parent();
  }
  return false;
}
