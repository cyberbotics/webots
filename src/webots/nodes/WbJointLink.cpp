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

#include "WbJoint.hpp"
#include "WbSolid.hpp"

void WbJointLink::init() {
  mEndPointName = findSFString("endPointName");
  mMultiplier = findSFDouble("multiplier");
  mGain = findSFDouble("gain");
  mJointId = findSFInt("jointId");

  mJoint = NULL;
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

  updateEndPointName();
  updateMultiplier();
  updateGain();
  updateJointId();
}

void WbJointLink::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mEndPointName, &WbSFString::changed, this, &WbJointLink::updateEndPointName);
  connect(mMultiplier, &WbSFDouble::changed, this, &WbJointLink::updateMultiplier);
  connect(mGain, &WbSFDouble::changed, this, &WbJointLink::updateGain);
  connect(mJointId, &WbSFInt::changed, this, &WbJointLink::updateJointId);
}

WbMotor *WbJointLink::jointMotor() const {
  if (!mJoint)
    return NULL;

  // TODO: take 'jointId' into account
  return mJoint->motor();
}

// Update methods: they check validity and correct if necessary
void WbJointLink::updateEndPointName() {
  WbSolid *const ts = topSolid();
  assert(ts);
  const QString &name = mEndPointName->value();
  const WbSolid *solid = ts->findSolid(name, upperSolid());
  if (!name.isEmpty() && !solid) {
    parsingWarn(tr("JointLink has an invalid '%1' endPointName or refers to itself, which is prohibited.").arg(name));
    return;
  }
  mJoint = dynamic_cast<WbJoint *>(solid->parentNode());
  // TODO: connect to mJoint deletion
  if (!mJoint) {
    parsingWarn(tr("JointLink has an invalid '%1' endPointName, the parent of this Solid is not a joint.").arg(name));
    return;
  }
  updateJointId();
  // TODO: emit signal
}

void WbJointLink::updateMultiplier() {
}

void WbJointLink::updateGain() {
}

void WbJointLink::updateJointId() {
}
