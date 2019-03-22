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

#include "WbHingeJointParameters.hpp"

void WbHingeJointParameters::init(bool fromDeprecatedHinge2JointParameters) {
  mAnchor = findSFVector3("anchor");
  mSuspensionSpringConstant = findSFDouble("suspensionSpringConstant");
  mSuspensionDampingConstant = findSFDouble("suspensionDampingConstant");
  mSuspensionAxis = findSFVector3("suspensionAxis");

  // DEPRECATED, only for backward compatibility
  if (fromDeprecatedHinge2JointParameters) {
    if (mAxis) {  // if axis is set the suspension axis must be equal to it
      if (mSuspensionAxis)
        mSuspensionAxis->setValue(mAxis->value());
      else
        mSuspensionAxis = new WbSFVector3(mAxis->value());
    }
    warn(tr("'Hinge2JointParameters' is deprecated, please use 'HingeJointParameters' instead."));
  }
}

// Constructors

WbHingeJointParameters::WbHingeJointParameters(const QString &modelName, WbTokenizer *tokenizer) :
  WbJointParameters(modelName, tokenizer) {
  init();
}

WbHingeJointParameters::WbHingeJointParameters(WbTokenizer *tokenizer, bool fromDeprecatedHinge2JointParameters) :
  WbJointParameters("HingeJointParameters", tokenizer) {
  init(fromDeprecatedHinge2JointParameters);
}

WbHingeJointParameters::WbHingeJointParameters(const WbHingeJointParameters &other) : WbJointParameters(other) {
  init();
}

WbHingeJointParameters::WbHingeJointParameters(const WbNode &other, bool fromDeprecatedHinge2JointParameters) :
  WbJointParameters(other) {
  init(fromDeprecatedHinge2JointParameters);
}

WbHingeJointParameters::~WbHingeJointParameters() {
}

void WbHingeJointParameters::preFinalize() {
  WbJointParameters::preFinalize();
}

void WbHingeJointParameters::postFinalize() {
  WbJointParameters::postFinalize();

  connect(mAnchor, &WbSFVector3::changed, this, &WbHingeJointParameters::anchorChanged);
  connect(mSuspensionSpringConstant, &WbSFDouble::changed, this, &WbHingeJointParameters::updateSuspension);
  connect(mSuspensionDampingConstant, &WbSFDouble::changed, this, &WbHingeJointParameters::updateSuspension);
  connect(mSuspensionAxis, &WbSFVector3::changed, this, &WbHingeJointParameters::updateSuspension);
}

void WbHingeJointParameters::updateAxis() {
  const WbVector3 &a = mAxis->value();
  if (a.isNull()) {
    warn(tr("'axis' must be non zero."));
    mAxis->setValue(1.0, 0.0, 0.0);
    return;
  }

  emit axisChanged();
}

void WbHingeJointParameters::updateSuspension() {
  const WbVector3 &a = mSuspensionAxis->value();
  if (a.isNull()) {
    warn(tr("'SuspensionAxis' must be non zero."));
    mSuspensionAxis->setValue(1.0, 0.0, 0.0);
    return;
  }

  emit suspensionChanged();
}
