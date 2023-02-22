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

#include "WbHingeJointParameters.hpp"

void WbHingeJointParameters::init(bool fromDeprecatedHinge2JointParameters) {
  mAnchor = findSFVector3("anchor");
  mSuspensionSpringConstant = findSFDouble("suspensionSpringConstant");
  mSuspensionDampingConstant = findSFDouble("suspensionDampingConstant");
  mSuspensionAxis = findSFVector3("suspensionAxis");
  mStopErp = findSFDouble("stopERP");
  mStopCfm = findSFDouble("stopCFM");

  // DEPRECATED, only for backward compatibility
  if (fromDeprecatedHinge2JointParameters) {
    if (mAxis) {  // if axis is set the suspension axis must be equal to it
      if (mSuspensionAxis)
        mSuspensionAxis->setValue(mAxis->value());
      else
        mSuspensionAxis = new WbSFVector3(mAxis->value());
    }
    parsingWarn(tr("'Hinge2JointParameters' is deprecated, please use 'HingeJointParameters' instead."));
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
  connect(mStopErp, &WbSFDouble::changed, this, &WbHingeJointParameters::updateStopErp);
  connect(mStopCfm, &WbSFDouble::changed, this, &WbHingeJointParameters::updateStopCfm);
  updateStopErp();
  updateStopCfm();
}

void WbHingeJointParameters::updateAxis() {
  const WbVector3 &a = mAxis->value();
  if (a.isNull()) {
    parsingWarn(tr("'axis' must be non zero."));
    mAxis->setValue(1.0, 0.0, 0.0);
    return;
  }

  emit axisChanged();
}

void WbHingeJointParameters::updateSuspension() {
  const WbVector3 &a = mSuspensionAxis->value();
  if (a.isNull()) {
    parsingWarn(tr("'SuspensionAxis' must be non zero."));
    mSuspensionAxis->setValue(1.0, 0.0, 0.0);
    return;
  }

  emit suspensionChanged();
}

void WbHingeJointParameters::updateStopErp() {
  if (mStopErp->value() < 0.0 && mStopErp->value() != -1) {
    mStopErp->setValue(-1);
    parsingWarn(tr("'stopERP' must be greater or equal to zero or -1. Reverting to -1 (use global ERP)."));
    return;
  }

  emit stopErpChanged();
}

void WbHingeJointParameters::updateStopCfm() {
  if (mStopCfm->value() <= 0.0 && mStopCfm->value() != -1) {
    mStopCfm->setValue(-1);
    parsingWarn(tr("'stopCFM' must be greater than zero or -1. Reverting to -1 (use global CFM)."));
    return;
  }

  emit stopCfmChanged();
}
