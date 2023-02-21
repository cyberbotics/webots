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

#include "WbJointParameters.hpp"

void WbJointParameters::init() {
  mPosition = findSFDouble("position");
  mAxis = findSFVector3("axis");
  mMinStop = findSFDouble("minStop");
  mMaxStop = findSFDouble("maxStop");
  mSpringConstant = findSFDouble("springConstant");
  mDampingConstant = findSFDouble("dampingConstant");
  mStaticFriction = findSFDouble("staticFriction");
}

// Constructors

WbJointParameters::WbJointParameters(const QString &modelName, WbTokenizer *tokenizer) : WbBaseNode(modelName, tokenizer) {
  init();
}

WbJointParameters::WbJointParameters(WbTokenizer *tokenizer) : WbBaseNode("JointParameters", tokenizer) {
  init();
}

WbJointParameters::WbJointParameters(const WbJointParameters &other) : WbBaseNode(other) {
  init();
}

WbJointParameters::WbJointParameters(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbJointParameters::~WbJointParameters() {
}

void WbJointParameters::preFinalize() {
  WbBaseNode::preFinalize();

  updateMinAndMaxStop();
  updateSpringConstant();
  updateDampingConstant();
  updateAxis();
}

void WbJointParameters::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mPosition, &WbSFDouble::changed, this, &WbJointParameters::positionChanged);
  connect(mMinStop, &WbSFDouble::changed, this, &WbJointParameters::updateMinAndMaxStop);
  connect(mMaxStop, &WbSFDouble::changed, this, &WbJointParameters::updateMinAndMaxStop);
  connect(mSpringConstant, &WbSFDouble::changed, this, &WbJointParameters::updateSpringConstant);
  connect(mDampingConstant, &WbSFDouble::changed, this, &WbJointParameters::updateDampingConstant);
  connect(mStaticFriction, &WbSFDouble::changed, this, &WbJointParameters::updateStaticFriction);
  if (mAxis)
    connect(mAxis, &WbSFDouble::changed, this, &WbJointParameters::updateAxis);
  disconnectFieldNotification(mPosition);
}

// Update methods: they check validity and correct if necessary

void WbJointParameters::updateSpringConstant() {
  if (mSpringConstant->value() < 0.0) {
    parsingWarn(tr("'springConstant' must be greater than or equal to zero."));
    mSpringConstant->makeAbsolute();
    return;
  }

  emit springAndDampingConstantsChanged();
}

void WbJointParameters::updateDampingConstant() {
  if (mDampingConstant->value() < 0.0) {
    parsingWarn(tr("'dampingConstant' must be greater than or equal to zero."));
    mDampingConstant->makeAbsolute();
    return;
  }

  emit springAndDampingConstantsChanged();
}

void WbJointParameters::updateStaticFriction() {
  if (mStaticFriction->value() < 0.0) {
    parsingWarn(tr("'staticFriction' must be greater than or equal to zero."));
    mStaticFriction->makeAbsolute();
    return;
  }
}

void WbJointParameters::updateMinAndMaxStop() {
  const double m = mMinStop->value();
  const double M = mMaxStop->value();
  const double p = mPosition->value();

  if (M != m) {
    emit minAndMaxStopChanged(m, M);

    // the joint's position must lie between minStop and maxStop
    // otherwise the Joint will be irrealistically pushed away from the stops when
    // the simulation starts which may cause instabilities

    if (p < m)  // in case of equality, some instabilities may be detected
      parsingWarn(tr("'minStop' must be less than or equal to 'position'."));

    if (p > M)  // in case of equality, some instabilities may be detected
      parsingWarn(tr("'maxStop' must be greater than or equal to 'position'."));
  }
}

void WbJointParameters::updateAxis() {
  if (!mAxis)
    return;
  const WbVector3 &a = mAxis->value();
  if (a.isNull()) {
    parsingWarn(tr("'axis' must be non zero."));
    mAxis->setValue(0.0, 0.0, 1.0);
    return;
  }

  emit axisChanged();
}

bool WbJointParameters::clampPosition(double &p) const {
  if (mMinStop->value() == mMaxStop->value() && mMinStop->value() == 0)
    return false;

  if (p < mMinStop->value())
    p = mMinStop->value();
  else if (p > mMaxStop->value())
    p = mMaxStop->value();
  else
    return false;
  return true;
}

bool WbJointParameters::exportNodeHeader(WbWriter &writer) const {
  if (writer.isUrdf())
    return true;
  return WbBaseNode::exportNodeHeader(writer);
}
