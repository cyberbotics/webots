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

#include "WbImmersionProperties.hpp"

#include "WbWorld.hpp"

#include <ode/fluid_dynamics/immersion.h>

void WbImmersionProperties::init() {
  mFluidName = findSFString("fluidName");
  mReferenceArea = findSFString("referenceArea");
  mDragForceCoefficients = findSFVector3("dragForceCoefficients");
  mDragTorqueCoefficients = findSFVector3("dragTorqueCoefficients");
  mViscousResistanceForceCoefficient = findSFDouble("viscousResistanceForceCoefficient");
  mViscousResistanceTorqueCoefficient = findSFDouble("viscousResistanceTorqueCoefficient");
  mImmersionSurfaceMode = dImmersionProjectedAreas;
}

WbImmersionProperties::WbImmersionProperties(WbTokenizer *tokenizer) : WbBaseNode("ImmersionProperties", tokenizer) {
  init();
}

WbImmersionProperties::WbImmersionProperties(const WbImmersionProperties &other) : WbBaseNode(other) {
  init();
}

WbImmersionProperties::WbImmersionProperties(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbImmersionProperties::~WbImmersionProperties() {
}

void WbImmersionProperties::preFinalize() {
  WbBaseNode::preFinalize();

  updateDragForceCoefficients();
  updateDragTorqueCoefficients();
  updateLinearViscousResistanceCoefficient();
  updateAngularViscousResistanceCoefficient();
  updateReferenceArea();
}

void WbImmersionProperties::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mDragForceCoefficients, &WbSFVector3::changed, this, &WbImmersionProperties::updateDragForceCoefficients);
  connect(mDragTorqueCoefficients, &WbSFVector3::changed, this, &WbImmersionProperties::updateDragTorqueCoefficients);
  connect(mViscousResistanceForceCoefficient, &WbSFDouble::changed, this,
          &WbImmersionProperties::updateLinearViscousResistanceCoefficient);
  connect(mViscousResistanceTorqueCoefficient, &WbSFDouble::changed, this,
          &WbImmersionProperties::updateAngularViscousResistanceCoefficient);
}

void WbImmersionProperties::updateDragForceCoefficients() {
  const WbVector3 &c = mDragForceCoefficients->value();
  if (c.x() < 0.0) {
    parsingWarn(tr("'cx' must be greater than or equal to zero."));
    mDragForceCoefficients->setX(0.0);
    return;
  }

  if (c.y() < 0.0) {
    parsingWarn(tr("'cy' must be greater than or equal to zero."));
    mDragForceCoefficients->setY(0.0);
    return;
  }

  if (c.z() < 0.0) {
    parsingWarn(tr("'cz' must be greater than or equal to zero."));
    mDragForceCoefficients->setZ(0.0);
    return;
  }
}

void WbImmersionProperties::updateDragTorqueCoefficients() {
  const WbVector3 &t = mDragTorqueCoefficients->value();
  if (t.x() < 0.0) {
    parsingWarn(tr("'tx' must be greater than or equal to zero."));
    mDragTorqueCoefficients->setX(0.0);
    return;
  }

  if (t.y() < 0.0) {
    parsingWarn(tr("'ty' must be greater than or equal to zero."));
    mDragTorqueCoefficients->setY(0.0);
    return;
  }

  if (t.z() < 0.0) {
    parsingWarn(tr("'tz' must be greater than or equal to zero."));
    mDragTorqueCoefficients->setZ(0.0);
    return;
  }
}

void WbImmersionProperties::updateLinearViscousResistanceCoefficient() {
  const double l = mViscousResistanceForceCoefficient->value();
  if (l < 0.0) {
    parsingWarn(tr("'viscousResistanceForceCoefficient' must be greater than or equal to zero."));
    mViscousResistanceForceCoefficient->setValue(0.0);
    return;
  }
}

void WbImmersionProperties::updateAngularViscousResistanceCoefficient() {
  const double a = mViscousResistanceTorqueCoefficient->value();
  if (a < 0.0) {
    parsingWarn(tr("'viscousResistanceTorqueCoefficient' must be greater than or equal to zero."));
    mViscousResistanceTorqueCoefficient->setValue(0.0);
    return;
  }
}

void WbImmersionProperties::updateReferenceArea() {
  if (mReferenceArea->value() == "xyz-projection")
    mImmersionSurfaceMode = dImmersionProjectedAreas;
  else if (mReferenceArea->value() == "immersed area")
    mImmersionSurfaceMode = dImmersionImmersedArea;
}
