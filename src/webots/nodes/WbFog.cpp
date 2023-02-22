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

#include "WbFog.hpp"

#include "WbFieldChecker.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbSFString.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/scene.h>

#include <cassert>
#include <limits>

QList<WbFog *> WbFog::cFogList;

void WbFog::init() {
  mWrenFogType = WR_SCENE_FOG_TYPE_LINEAR;

  mColor = findSFColor("color");
  mFogType = findSFString("fogType");
  mVisibilityRange = findSFDouble("visibilityRange");
}

WbFog::WbFog(WbTokenizer *tokenizer) : WbBaseNode("Fog", tokenizer) {
  init();
}

WbFog::WbFog(const WbFog &other) : WbBaseNode(other) {
  init();
}

WbFog::WbFog(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbFog::~WbFog() {
  cFogList.removeAll(this);

  if (areWrenObjectsInitialized())
    wr_scene_set_fog(wr_scene_get_instance(), WR_SCENE_FOG_TYPE_NONE, WR_SCENE_FOG_DEPTH_TYPE_PLANE, NULL, 1.0f, 0.0f, 1.0f);

  foreach (WbFog *fog, cFogList)
    fog->updateAndConnectIfNeeded();

  if (!WbWorld::instance()->isCleaning())
    emit WbWrenRenderingContext::instance()->fogNodeDeleted();
}

void WbFog::preFinalize() {
  WbBaseNode::preFinalize();

  cFogList << this;

  if (isFirstInstance()) {
    updateColor();
    updateFogType();
    updateVisibilityRange();

    if (!WbWorld::instance()->isLoading())
      emit WbWrenRenderingContext::instance()->fogNodeAdded();
  } else
    parsingWarn(tr("Only one Fog node is allowed. Only the first Fog node will be taken into account."));
}

void WbFog::postFinalize() {
  WbBaseNode::postFinalize();

  if (isFirstInstance()) {
    connect(mColor, &WbSFColor::changed, this, &WbFog::updateColor);
    connect(mFogType, &WbSFString::changed, this, &WbFog::updateFogType);
    connect(mVisibilityRange, &WbSFDouble::changed, this, &WbFog::updateVisibilityRange);
  }
}

void WbFog::updateAndConnectIfNeeded() {
  if (isFirstInstance()) {
    updateColor();
    updateFogType();
    updateVisibilityRange();
    connect(mColor, &WbSFColor::changed, this, &WbFog::updateColor);
    connect(mFogType, &WbSFString::changed, this, &WbFog::updateFogType);
    connect(mVisibilityRange, &WbSFDouble::changed, this, &WbFog::updateVisibilityRange);
  }
}

void WbFog::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  if (isFirstInstance())
    applyChangesToWren();
}

void WbFog::updateColor() {
  if (WbFieldChecker::resetColorIfInvalid(this, mColor))
    return;

  if (areWrenObjectsInitialized())
    applyChangesToWren();
}

void WbFog::updateFogType() {
  const QString fogText(mFogType->value().toLower());
  if (fogText == QString("exponential"))
    mWrenFogType = WR_SCENE_FOG_TYPE_EXPONENTIAL;
  else if (fogText == QString("exponential2"))
    mWrenFogType = WR_SCENE_FOG_TYPE_EXPONENTIAL2;
  else
    mWrenFogType = WR_SCENE_FOG_TYPE_LINEAR;

  if (mWrenFogType == WR_SCENE_FOG_TYPE_LINEAR && fogText != QString("linear"))
    parsingWarn(tr("Unknown 'fogType': \"%1\". Set to \"LINEAR\"").arg(mFogType->value()));

  if (areWrenObjectsInitialized()) {
    applyChangesToWren();
    emit modeChanged();
  }
}

void WbFog::updateVisibilityRange() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mVisibilityRange, 0.0))
    return;

  if (areWrenObjectsInitialized())
    applyChangesToWren();
}

void WbFog::applyChangesToWren() {
  assert(isFirstInstance());

  WrSceneFogType fogType = mWrenFogType;
  float density = 0.0f;
  if (mVisibilityRange->value() > 0.0)
    density = 1.0 / mVisibilityRange->value();
  else
    fogType = WR_SCENE_FOG_TYPE_NONE;
  const float color[] = {static_cast<float>(mColor->red()), static_cast<float>(mColor->green()),
                         static_cast<float>(mColor->blue())};

  wr_scene_set_fog(wr_scene_get_instance(), fogType, WR_SCENE_FOG_DEPTH_TYPE_POINT, color, density, 0.0f,
                   mVisibilityRange->value());
}

QStringList WbFog::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "color"
         << "fogType"
         << "visibilityRange";
  return fields;
}
