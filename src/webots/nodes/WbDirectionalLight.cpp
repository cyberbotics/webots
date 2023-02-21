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

#include "WbDirectionalLight.hpp"
#include "WbField.hpp"
#include "WbSFBool.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbSFVector3.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/directional_light.h>
#include <wren/node.h>
#include <wren/scene.h>

void WbDirectionalLight::init() {
  mWrenLight = NULL;
  mDirection = findSFVector3("direction");
}

WbDirectionalLight::WbDirectionalLight(WbTokenizer *tokenizer) : WbLight("DirectionalLight", tokenizer) {
  init();
  if (tokenizer == NULL)
    mDirection->setValueNoSignal(0, -1, 0);
}

WbDirectionalLight::WbDirectionalLight(const WbDirectionalLight &other) : WbLight(other) {
  init();
}

WbDirectionalLight::WbDirectionalLight(const WbNode &other) : WbLight(other) {
  init();
}

void WbDirectionalLight::preFinalize() {
  WbLight::preFinalize();

  updateDirection();
}

void WbDirectionalLight::postFinalize() {
  WbLight::postFinalize();

  connect(mDirection, &WbSFVector3::changed, this, &WbDirectionalLight::updateDirection);
}

WbDirectionalLight::~WbDirectionalLight() {
  if (areWrenObjectsInitialized())
    wr_node_delete(WR_NODE(mWrenLight));
}

void WbDirectionalLight::createWrenObjects() {
  mWrenLight = wr_directional_light_new();
  WbLight::createWrenObjects();  // sets properties common to all lights
  applyLightDirectionToWren();
}

void WbDirectionalLight::updateDirection() {
  if (areWrenObjectsInitialized())
    applyLightDirectionToWren();
  emit directionChanged();
}

void WbDirectionalLight::applyLightIntensityToWren() {
  wr_directional_light_set_intensity(mWrenLight, mIntensity->value());
}

void WbDirectionalLight::applyLightColorToWren() {
  const float color[] = {static_cast<float>(mColor->red()), static_cast<float>(mColor->green()),
                         static_cast<float>(mColor->blue())};
  wr_directional_light_set_color(mWrenLight, color);
}

void WbDirectionalLight::applyLightVisibilityToWren() {
  wr_directional_light_set_on(mWrenLight, mOn->value());

  const int maxCount = wr_config_get_max_active_directional_light_count();
  const int activeCount = wr_scene_get_active_directional_light_count(wr_scene_get_instance());
  if (activeCount == maxCount)
    parsingWarn(
      tr("Maximum number of directional lights (%1) has been reached, newly added lights won't be rendered.").arg(maxCount));
}

void WbDirectionalLight::applyLightShadowsToWren() {
  wr_directional_light_set_cast_shadows(mWrenLight, mCastShadows->value());
}

void WbDirectionalLight::applyLightDirectionToWren() {
  float d[] = {static_cast<float>(mDirection->x()), static_cast<float>(mDirection->y()), static_cast<float>(mDirection->z())};
  wr_directional_light_set_direction(mWrenLight, d);
}

const WbVector3 &WbDirectionalLight::direction() const {
  return mDirection->value();
}

QStringList WbDirectionalLight::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "direction" << WbLight::fieldsToSynchronizeWithX3D();

  return fields;
}
