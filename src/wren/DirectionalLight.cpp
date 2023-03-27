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

#include "DirectionalLight.hpp"

#include "Scene.hpp"
#include "ShadowVolumeCaster.hpp"

#include <wren/directional_light.h>

namespace wren {

  void DirectionalLight::setDirection(const glm::vec3 &direction) {
    if (mDirection == direction)
      return;

    mDirection = direction;

    for (auto entry : mShadowListeners)
      entry.second->notifyLightDirty(this);
  }

  DirectionalLight::DirectionalLight() : mDirection(gVec3UnitZ) {
    Scene::instance()->addLight(this);
  }

  DirectionalLight::~DirectionalLight() {
    Scene::instance()->removeLight(this);

    // Needs to be done for each subclass of LightNode because virtual methods can't be called in the destructor
    if (on() && castShadows())
      --LightNode::cActiveLightsCastingShadows[TYPE_DIRECTIONAL];
  }

}  // namespace wren

// C interface implementation
WrDirectionalLight *wr_directional_light_new() {
  return reinterpret_cast<WrDirectionalLight *>(wren::DirectionalLight::createDirectionalLight());
}

void wr_directional_light_set_color(WrDirectionalLight *light, const float *color) {
  reinterpret_cast<wren::LightNode *>(light)->setColor(glm::make_vec3(color));
}

void wr_directional_light_set_intensity(WrDirectionalLight *light, float intensity) {
  reinterpret_cast<wren::LightNode *>(light)->setIntensity(intensity);
}

void wr_directional_light_set_ambient_intensity(WrDirectionalLight *light, float ambient_intensity) {
  reinterpret_cast<wren::LightNode *>(light)->setAmbientIntensity(ambient_intensity);
}

void wr_directional_light_set_on(WrDirectionalLight *light, bool on) {
  reinterpret_cast<wren::LightNode *>(light)->setOn(on);
}

void wr_directional_light_set_cast_shadows(WrDirectionalLight *light, bool cast_shadows) {
  reinterpret_cast<wren::LightNode *>(light)->setCastShadows(cast_shadows);
}

void wr_directional_light_set_direction(WrDirectionalLight *light, float *direction) {
  reinterpret_cast<wren::DirectionalLight *>(light)->setDirection(glm::make_vec3(direction));
}
