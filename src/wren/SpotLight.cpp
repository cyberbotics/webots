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

#include "SpotLight.hpp"

#include "Scene.hpp"

#include <wren/spot_light.h>

namespace wren {

  SpotLight::SpotLight() :
    mIsDirectionDirty(true),
    mDirectionRelative(gVec3UnitZ),
    mCutOffInner(glm::quarter_pi<float>()),
    mCutOffOuter(glm::quarter_pi<float>()) {
    Scene::instance()->addLight(this);
  }

  SpotLight::~SpotLight() {
    Scene::instance()->removeLight(this);

    // Needs to be done for each subclass of LightNode because virtual methods can't be called in the destructor
    if (on() && castShadows())
      --LightNode::cActiveLightsCastingShadows[TYPE_SPOT];
  }

}  // namespace wren

// C interface implementation
WrSpotLight *wr_spot_light_new() {
  return reinterpret_cast<WrSpotLight *>(wren::SpotLight::createSpotLight());
}

void wr_spot_light_set_color(WrSpotLight *light, const float *color) {
  reinterpret_cast<wren::SpotLight *>(light)->setColor(glm::make_vec3(color));
}

void wr_spot_light_set_intensity(WrSpotLight *light, float intensity) {
  reinterpret_cast<wren::LightNode *>(light)->setIntensity(intensity);
}

void wr_spot_light_set_ambient_intensity(WrSpotLight *light, float ambient_intensity) {
  reinterpret_cast<wren::LightNode *>(light)->setAmbientIntensity(ambient_intensity);
}

void wr_spot_light_set_on(WrSpotLight *light, bool on) {
  reinterpret_cast<wren::LightNode *>(light)->setOn(on);
}

void wr_spot_light_set_cast_shadows(WrSpotLight *light, bool cast_shadows) {
  reinterpret_cast<wren::LightNode *>(light)->setCastShadows(cast_shadows);
}

void wr_spot_light_set_attenuation(WrSpotLight *light, float attenuation_constant, float attenuation_linear,
                                   float attenuation_quadratic) {
  reinterpret_cast<wren::PositionalLight *>(light)->setAttenuation(attenuation_constant, attenuation_linear,
                                                                   attenuation_quadratic);
}

void wr_spot_light_set_position_relative(WrSpotLight *light, const float *position) {
  reinterpret_cast<wren::SpotLight *>(light)->setPosition(glm::make_vec3(position));
}

void wr_spot_light_set_radius(WrSpotLight *light, float radius) {
  reinterpret_cast<wren::SpotLight *>(light)->setRadius(radius);
}

void wr_spot_light_set_direction(WrSpotLight *light, const float *direction) {
  reinterpret_cast<wren::SpotLight *>(light)->setDirection(glm::make_vec3(direction));
}

void wr_spot_light_set_beam_width(WrSpotLight *light, float beam_width) {
  reinterpret_cast<wren::SpotLight *>(light)->setBeamWidth(beam_width);
}

void wr_spot_light_set_cutoff_angle(WrSpotLight *light, float cutoff_angle) {
  reinterpret_cast<wren::SpotLight *>(light)->setCutOffAngle(cutoff_angle);
}
