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

#include "PointLight.hpp"

#include "Scene.hpp"

#include <wren/point_light.h>

namespace wren {

  PointLight::PointLight() {
    Scene::instance()->addLight(this);
  }

  PointLight::~PointLight() {
    Scene::instance()->removeLight(this);

    // Needs to be done for each subclass of LightNode because virtual methods can't be called in the destructor
    if (on() && castShadows())
      --LightNode::cActiveLightsCastingShadows[TYPE_POINT];
  }

}  // namespace wren

// C interface implementation
WrPointLight *wr_point_light_new() {
  return reinterpret_cast<WrPointLight *>(wren::PointLight::createPointLight());
}

void wr_point_light_set_color(WrPointLight *light, const float *color) {
  reinterpret_cast<wren::PointLight *>(light)->setColor(glm::make_vec3(color));
}

void wr_point_light_set_intensity(WrPointLight *light, float intensity) {
  reinterpret_cast<wren::PointLight *>(light)->setIntensity(intensity);
}

void wr_point_light_set_ambient_intensity(WrPointLight *light, float ambient_intensity) {
  reinterpret_cast<wren::PointLight *>(light)->setAmbientIntensity(ambient_intensity);
}

void wr_point_light_set_on(WrPointLight *light, bool on) {
  reinterpret_cast<wren::PointLight *>(light)->setOn(on);
}

void wr_point_light_set_cast_shadows(WrPointLight *light, bool cast_shadows) {
  reinterpret_cast<wren::PointLight *>(light)->setCastShadows(cast_shadows);
}

void wr_point_light_set_position_relative(WrPointLight *light, const float *position) {
  reinterpret_cast<wren::PointLight *>(light)->setPosition(glm::make_vec3(position));
}

void wr_point_light_set_radius(WrPointLight *light, float radius) {
  reinterpret_cast<wren::PointLight *>(light)->setRadius(radius);
}

void wr_point_light_set_attenuation(WrPointLight *light, float attenuation_constant, float attenuation_linear,
                                    float attenuation_quadratic) {
  reinterpret_cast<wren::PointLight *>(light)->setAttenuation(attenuation_constant, attenuation_linear, attenuation_quadratic);
}
