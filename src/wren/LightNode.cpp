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

#include "LightNode.hpp"

#include "Camera.hpp"
#include "ColorUtils.hpp"
#include "Config.hpp"
#include "Debug.hpp"
#include "DirectionalLight.hpp"
#include "FrameBuffer.hpp"
#include "GlState.hpp"
#include "PointLight.hpp"
#include "Scene.hpp"
#include "ShadowVolumeCaster.hpp"
#include "SpotLight.hpp"
#include "TextureRtt.hpp"
#include "Transform.hpp"
#include "UniformBuffer.hpp"
#include "Viewport.hpp"

namespace wren {

  glm::vec3 LightNode::cGlobalAmbientIntensity;
  GlslLayout::Lights LightNode::cActiveLights;
  std::vector<int> LightNode::cActiveLightsCastingShadows(LightNode::TYPE_COUNT, 0);

  void LightNode::updateUniforms() {
    // DEBUG("LightNode::updateUniforms");
    const std::vector<DirectionalLight *> &directionalLights = Scene::instance()->directionalLights();
    const std::vector<PointLight *> &pointLights = Scene::instance()->pointLights();
    const std::vector<SpotLight *> &spotLights = Scene::instance()->spotLights();
    const Camera *camera = Scene::instance()->currentViewport()->camera();

    LightNode::cActiveLights.mAmbientLight = glm::vec4(LightNode::ambientLight(), 1.0f);
    // debug::printVec4(LightNode::cActiveLights.mAmbientLight, "ambient light");

    glm::int32 lightCount = 0;
    for (DirectionalLight *l : directionalLights) {
      if (!l->parent() || l->parent()->isVisible()) {
        LightNode::cActiveLights.mDirectionalLights[lightCount].mColorAndIntensity = glm::vec4(l->color(), l->intensity()),
        LightNode::cActiveLights.mDirectionalLights[lightCount].mDirection =
          glm::normalize(camera->view() * glm::vec4(l->direction(), 0.0f));

        // debug::printVec3(l->direction(), "light direction");

        ++lightCount;
      }

      if (lightCount >= gMaxActiveDirectionalLights)
        break;
    }
    LightNode::cActiveLights.mLightCount[0] = lightCount;
    // DEBUG(lightCount << " active directional lights");

    lightCount = 0;
    for (PointLight *l : pointLights) {
      if (!l->parent() || l->parent()->isVisible()) {
        LightNode::cActiveLights.mPointLights[lightCount].mColorAndIntensity = glm::vec4(l->color(), l->intensity()),
        LightNode::cActiveLights.mPointLights[lightCount].mPosition = camera->view() * glm::vec4(l->position(), 1.0);
        LightNode::cActiveLights.mPointLights[lightCount].mAttenuationAndRadius =
          glm::vec4(l->attenuationConstant(), l->attenuationLinear(), l->attenuationQuadratic(), l->radius());

        // debug::printVec3(l->position(), "light position");
        // debug::printVec4(LightNode::cActiveLights.mPointLights[lightCount].mAttenuationAndRadius, "attenuation and radius");

        ++lightCount;
      }

      if (lightCount >= gMaxActivePointLights)
        break;
    }

    // DEBUG(lightCount << " active point lights");

    LightNode::cActiveLights.mLightCount[1] = lightCount;
    lightCount = 0;
    for (SpotLight *l : spotLights) {
      if (!l->parent() || l->parent()->isVisible()) {
        LightNode::cActiveLights.mSpotLights[lightCount].mColorAndIntensity = glm::vec4(l->color(), l->intensity()),
        LightNode::cActiveLights.mSpotLights[lightCount].mPosition = camera->view() * glm::vec4(l->position(), 1.0f);
        LightNode::cActiveLights.mSpotLights[lightCount].mDirection =
          glm::normalize(camera->view() * glm::vec4(l->direction(), 0.0f));
        LightNode::cActiveLights.mSpotLights[lightCount].mAttenuationAndRadius =
          glm::vec4(l->attenuationConstant(), l->attenuationLinear(), l->attenuationQuadratic(), l->radius());
        LightNode::cActiveLights.mSpotLights[lightCount].mSpotParams =
          glm::vec4(l->beamWidth(), l->cutOffAngle(), 1.0f / (l->cutOffAngle() - l->beamWidth()), 0.0f);

        // debug::printVec3(l->position(), "light position");
        // debug::printVec3(l->direction(), "light direction");
        // debug::printVec4(LightNode::cActiveLights.mSpotLights[lightCount].mAttenuationAndRadius, "attenuation and radius");
        // debug::printVec4(LightNode::cActiveLights.mSpotLights[lightCount].mSpotParams, "spotlight params");

        ++lightCount;
      }

      if (lightCount >= gMaxActiveSpotLights)
        break;
    }
    LightNode::cActiveLights.mLightCount[2] = lightCount;

    // DEBUG(lightCount << " active spot lights");

    glstate::uniformBuffer(WR_GLSL_LAYOUT_UNIFORM_BUFFER_LIGHTS)->writeValue(&LightNode::cActiveLights);
  }

  void LightNode::setOn(bool on) {
    if (on == mOn)
      return;

    Scene::instance()->removeLight(this);
    mOn = on;
    Scene::instance()->addLight(this);

    if (mCastShadows) {
      if (on)
        enableShadowCasting();
      else
        disableShadowCasting();
    }
  }

  void LightNode::setColor(const glm::vec3 &color) {
    mColor = colorutils::srgbToLinear(glm::vec4(color, 1.0));
  }

  void LightNode::setCastShadows(bool castShadows) {
    if (mCastShadows == castShadows)
      return;

    mCastShadows = castShadows;
    if (mOn) {
      if (castShadows)
        enableShadowCasting();
      else
        disableShadowCasting();
    }
  }

  LightNode::LightNode() : mOn(true), mCastShadows(false), mIntensity(1.0f), mAmbientIntensity(0.0f), mColor(gVec3Ones) {
  }

  LightNode::~LightNode() {
    if (mOn && mCastShadows) {
      for (auto entry : mShadowListeners)
        entry.second->notifyLightRemoved(this);
    }
  }

  void LightNode::enableShadowCasting() {
    ++LightNode::cActiveLightsCastingShadows[this->type()];
  }

  void LightNode::disableShadowCasting() {
    --LightNode::cActiveLightsCastingShadows[this->type()];
    for (auto entry : mShadowListeners)
      entry.second->notifyLightRemoved(this);

    mShadowListeners.clear();
  }

}  // namespace wren
