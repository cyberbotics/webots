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

#ifndef LIGHT_NODE_HPP
#define LIGHT_NODE_HPP

#include "Constants.hpp"
#include "GlslLayout.hpp"
#include "Node.hpp"

#include <numeric>
#include <vector>

namespace wren {

  class Camera;
  class DirectionalLight;
  class FrameBuffer;
  class PointLight;
  class ShaderProgram;
  class ShadowVolumeCaster;
  class SpotLight;
  class TextureRtt;

  // Base class for all light types.
  // Stores parameters common to all light sources and provides a static method for
  // updating the Lights uniform buffer with information about all active lights.
  class LightNode : public Node {
  public:
    enum Type { TYPE_DIRECTIONAL, TYPE_POINT, TYPE_SPOT, TYPE_COUNT };

    static void setAmbientLight(const glm::vec3 &ambientLight) { cGlobalAmbientIntensity = ambientLight; }
    static int activeLightsCastingShadows() {
      return std::accumulate(cActiveLightsCastingShadows.begin(), cActiveLightsCastingShadows.end(), 0);
    }
    static int activeLightsCastingShadows(Type type) { return cActiveLightsCastingShadows[type]; }

    // Updates the global uniform buffer with the params of all active lights in the scene
    static void updateUniforms();
    static const glm::vec3 &ambientLight() { return cGlobalAmbientIntensity; }

    void setOn(bool on);
    void setCastShadows(bool castShadows);
    void setColor(const glm::vec3 &color);
    void setIntensity(float intensity) { mIntensity = intensity; }
    void setAmbientIntensity(float ambientIntensity) { mAmbientIntensity = ambientIntensity; }

    void registerShadowListener(ShadowVolumeCaster *listener) { mShadowListeners.emplace(listener, listener); }
    void unregisterShadowListener(ShadowVolumeCaster *listener) { mShadowListeners.erase(listener); }

    const glm::vec3 &color() const { return mColor; }
    float intensity() const { return mIntensity; }
    float ambientIntensity() const { return mAmbientIntensity; }
    bool on() const { return mOn; }
    bool castShadows() const { return mCastShadows; }

    virtual LightNode::Type type() = 0;

  protected:
    LightNode();
    virtual ~LightNode();

    static std::vector<int> cActiveLightsCastingShadows;

    std::unordered_map<ShadowVolumeCaster *, ShadowVolumeCaster *> mShadowListeners;

  private:
    void enableShadowCasting();
    void disableShadowCasting();

    static glm::vec3 cGlobalAmbientIntensity;
    static GlslLayout::Lights cActiveLights;

    bool mOn;
    bool mCastShadows;
    float mIntensity;
    float mAmbientIntensity;
    glm::vec3 mColor;
  };

}  // namespace wren

#endif  // LIGHT_NODE_HPP
