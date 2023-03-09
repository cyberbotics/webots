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

#ifndef SHADOW_VOLUME_CASTER_HPP
#define SHADOW_VOLUME_CASTER_HPP

#include "LightNode.hpp"
#include "Primitive.hpp"

#include <unordered_map>
#include <vector>

namespace wren {

  class DirectionalLight;
  class PointLight;
  class Renderable;
  class ShaderProgram;

  class ShadowVolumeCaster {
  public:
    struct ShadowVolume {
      ShadowVolume() :
        mIndexCountSides(0),
        mIndexCountCaps(0),
        mIsDirty(true),
        mGlNameSidesIndexBuffer(0),
        mGlNameCapsIndexBuffer(0),
        mAabb(gAabbInf),
        mAabbDirty(true) {}

      int mIndexCountSides;
      int mIndexCountCaps;
      bool mIsDirty;

      unsigned int mGlNameSidesIndexBuffer;
      unsigned int mGlNameCapsIndexBuffer;

      primitive::Aabb mAabb;
      bool mAabbDirty;
    };

    explicit ShadowVolumeCaster(Renderable *renderable);
    ~ShadowVolumeCaster();

    Renderable *renderable() const { return mRenderable; }
    const primitive::Aabb &aabb(LightNode *light);

    void notifyRenderableDirty();                                    // called when Renderable transform changes
    void notifyLightDirty(LightNode *light, bool aabbOnly = false);  // called when Light transform changes
    void notifyLightRemoved(LightNode *light);                       // called when light is destroyed or stops casting shadows

    void computeSilhouette(LightNode *light, bool computeCaps = true);
    void renderSides(LightNode *light) const;
    void renderCaps(LightNode *light) const;

  private:
    Renderable *mRenderable;
    bool mHasCaps;

    std::unordered_map<LightNode *, ShadowVolume> mShadowVolumes;
  };

}  // namespace wren

#endif  // SHADOW_VOLUME_CASTER_HPP
