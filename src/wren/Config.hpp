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

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "Primitive.hpp"

namespace wren {

  class Camera;
  class ShaderProgram;

  namespace config {

    void enableShadows(bool enable);
    void enablePointSize(bool enable);
    void setLineScale(float lineScale);
    void setShowAabbs(bool show);
    void setShowShadowAabbs(bool show);
    void setShowBoundingSpheres(bool show);
    void setBoundingVolumeProgram(ShaderProgram *program);
    void setRequiresFlushAfterDraw(bool require);
    void setRequiresDepthBufferDistortion(bool require);
    void drawBoundingSphere(const primitive::Sphere &sphere);
    void drawAabb(const primitive::Aabb &aabb);

    bool showAabbs();
    bool showShadowAabbs();
    bool showBoundingSpheres();
    bool requiresFlushAfterDraw();
    bool requiresDepthBufferDistortion();
    bool areShadowsEnabled();
    bool isPointSizeEnabled();
    float lineScale();
    int maxActiveSpotLightCount();
    int maxActivePointLightCount();
    int maxActiveDirectionalLightCount();

    unsigned int maxVerticesPerMeshForShadowRendering();

    void cleanup();
  };  // namespace config

}  // namespace wren

#endif  // CONFIG_HPP
