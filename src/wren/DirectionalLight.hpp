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

#ifndef DIRECTIONAL_LIGHT_HPP
#define DIRECTIONAL_LIGHT_HPP

#include "Constants.hpp"
#include "LightNode.hpp"

namespace wren {

  // Represents an infinitely far-away light source.
  // Although it inherits from Node and can be attached to a Transform, the effect of doing so is nil.
  class DirectionalLight : public LightNode {
  public:
    // Encapsulate memory management
    static DirectionalLight *createDirectionalLight() { return new DirectionalLight(); }

    const glm::vec3 &direction() const { return mDirection; }

    void setDirection(const glm::vec3 &direction);

    LightNode::Type type() override { return TYPE_DIRECTIONAL; }
    void updateFromParent() override {}

  private:
    DirectionalLight();
    virtual ~DirectionalLight();

    void recomputeAabb() const override { mAabb = gAabbInf; }

    void recomputeBoundingSphere() const override { mBoundingSphere = gSphereInf; }

    glm::vec3 mDirection;
  };

}  // namespace wren

#endif  // DIRECTIONAL_LIGHT_HPP
