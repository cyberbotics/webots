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

#ifndef LIGHT_POSITIONAL_HPP
#define LIGHT_POSITIONAL_HPP

#include "Constants.hpp"
#include "LightNode.hpp"
#include "Transform.hpp"

namespace wren {

  // Base class for light sources which have a position.
  // Inherits from Node and can be attached to a Transform for positioning.
  class PositionalLight : public LightNode {
  public:
    const glm::vec3 &position() const {
      update();
      return mPositionAbsolute;
    }

    float radius() const { return mRadius; }
    float attenuationConstant() const { return mAttenuationConstant; }
    float attenuationLinear() const { return mAttenuationLinear; }
    float attenuationQuadratic() const { return mAttenuationQuadratic; }

    void setAttenuation(float attenuationConstant, float attenuationLinear, float attenuationQuadratic);

    virtual void setPosition(const glm::vec3 &position);
    virtual void setRadius(float radius);
    void setMatrixDirty() const override {
      Node::setMatrixDirty();
      mIsPositionDirty = true;
    }

    void update() const override {
      if (!mIsPositionDirty)
        return;

      if (mParent)
        mPositionAbsolute = mParent->position() + mParent->orientation() * (mParent->scale() * mPositionRelative);
      else
        mPositionAbsolute = mPositionRelative;
    }

  protected:
    PositionalLight();
    virtual ~PositionalLight() {}

  private:
    mutable bool mIsPositionDirty;
    mutable glm::vec3 mPositionAbsolute;

    glm::vec3 mPositionRelative;
    float mRadius;
    float mAttenuationConstant;
    float mAttenuationLinear;
    float mAttenuationQuadratic;
  };

}  // namespace wren

#endif  // LIGHT_POSITIONAL_HPP
