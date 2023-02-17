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

#ifndef SPOT_LIGHT_HPP
#define SPOT_LIGHT_HPP

#include "Constants.hpp"
#include "PositionalLight.hpp"

namespace wren {

  // Represents a light source which radiates with full intensity inside a cone surrounding
  // its direction parameter, and with falling intensity inside a larger cone in the same direction.
  // Currently, the orientation isn't influenced by the orientation of the containing Transform.
  class SpotLight : public PositionalLight {
  public:
    // Encapsulate memory management
    static SpotLight *createSpotLight() { return new SpotLight(); }

    void setDirection(const glm::vec3 &direction) {
      mDirectionRelative = direction;
      mIsDirectionDirty = true;
    }

    void setBeamWidth(float beamWidth) { mCutOffInner = beamWidth; }
    void setCutOffAngle(float cutOffAngle) { mCutOffOuter = cutOffAngle; }

    const glm::vec3 &direction() const {
      update();
      return mDirectionAbsolute;
    }

    float beamWidth() const { return mCutOffInner; }
    float cutOffAngle() const { return mCutOffOuter; }

    void setMatrixDirty() const override {
      PositionalLight::setMatrixDirty();
      mIsDirectionDirty = true;
    }

    LightNode::Type type() override { return TYPE_SPOT; }

    void update() const override {
      PositionalLight::update();

      if (mIsDirectionDirty) {
        if (mParent)
          mDirectionAbsolute = mParent->orientation() * mDirectionRelative;
        else
          mDirectionAbsolute = mDirectionRelative;

        mIsDirectionDirty = false;
      }
    }

  private:
    SpotLight();
    virtual ~SpotLight();

    mutable bool mIsDirectionDirty;
    mutable glm::vec3 mDirectionAbsolute;

    glm::vec3 mDirectionRelative;
    float mCutOffInner;
    float mCutOffOuter;
  };

}  // namespace wren

#endif  // SPOT_LIGHT_HPP
