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

#include "PositionalLight.hpp"

#include "Debug.hpp"
#include "ShadowVolumeCaster.hpp"
#include "Transform.hpp"

namespace wren {

  PositionalLight::PositionalLight() :
    mIsPositionDirty(true),
    mPositionAbsolute(gVec3Zeros),
    mPositionRelative(gVec3Zeros),
    mRadius(100.0f),
    mAttenuationConstant(1.0f),
    mAttenuationLinear(0.0f),
    mAttenuationQuadratic(0.0f) {
  }

  void PositionalLight::setPosition(const glm::vec3 &position) {
    mPositionRelative = position;
    mIsPositionDirty = true;
    mIsAabbDirty = mIsBoundingSphereDirty = true;

    for (auto entry : mShadowListeners)
      entry.second->notifyLightDirty(this);
  }

  void PositionalLight::setRadius(float radius) {
    mRadius = radius;
    for (auto entry : mShadowListeners)
      entry.second->notifyLightDirty(this, true);
  }

  void PositionalLight::setAttenuation(float attenuationConstant, float attenuationLinear, float attenuationQuadratic) {
    mAttenuationConstant = attenuationConstant;
    mAttenuationLinear = attenuationLinear;
    mAttenuationQuadratic = attenuationQuadratic;
  }

}  // namespace wren
