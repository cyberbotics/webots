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

#ifndef WB_WREN_NOISE_MASK_HPP
#define WB_WREN_NOISE_MASK_HPP

#include "WbWrenAbstractPostProcessingEffect.hpp"

class WbWrenNoiseMask : public WbWrenAbstractPostProcessingEffect {
public:
  WbWrenNoiseMask();

  void setup(WrViewport *viewport) override;
  void setTexture(WrTexture *texture);
  void setTextureOffset(float x, float y);
  void setTextureFactor(float x, float y);

private:
  void applyParametersToWren() override;

  WrTexture *mTexture;
  float mTextureOffset[2];
  float mTextureFactor[2];
};

#endif  // WB_WREN_NOISE_MASK_HPP
