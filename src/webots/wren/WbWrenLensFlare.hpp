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

#ifndef WB_WREN_LENS_FLARE_HPP
#define WB_WREN_LENS_FLARE_HPP

#include "WbWrenAbstractPostProcessingEffect.hpp"

struct WrPostProcessingEffectPass;

class WbWrenLensFlare : public WbWrenAbstractPostProcessingEffect {
public:
  WbWrenLensFlare();

  void setup(WrViewport *viewport) override;

  void setTransparency(float transparency);
  void setScale(float scale);
  void setDispersal(float dispersal);
  void setBias(float bias);
  void setHaloWidth(float haloWidth);
  void setChromaDistortion(float chromaDistortion);
  void setSamples(int samples);
  void setBlurIterations(int iterations);

private:
  void applyParametersToWren() override;

  WrPostProcessingEffectPass *mLensFlarePass;
  WrPostProcessingEffectPass *mBlurPass;
  WrPostProcessingEffectPass *mBlendPass;

  float mTransparency;
  float mScale;
  float mBias;
  float mDispersal;
  float mHaloWidth;
  float mChromaDistortion;
  int mSamples;
  int mBlurIterations;
};

#endif  // WB_WREN_LENS_FLARE_HPP
