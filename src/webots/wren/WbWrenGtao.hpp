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

#ifndef WB_WREN_GTAO_HPP
#define WB_WREN_GTAO_HPP

#include "WbWrenAbstractPostProcessingEffect.hpp"

struct WrPostProcessingEffectPass;

class WbWrenGtao : public WbWrenAbstractPostProcessingEffect {
public:
  WbWrenGtao();

  void setup(WrViewport *viewport) override;
  void detachFromViewport() override;

  void setNear(float nearValue);
  void setFar(float farValue);
  void setFov(float fov);
  void setRadius(float radius);
  void setQualityLevel(int qualityLevel);
  void setHalfResolution(bool halfResolution) { mHalfResolution = halfResolution; }
  void setFlipNormalY(bool flip);
  void copyNewInverseViewMatrix(const float *inverseViewMatrix);
  void applyOldInverseViewMatrixToWren();

private:
  void applyParametersToWren() override;

  WrPostProcessingEffectPass *mGtaoPass;
  WrPostProcessingEffectPass *mTemporalPass;
  float mNear, mFar, mFov;
  float mRadius;
  float mClipInfo[4];
  float mRotations[6];
  float mOffsets[4];
  float mParams[4];
  bool mHalfResolution;
  bool mFlipNormalY;
  float mPreviousInverseViewMatrix[16];
  int mFrameCounter;
};

#endif  // WB_WREN_GTAO_HPP
