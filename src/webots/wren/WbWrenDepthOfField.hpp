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

#ifndef WB_WREN_DEPTH_OF_FIELD_HPP
#define WB_WREN_DEPTH_OF_FIELD_HPP

#include "WbWrenAbstractPostProcessingEffect.hpp"

class WbWrenDepthOfField : public WbWrenAbstractPostProcessingEffect {
public:
  WbWrenDepthOfField();

  void setup(WrViewport *viewport) override;
  void setTextureWidth(float width);
  void setTextureHeight(float height);
  void setColorTexture(WrTexture *colorTexture);
  void setDepthTexture(WrTexture *depthTexture);

  void setCameraParams(float nearValue, float farValue);
  void setDepthOfFieldParams(float minLength, float focalLength, float maxLength, float blurCutoff);

private:
  void applyParametersToWren() override;

  float mTextureSize[2];
  float mCameraParams[2];
  float mDepthOfFieldParams[4];
  WrTexture *mColorTexture;
  WrTexture *mDepthTexture;
};

#endif  // WB_WREN_DEPTH_OF_FIELD_HPP
