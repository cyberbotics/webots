// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbWrenDepthOfField.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/post_processing_effect.h>
#include <wren/texture.h>
#include <wren/viewport.h>

WbWrenDepthOfField::WbWrenDepthOfField() :
  WbWrenAbstractPostProcessingEffect(),
  mTextureSize{0.0f, 0.0f},
  mCameraParams{0.0f, 0.0f},
  mDepthOfFieldParams{0.0f, 0.0f, 0.0f, 0.0f},
  mColorTexture(NULL),
  mDepthTexture(NULL) {
}

void WbWrenDepthOfField::setup(WrViewport *viewport) {
  if (mWrenPostProcessingEffect) {
    // In case we want to update the viewport, the old postProcessingEffect has to be removed first
    if (mWrenViewport == viewport)
      wr_viewport_remove_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);

    wr_post_processing_effect_delete(mWrenPostProcessingEffect);
  }

  mWrenViewport = viewport;

  const float width = wr_viewport_get_width(mWrenViewport);
  const float height = wr_viewport_get_height(mWrenViewport);

  mWrenPostProcessingEffect = WbWrenPostProcessingEffects::depthOfField(width, height, mTextureSize[0], mTextureSize[1],
                                                                        mTextureFormat, mColorTexture, mDepthTexture);

  applyParametersToWren();

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_viewport_add_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);
  wr_post_processing_effect_setup(mWrenPostProcessingEffect);

  WbWrenOpenGlContext::doneWren();
  mHasBeenSetup = true;
}

void WbWrenDepthOfField::setTextureWidth(float width) {
  mTextureSize[0] = width;
}

void WbWrenDepthOfField::setTextureHeight(float height) {
  mTextureSize[1] = height;
}

void WbWrenDepthOfField::setColorTexture(WrTexture *colorTexture) {
  mColorTexture = colorTexture;
}

void WbWrenDepthOfField::setDepthTexture(WrTexture *depthTexture) {
  mDepthTexture = depthTexture;
}

void WbWrenDepthOfField::setCameraParams(float nearValue, float farValue) {
  mCameraParams[0] = nearValue;
  mCameraParams[1] = farValue;

  applyParametersToWren();
}

void WbWrenDepthOfField::setDepthOfFieldParams(float minLength, float focalLength, float maxLength, float blurCutoff) {
  mDepthOfFieldParams[0] = minLength;
  mDepthOfFieldParams[1] = focalLength;
  mDepthOfFieldParams[2] = maxLength;
  mDepthOfFieldParams[3] = blurCutoff;
}

void WbWrenDepthOfField::applyParametersToWren() {
  if (!mWrenPostProcessingEffect)
    return;

  wr_post_processing_effect_pass_set_program_parameter(
    wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "LensFocus_Dof"), "cameraParams",
    reinterpret_cast<const char *>(&mCameraParams));
  wr_post_processing_effect_pass_set_program_parameter(
    wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "LensFocus_Dof"), "dofParams",
    reinterpret_cast<const char *>(&mDepthOfFieldParams));
  wr_post_processing_effect_pass_set_program_parameter(
    wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "LensFocus_Dof"), "blurTextureSize",
    reinterpret_cast<const char *>(&mTextureSize));
}
