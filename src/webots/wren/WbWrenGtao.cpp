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

#include "WbWrenGtao.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/frame_buffer.h>
#include <wren/post_processing_effect.h>
#include <wren/texture.h>
#include <wren/viewport.h>

#include <cmath>

WbWrenGtao::WbWrenGtao() :
  WbWrenAbstractPostProcessingEffect(),
  mGtaoPass(NULL),
  mTemporalPass(NULL),
  mNear(0.0f),
  mFar(0.0f),
  mFov(0.78f),
  mRadius(2.0),
  mHalfResolution(false),
  mFlipNormalY(false),
  mFrameCounter(0) {
  mClipInfo[0] = mClipInfo[1] = mClipInfo[2] = mClipInfo[3] = 0.0f;
  mParams[0] = mParams[1] = mParams[2] = mParams[3] = 0.0f;

  mRotations[0] = 60.0f;
  mRotations[1] = 300.0f;
  mRotations[2] = 180.0f;
  mRotations[3] = 240.0f;
  mRotations[4] = 120.0f;
  mRotations[5] = 0.0f;

  mOffsets[0] = 0.0f;
  mOffsets[1] = 0.5f;
  mOffsets[2] = 0.25f;
  mOffsets[3] = 0.75f;

  mPreviousInverseViewMatrix[0] = 1.0f;
  mPreviousInverseViewMatrix[1] = 0.0f;
  mPreviousInverseViewMatrix[2] = 0.0f;
  mPreviousInverseViewMatrix[3] = 0.0f;
  mPreviousInverseViewMatrix[4] = 0.0f;
  mPreviousInverseViewMatrix[5] = 1.0f;
  mPreviousInverseViewMatrix[6] = 0.0f;
  mPreviousInverseViewMatrix[7] = 0.0f;
  mPreviousInverseViewMatrix[8] = 0.0f;
  mPreviousInverseViewMatrix[9] = 0.0f;
  mPreviousInverseViewMatrix[10] = 1.0f;
  mPreviousInverseViewMatrix[11] = 0.0f;
  mPreviousInverseViewMatrix[12] = 0.0f;
  mPreviousInverseViewMatrix[13] = 0.0f;
  mPreviousInverseViewMatrix[14] = 0.0f;
  mPreviousInverseViewMatrix[15] = 0.0f;
}

void WbWrenGtao::setup(WrViewport *viewport) {
  if (mWrenPostProcessingEffect) {
    // In case we want to update the viewport, the old postProcessingEffect has to be removed first
    if (mWrenViewport == viewport)
      wr_viewport_remove_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);

    wr_post_processing_effect_delete(mWrenPostProcessingEffect);
  }

  mWrenViewport = viewport;

  float width = wr_viewport_get_width(mWrenViewport);
  float height = wr_viewport_get_height(mWrenViewport);

  if (mHalfResolution) {
    width = width <= 1.0f ? 2.0f : width;
    height = height <= 1.0f ? 2.0f : height;
  }

  WrFrameBuffer *viewportFramebuffer = wr_viewport_get_frame_buffer(mWrenViewport);

  WrTexture *depthTexture = WR_TEXTURE(wr_frame_buffer_get_depth_texture(viewportFramebuffer));
  WrTexture *normalTexture = WR_TEXTURE(wr_frame_buffer_get_output_texture(viewportFramebuffer, 1));

  mWrenPostProcessingEffect = WbWrenPostProcessingEffects::gtao(width, height, WR_TEXTURE_INTERNAL_FORMAT_RGB16F, depthTexture,
                                                                normalTexture, mHalfResolution);

  mGtaoPass = wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "gtaoForwardPass");
  mTemporalPass = wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "temporalDenoise");

  applyParametersToWren();

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_viewport_set_ambient_occlusion_effect(mWrenViewport, mWrenPostProcessingEffect);
  wr_post_processing_effect_setup(mWrenPostProcessingEffect);

  WbWrenOpenGlContext::doneWren();

  mHasBeenSetup = true;
}

void WbWrenGtao::detachFromViewport() {
  if (mWrenViewport) {
    wr_viewport_set_ambient_occlusion_effect(mWrenViewport, NULL);
    wr_post_processing_effect_delete(mWrenPostProcessingEffect);
    mWrenPostProcessingEffect = NULL;
    mWrenViewport = NULL;
    mHasBeenSetup = false;
  }
}

void WbWrenGtao::setNear(float nearValue) {
  mNear = nearValue;
  applyParametersToWren();
}

void WbWrenGtao::setFar(float farValue) {
  mFar = farValue;
  applyParametersToWren();
}

void WbWrenGtao::setFov(float fov) {
  mFov = fov;
  applyParametersToWren();
}

void WbWrenGtao::setRadius(float radius) {
  mRadius = radius;
  applyParametersToWren();
}

void WbWrenGtao::setQualityLevel(int qualityLevel) {
  mParams[2] = 2 << (qualityLevel - 1);
  applyParametersToWren();
}

void WbWrenGtao::setFlipNormalY(bool flip) {
  mFlipNormalY = flip;

  applyParametersToWren();
}

void WbWrenGtao::copyNewInverseViewMatrix(const float *inverseViewMatrix) {
  memcpy(mPreviousInverseViewMatrix, inverseViewMatrix, sizeof(float) * 16);
}

void WbWrenGtao::applyOldInverseViewMatrixToWren() {
  if (!mWrenPostProcessingEffect)
    return;
  wr_post_processing_effect_pass_set_program_parameter(mTemporalPass, "previousInverseViewMatrix",
                                                       reinterpret_cast<const char *>(&mPreviousInverseViewMatrix));
}

void WbWrenGtao::applyParametersToWren() {
  if (!mWrenPostProcessingEffect)
    return;

  WbWrenOpenGlContext::makeWrenCurrent();
  mClipInfo[0] = mNear;
  mClipInfo[1] = mFar ? mFar : 1000000.0f;
  mClipInfo[2] = 0.5f * (wr_viewport_get_height(mWrenViewport) / (2.0f * tanf(mFov * 0.5f)));
  wr_post_processing_effect_pass_set_program_parameter(mGtaoPass, "clipInfo", reinterpret_cast<const char *>(&mClipInfo));

  mParams[0] = mRotations[mFrameCounter % 6] / 360.0f;
  mParams[1] = mOffsets[(mFrameCounter / 6) % 4];
  wr_post_processing_effect_pass_set_program_parameter(mGtaoPass, "params", reinterpret_cast<const char *>(&mParams));

  wr_post_processing_effect_pass_set_program_parameter(mGtaoPass, "radius", reinterpret_cast<const char *>(&mRadius));

  wr_post_processing_effect_pass_set_program_parameter(mGtaoPass, "flipNormalY", reinterpret_cast<const char *>(&mFlipNormalY));

  ++mFrameCounter;

  WbWrenOpenGlContext::doneWren();
}
