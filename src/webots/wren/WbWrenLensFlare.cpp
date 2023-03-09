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

#include "WbWrenLensFlare.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/post_processing_effect.h>
#include <wren/viewport.h>

WbWrenLensFlare::WbWrenLensFlare() :
  WbWrenAbstractPostProcessingEffect(),
  mLensFlarePass(NULL),
  mBlurPass(NULL),
  mBlendPass(NULL),
  mTransparency(0.0f),
  mScale(0.0f),
  mBias(0.0f),
  mDispersal(0.0f),
  mHaloWidth(0.0f),
  mChromaDistortion(0.0f),
  mSamples(0),
  mBlurIterations(0) {
}

void WbWrenLensFlare::setup(WrViewport *viewport) {
  if (mWrenPostProcessingEffect) {
    // In case we want to update the viewport, the old postProcessingEffect has to be removed first
    if (mWrenViewport == viewport)
      wr_viewport_remove_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);

    wr_post_processing_effect_delete(mWrenPostProcessingEffect);
  }

  mWrenViewport = viewport;

  const float width = wr_viewport_get_width(mWrenViewport);
  const float height = wr_viewport_get_height(mWrenViewport);

  mWrenPostProcessingEffect = WbWrenPostProcessingEffects::lensFlare(width, height, mBlurIterations);

  mLensFlarePass = wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "LensFlare");
  mBlurPass = wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "LensFlareBlur");
  mBlendPass = wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "LensFlareBlend");

  applyParametersToWren();

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_viewport_add_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);
  wr_post_processing_effect_setup(mWrenPostProcessingEffect);

  WbWrenOpenGlContext::doneWren();
}

void WbWrenLensFlare::setTransparency(float transparency) {
  mTransparency = transparency;
  applyParametersToWren();
}

void WbWrenLensFlare::setScale(float scale) {
  mScale = scale;
  applyParametersToWren();
}

void WbWrenLensFlare::setDispersal(float dispersal) {
  mDispersal = dispersal;
  applyParametersToWren();
}

void WbWrenLensFlare::setBias(float bias) {
  mBias = bias;
  applyParametersToWren();
}

void WbWrenLensFlare::setHaloWidth(float haloWidth) {
  mHaloWidth = haloWidth;
  applyParametersToWren();
}

void WbWrenLensFlare::setChromaDistortion(float chromaDistortion) {
  mChromaDistortion = chromaDistortion;
  applyParametersToWren();
}

void WbWrenLensFlare::setSamples(int samples) {
  mSamples = samples;
  applyParametersToWren();
}

void WbWrenLensFlare::setBlurIterations(int iterations) {
  mBlurIterations = iterations;
  applyParametersToWren();
}

void WbWrenLensFlare::applyParametersToWren() {
  if (!mWrenPostProcessingEffect)
    return;

  wr_post_processing_effect_pass_set_program_parameter(mBlendPass, "transparency",
                                                       reinterpret_cast<const char *>(&mTransparency));
  wr_post_processing_effect_pass_set_program_parameter(mLensFlarePass, "uScale", reinterpret_cast<const char *>(&mScale));
  wr_post_processing_effect_pass_set_program_parameter(mLensFlarePass, "uGhostDispersal",
                                                       reinterpret_cast<const char *>(&mDispersal));
  wr_post_processing_effect_pass_set_program_parameter(mLensFlarePass, "uBias", reinterpret_cast<const char *>(&mBias));
  wr_post_processing_effect_pass_set_program_parameter(mLensFlarePass, "uHaloWidth",
                                                       reinterpret_cast<const char *>(&mHaloWidth));
  wr_post_processing_effect_pass_set_program_parameter(mLensFlarePass, "uDistortion",
                                                       reinterpret_cast<const char *>(&mChromaDistortion));
  wr_post_processing_effect_pass_set_program_parameter(mLensFlarePass, "uSamples", reinterpret_cast<const char *>(&mSamples));
  wr_post_processing_effect_pass_set_iteration_count(mBlurPass, mBlurIterations);
}
