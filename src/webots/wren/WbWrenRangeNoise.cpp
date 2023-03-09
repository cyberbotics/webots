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

#include "WbWrenRangeNoise.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/post_processing_effect.h>
#include <wren/texture.h>
#include <wren/viewport.h>

WbWrenRangeNoise::WbWrenRangeNoise() :
  WbWrenAbstractPostProcessingEffect(),
  mTime(0.0f),
  mIntensity(0.0f),
  mMinRange(0.0f),
  mMaxRange(0.0f) {
}

void WbWrenRangeNoise::setup(WrViewport *viewport) {
  if (mWrenPostProcessingEffect) {
    // In case we want to update the viewport, the old postProcessingEffect has to be removed first
    if (mWrenViewport == viewport)
      wr_viewport_remove_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);

    wr_post_processing_effect_delete(mWrenPostProcessingEffect);
  }

  mWrenViewport = viewport;

  const float width = wr_viewport_get_width(mWrenViewport);
  const float height = wr_viewport_get_height(mWrenViewport);

  mWrenPostProcessingEffect = WbWrenPostProcessingEffects::rangeNoise(width, height, WR_TEXTURE_INTERNAL_FORMAT_R32F);

  applyParametersToWren();

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_viewport_add_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);
  wr_post_processing_effect_setup(mWrenPostProcessingEffect);

  WbWrenOpenGlContext::doneWren();
  mHasBeenSetup = true;
}

void WbWrenRangeNoise::setTime(float time) {
  mTime = time;

  applyParametersToWren();
}

void WbWrenRangeNoise::setIntensity(float intensity) {
  mIntensity = intensity;

  applyParametersToWren();
}

void WbWrenRangeNoise::setMinRange(float minRange) {
  mMinRange = minRange;

  applyParametersToWren();
}

void WbWrenRangeNoise::setMaxRange(float maxRange) {
  mMaxRange = maxRange;

  applyParametersToWren();
}

void WbWrenRangeNoise::applyParametersToWren() {
  if (!mWrenPostProcessingEffect)
    return;

  wr_post_processing_effect_pass_set_program_parameter(wr_post_processing_effect_get_first_pass(mWrenPostProcessingEffect),
                                                       "time", reinterpret_cast<const char *>(&mTime));

  wr_post_processing_effect_pass_set_program_parameter(wr_post_processing_effect_get_first_pass(mWrenPostProcessingEffect),
                                                       "intensity", reinterpret_cast<const char *>(&mIntensity));

  wr_post_processing_effect_pass_set_program_parameter(wr_post_processing_effect_get_first_pass(mWrenPostProcessingEffect),
                                                       "minRange", reinterpret_cast<const char *>(&mMinRange));

  wr_post_processing_effect_pass_set_program_parameter(wr_post_processing_effect_get_first_pass(mWrenPostProcessingEffect),
                                                       "maxRange", reinterpret_cast<const char *>(&mMaxRange));
}
