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

#include "WbWrenBloom.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/post_processing_effect.h>
#include <wren/texture.h>
#include <wren/viewport.h>

WbWrenBloom::WbWrenBloom() : WbWrenAbstractPostProcessingEffect(), mThreshold(10.0f) {
}

void WbWrenBloom::setup(WrViewport *viewport) {
  if (mWrenPostProcessingEffect) {
    // In case we want to update the viewport, the old postProcessingEffect has to be removed first
    if (mWrenViewport == viewport)
      wr_viewport_remove_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);

    wr_post_processing_effect_delete(mWrenPostProcessingEffect);
    mWrenPostProcessingEffect = NULL;
  }

  mWrenViewport = viewport;

  const float width = wr_viewport_get_width(mWrenViewport);
  const float height = wr_viewport_get_height(mWrenViewport);

  // can't use the effect on resolutions smaller than this, it requires 6 passes dividing the viewport each time, so resolutions
  // smaller than 2^6 in width or height preculde the use of this effect
  if (qMin(width, height) <= 64.0)
    return;

  mWrenPostProcessingEffect = WbWrenPostProcessingEffects::bloom(width, height, WR_TEXTURE_INTERNAL_FORMAT_RGB16F);

  applyParametersToWren();

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_viewport_add_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);
  wr_post_processing_effect_setup(mWrenPostProcessingEffect);

  WbWrenOpenGlContext::doneWren();
  mHasBeenSetup = true;
}

void WbWrenBloom::setThreshold(float threshold) {
  mThreshold = threshold;

  applyParametersToWren();
}

void WbWrenBloom::applyParametersToWren() {
  if (!mWrenPostProcessingEffect)
    return;

  wr_post_processing_effect_pass_set_program_parameter(
    wr_post_processing_effect_get_pass(mWrenPostProcessingEffect, "brightPassFilter"), "threshold",
    reinterpret_cast<const char *>(&mThreshold));
}
