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

#include "WbWrenLensDistortion.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/post_processing_effect.h>
#include <wren/texture.h>
#include <wren/viewport.h>

WbWrenLensDistortion::WbWrenLensDistortion() :
  WbWrenAbstractPostProcessingEffect(),
  mCenter{0, 0},
  mRadialDistortionCoefficients{0, 0},
  mTangentialDistortionCoefficients{0, 0} {
}

void WbWrenLensDistortion::setup(WrViewport *viewport) {
  if (mWrenPostProcessingEffect) {
    // In case we want to update the viewport, the old postProcessingEffect has to be removed first
    if (mWrenViewport == viewport)
      wr_viewport_remove_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);

    wr_post_processing_effect_delete(mWrenPostProcessingEffect);
  }

  mWrenViewport = viewport;

  const float width = wr_viewport_get_width(mWrenViewport);
  const float height = wr_viewport_get_height(mWrenViewport);

  mWrenPostProcessingEffect = WbWrenPostProcessingEffects::lensDistortion(width, height, mTextureFormat);

  applyParametersToWren();

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_viewport_add_post_processing_effect(mWrenViewport, mWrenPostProcessingEffect);
  wr_post_processing_effect_setup(mWrenPostProcessingEffect);

  WbWrenOpenGlContext::doneWren();
  mHasBeenSetup = true;
}

void WbWrenLensDistortion::setCenter(float x, float y) {
  mCenter[0] = x;
  mCenter[1] = y;

  applyParametersToWren();
}

void WbWrenLensDistortion::setRadialDistortionCoefficients(float x, float y) {
  mRadialDistortionCoefficients[0] = x;
  mRadialDistortionCoefficients[1] = y;

  applyParametersToWren();
}

void WbWrenLensDistortion::setTangentialDistortionCoefficients(float x, float y) {
  mTangentialDistortionCoefficients[0] = x;
  mTangentialDistortionCoefficients[1] = y;

  applyParametersToWren();
}
void WbWrenLensDistortion::applyParametersToWren() {
  if (!mWrenPostProcessingEffect)
    return;

  wr_post_processing_effect_pass_set_program_parameter(wr_post_processing_effect_get_first_pass(mWrenPostProcessingEffect),
                                                       "center", reinterpret_cast<const char *>(&mCenter));

  wr_post_processing_effect_pass_set_program_parameter(wr_post_processing_effect_get_first_pass(mWrenPostProcessingEffect),
                                                       "radialDistortionCoeffs",
                                                       reinterpret_cast<const char *>(&mRadialDistortionCoefficients));

  wr_post_processing_effect_pass_set_program_parameter(wr_post_processing_effect_get_first_pass(mWrenPostProcessingEffect),
                                                       "tangentialDistortionCoeffs",
                                                       reinterpret_cast<const char *>(&mTangentialDistortionCoefficients));
}
