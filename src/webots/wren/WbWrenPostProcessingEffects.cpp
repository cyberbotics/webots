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

#include "WbWrenPostProcessingEffects.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/post_processing_effect.h>
#include <wren/texture_2d.h>

#include <QtCore/QFileInfo>
#include <QtGui/QImage>

namespace WbWrenPostProcessingEffects {
  static WrTexture2d *lensFlareLenTexture = NULL;
  static WrTexture2d *smaaAreaTexture = NULL;
  static WrTexture2d *smaaSearchTexture = NULL;
  static WrTexture2d *gtaoNoiseTexture = NULL;
}  // namespace WbWrenPostProcessingEffects

WrTexture2d *loadReadOnlyTexture(const QString &path) {
  // Load len chroma distortion texture (512x1 ARGB)
  QByteArray imagePath = QFileInfo(path).absoluteFilePath().toUtf8();
  QImage *image = new QImage(imagePath.constData());
  assert(!image->isNull());

  bool isTranslucent = image->pixelFormat().alphaUsage() == QPixelFormat::UsesAlpha;
  if (image->format() != QImage::Format_ARGB32) {
    QImage tmp = image->convertToFormat(QImage::Format_ARGB32);
    image->swap(tmp);
  }

  WbWrenOpenGlContext::makeWrenCurrent();

  WrTexture2d *targetTexture = wr_texture_2d_new();
  wr_texture_set_translucent(WR_TEXTURE(targetTexture), true);
  wr_texture_set_size(WR_TEXTURE(targetTexture), image->width(), image->height());
  wr_texture_2d_set_data(targetTexture, reinterpret_cast<const char *>(image->bits()));
  wr_texture_2d_set_file_path(targetTexture, imagePath);
  wr_texture_2d_set_cache_persistency(targetTexture, true);
  wr_texture_set_translucent(WR_TEXTURE(targetTexture), isTranslucent);
  wr_texture_setup(WR_TEXTURE(targetTexture));

  WbWrenOpenGlContext::doneWren();
  delete image;
  return targetTexture;
}

void WbWrenPostProcessingEffects::loadResources() {
  lensFlareLenTexture = loadReadOnlyTexture("gl:textures/lens_flare.png");
  smaaAreaTexture = loadReadOnlyTexture("gl:textures/smaa_area_texture.png");
  smaaSearchTexture = loadReadOnlyTexture("gl:textures/smaa_search_texture.png");
  gtaoNoiseTexture = loadReadOnlyTexture("gl:textures/gtao_noise_texture.png");
}

void WbWrenPostProcessingEffects::clearResources() {
  wr_texture_delete(WR_TEXTURE(lensFlareLenTexture));
  wr_texture_delete(WR_TEXTURE(smaaAreaTexture));
  wr_texture_delete(WR_TEXTURE(smaaSearchTexture));
  wr_texture_delete(WR_TEXTURE(gtaoNoiseTexture));

  lensFlareLenTexture = NULL;
  smaaAreaTexture = NULL;
  smaaSearchTexture = NULL;
  gtaoNoiseTexture = NULL;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::lensFlare(float width, float height, int blurIterations) {
  WrPostProcessingEffect *lensFlareEffect = wr_post_processing_effect_new();

  wr_post_processing_effect_set_drawing_index(lensFlareEffect, WbWrenRenderingContext::PP_LENS_FLARE);

  WrPostProcessingEffectPass *throughPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(throughPass, "LensFlarePassToBlend");
  wr_post_processing_effect_pass_set_program(throughPass, WbWrenShaders::passThroughShader());
  wr_post_processing_effect_pass_set_output_size(throughPass, width, height);
  wr_post_processing_effect_pass_set_alpha_blending(throughPass, false);
  wr_post_processing_effect_pass_set_input_texture_count(throughPass, 1);
  wr_post_processing_effect_pass_set_output_texture_count(throughPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(throughPass, 0, WR_TEXTURE_INTERNAL_FORMAT_RGBA8);
  wr_post_processing_effect_append_pass(lensFlareEffect, throughPass);

  WrPostProcessingEffectPass *lensFlarePass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(lensFlarePass, "LensFlare");
  wr_post_processing_effect_pass_set_program(lensFlarePass, WbWrenShaders::lensFlareShader());
  wr_post_processing_effect_pass_set_output_size(lensFlarePass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(lensFlarePass, 2);
  wr_post_processing_effect_pass_set_input_texture(lensFlarePass, 1, WR_TEXTURE(lensFlareLenTexture));
  wr_post_processing_effect_pass_set_output_texture_count(lensFlarePass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(lensFlarePass, 0, WR_TEXTURE_INTERNAL_FORMAT_RGBA8);
  wr_post_processing_effect_append_pass(lensFlareEffect, lensFlarePass);

  WrPostProcessingEffectPass *blurPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(blurPass, "LensFlareBlur");
  wr_post_processing_effect_pass_set_program(blurPass, WbWrenShaders::gaussianBlur13TapShader());
  wr_post_processing_effect_pass_set_output_size(blurPass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(blurPass, 2);
  wr_post_processing_effect_pass_set_output_texture_count(blurPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(blurPass, 0, WR_TEXTURE_INTERNAL_FORMAT_RGBA8);
  wr_post_processing_effect_pass_set_iteration_count(blurPass, blurIterations);
  wr_post_processing_effect_append_pass(lensFlareEffect, blurPass);

  WrPostProcessingEffectPass *blendPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(blendPass, "LensFlareBlend");
  wr_post_processing_effect_pass_set_program(blendPass, WbWrenShaders::blendLensFlareShader());
  wr_post_processing_effect_pass_set_output_size(blendPass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(blendPass, 2);
  wr_post_processing_effect_pass_set_output_texture_count(blendPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(blendPass, 0, WR_TEXTURE_INTERNAL_FORMAT_RGBA8);
  wr_post_processing_effect_append_pass(lensFlareEffect, blendPass);

  wr_post_processing_effect_connect(lensFlareEffect, lensFlarePass, 0, blurPass, 0);
  wr_post_processing_effect_connect(lensFlareEffect, blurPass, 0, blurPass, 1);

  wr_post_processing_effect_connect(lensFlareEffect, blurPass, 0, blendPass, 1);
  wr_post_processing_effect_connect(lensFlareEffect, throughPass, 0, blendPass, 0);

  wr_post_processing_effect_set_result_program(lensFlareEffect, WbWrenShaders::passThroughShader());

  return lensFlareEffect;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::sphericalCameraMerge(float width, float height, int cameraCount,
                                                                          WrTextureInternalFormat textureFormat) {
  WrPostProcessingEffect *sphericalMerge = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(sphericalMerge, WbWrenRenderingContext::PP_SPHERICAL_CAMERA_MERGE);
  WrPostProcessingEffectPass *mergePass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(mergePass, "MergeSpherical");
  wr_post_processing_effect_pass_set_program(mergePass, WbWrenShaders::mergeSphericalShader());
  wr_post_processing_effect_pass_set_output_size(mergePass, width, height);
  wr_post_processing_effect_pass_set_alpha_blending(mergePass, false);

  wr_post_processing_effect_pass_set_input_texture_count(mergePass, cameraCount);
  for (int i = 0; i < cameraCount; ++i)
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(mergePass, i, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);

  wr_post_processing_effect_pass_set_output_texture_count(mergePass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(mergePass, 0, textureFormat);
  wr_post_processing_effect_append_pass(sphericalMerge, mergePass);

  wr_post_processing_effect_set_result_program(sphericalMerge, WbWrenShaders::passThroughShader());

  return sphericalMerge;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::lensDistortion(float width, float height,
                                                                    WrTextureInternalFormat textureFormat) {
  WrPostProcessingEffect *lensDistortion = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(lensDistortion, WbWrenRenderingContext::PP_LENS_DISTORTION);

  WrPostProcessingEffectPass *pass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(pass, "LensDistortion");
  wr_post_processing_effect_pass_set_program(pass, WbWrenShaders::lensDistortionShader());
  wr_post_processing_effect_pass_set_output_size(pass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(pass, 0, textureFormat);
  wr_post_processing_effect_append_pass(lensDistortion, pass);

  wr_post_processing_effect_pass_set_input_texture_wrap_mode(pass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_set_result_program(lensDistortion, WbWrenShaders::passThroughShader());

  return lensDistortion;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::depthOfField(float width, float height, float depthOfFieldTextureWidth,
                                                                  float depthOfFieldTextureHeight,
                                                                  WrTextureInternalFormat textureFormat,
                                                                  WrTexture *colorTexture, WrTexture *depthTexture) {
  WrPostProcessingEffect *depthOfField = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(depthOfField, WbWrenRenderingContext::PP_DEPTH_OF_FIELD);
  WrPostProcessingEffectPass *blurPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(blurPass, "LensFocus_Blur");
  wr_post_processing_effect_pass_set_program(blurPass, WbWrenShaders::gaussianBlur9TapShader());
  wr_post_processing_effect_pass_set_output_size(blurPass, depthOfFieldTextureWidth, depthOfFieldTextureHeight);
  wr_post_processing_effect_pass_set_input_texture_count(blurPass, 2);
  wr_post_processing_effect_pass_set_output_texture_count(blurPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(blurPass, 0, textureFormat);
  wr_post_processing_effect_pass_set_iteration_count(blurPass, 2);
  wr_post_processing_effect_pass_set_alpha_blending(blurPass, false);
  wr_post_processing_effect_append_pass(depthOfField, blurPass);

  wr_post_processing_effect_connect(depthOfField, blurPass, 0, blurPass, 1);

  wr_post_processing_effect_pass_set_input_texture_wrap_mode(blurPass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(blurPass, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);

  WrPostProcessingEffectPass *dofPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(dofPass, "LensFocus_Dof");
  wr_post_processing_effect_pass_set_program(dofPass, WbWrenShaders::depthOfFieldShader());
  wr_post_processing_effect_pass_set_output_size(dofPass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(dofPass, 3);
  wr_post_processing_effect_pass_set_output_texture_count(dofPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(dofPass, 0, textureFormat);
  wr_post_processing_effect_pass_set_alpha_blending(dofPass, false);
  wr_post_processing_effect_append_pass(depthOfField, dofPass);

  wr_post_processing_effect_pass_set_input_texture(dofPass, 0, WR_TEXTURE(colorTexture));
  wr_post_processing_effect_pass_set_input_texture(dofPass, 1, WR_TEXTURE(depthTexture));
  wr_post_processing_effect_connect(depthOfField, blurPass, 0, dofPass, 2);

  wr_post_processing_effect_pass_set_input_texture_wrap_mode(dofPass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(dofPass, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(dofPass, 2, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);

  wr_post_processing_effect_set_result_program(depthOfField, WbWrenShaders::passThroughShader());

  return depthOfField;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::motionBlur(float width, float height,
                                                                WrTextureInternalFormat textureFormat) {
  WrPostProcessingEffect *motionBlur = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(motionBlur, WbWrenRenderingContext::PP_MOTION_BLUR);
  WrPostProcessingEffectPass *pass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(pass, "MotionBlur");
  wr_post_processing_effect_pass_set_program(pass, WbWrenShaders::motionBlurShader());
  wr_post_processing_effect_pass_set_output_size(pass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(pass, 2);
  wr_post_processing_effect_pass_set_output_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(pass, 0, textureFormat);
  wr_post_processing_effect_pass_set_alpha_blending(pass, false);
  wr_post_processing_effect_append_pass(motionBlur, pass);

  wr_post_processing_effect_connect(motionBlur, pass, 0, pass, 1);

  wr_post_processing_effect_pass_set_input_texture_wrap_mode(pass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(pass, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);

  wr_post_processing_effect_set_result_program(motionBlur, WbWrenShaders::passThroughShader());

  return motionBlur;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::colorNoise(float width, float height,
                                                                WrTextureInternalFormat textureFormat) {
  WrPostProcessingEffect *colorNoise = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(colorNoise, WbWrenRenderingContext::PP_COLOR_NOISE);
  WrPostProcessingEffectPass *pass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(pass, "ColorNoise");
  wr_post_processing_effect_pass_set_program(pass, WbWrenShaders::colorNoiseShader());
  wr_post_processing_effect_pass_set_output_size(pass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(pass, 0, textureFormat);
  wr_post_processing_effect_pass_set_alpha_blending(pass, false);
  wr_post_processing_effect_append_pass(colorNoise, pass);

  wr_post_processing_effect_set_result_program(colorNoise, WbWrenShaders::passThroughShader());

  return colorNoise;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::rangeNoise(float width, float height,
                                                                WrTextureInternalFormat textureFormat) {
  WrPostProcessingEffect *rangeNoise = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(rangeNoise, WbWrenRenderingContext::PP_RANGE_NOISE);
  WrPostProcessingEffectPass *pass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(pass, "RangeNoise");
  wr_post_processing_effect_pass_set_program(pass, WbWrenShaders::rangeNoiseShader());
  wr_post_processing_effect_pass_set_output_size(pass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(pass, 0, textureFormat);
  wr_post_processing_effect_pass_set_alpha_blending(pass, false);
  wr_post_processing_effect_append_pass(rangeNoise, pass);

  wr_post_processing_effect_set_result_program(rangeNoise, WbWrenShaders::passThroughShader());

  return rangeNoise;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::depthResolution(float width, float height,
                                                                     WrTextureInternalFormat textureFormat) {
  WrPostProcessingEffect *depthResolution = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(depthResolution, WbWrenRenderingContext::PP_DEPTH_RESOLUTION);
  WrPostProcessingEffectPass *pass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(pass, "DepthResolution");
  wr_post_processing_effect_pass_set_program(pass, WbWrenShaders::depthResolutionShader());
  wr_post_processing_effect_pass_set_output_size(pass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(pass, 0, textureFormat);
  wr_post_processing_effect_pass_set_alpha_blending(pass, false);
  wr_post_processing_effect_append_pass(depthResolution, pass);

  wr_post_processing_effect_set_result_program(depthResolution, WbWrenShaders::passThroughShader());

  return depthResolution;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::noiseMask(float width, float height, WrTextureInternalFormat textureFormat,
                                                               WrTexture *noiseMaskTexture) {
  WrPostProcessingEffect *noiseMask = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(noiseMask, WbWrenRenderingContext::PP_NOISE_MASK);
  WrPostProcessingEffectPass *pass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(pass, "NoiseMask");
  wr_post_processing_effect_pass_set_program(pass, WbWrenShaders::noiseMaskShader());
  wr_post_processing_effect_pass_set_output_size(pass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(pass, 2);
  wr_post_processing_effect_pass_set_output_texture_count(pass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(pass, 0, textureFormat);
  wr_post_processing_effect_pass_set_input_texture(pass, 1, WR_TEXTURE(noiseMaskTexture));
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(pass, 1, WR_TEXTURE_WRAP_MODE_REPEAT);
  wr_post_processing_effect_append_pass(noiseMask, pass);

  wr_post_processing_effect_set_result_program(noiseMask, WbWrenShaders::passThroughShader());

  return noiseMask;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::smaa(float width, float height, WrTextureInternalFormat textureFormat) {
  WrPostProcessingEffect *smaaEffect = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(smaaEffect, WbWrenRenderingContext::PP_SMAA);

  WrPostProcessingEffectPass *throughPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(throughPass, "LensFlarePassToBlend");
  wr_post_processing_effect_pass_set_program(throughPass, WbWrenShaders::passThroughShader());
  wr_post_processing_effect_pass_set_output_size(throughPass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(throughPass, 1);
  wr_post_processing_effect_pass_set_alpha_blending(throughPass, false);
  wr_post_processing_effect_pass_set_output_texture_count(throughPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(throughPass, 0, WR_TEXTURE_INTERNAL_FORMAT_RGBA8);
  wr_post_processing_effect_append_pass(smaaEffect, throughPass);

  WrPostProcessingEffectPass *edgeDetection = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(edgeDetection, "EdgeDetect");
  wr_post_processing_effect_pass_set_program(edgeDetection, WbWrenShaders::smaaEdgeDetectionShader());
  wr_post_processing_effect_pass_set_output_size(edgeDetection, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(edgeDetection, 1);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(edgeDetection, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_clear_before_draw(edgeDetection, true);
  wr_post_processing_effect_pass_set_output_texture_count(edgeDetection, 1);
  wr_post_processing_effect_pass_set_output_texture_format(edgeDetection, 0, WR_TEXTURE_INTERNAL_FORMAT_RG8);
  wr_post_processing_effect_append_pass(smaaEffect, edgeDetection);

  WrPostProcessingEffectPass *weightCalculation = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(weightCalculation, "WeightCalculation");
  wr_post_processing_effect_pass_set_program(weightCalculation, WbWrenShaders::smaaBlendingWeightCalculationShader());
  wr_post_processing_effect_pass_set_output_size(weightCalculation, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(weightCalculation, 3);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(weightCalculation, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(weightCalculation, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(weightCalculation, 2, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture(weightCalculation, 1, WR_TEXTURE(smaaAreaTexture));
  wr_post_processing_effect_pass_set_input_texture(weightCalculation, 2, WR_TEXTURE(smaaSearchTexture));
  wr_post_processing_effect_pass_set_input_texture_interpolation(weightCalculation, 2, false);
  wr_post_processing_effect_pass_set_clear_before_draw(weightCalculation, true);
  wr_post_processing_effect_pass_set_alpha_blending(weightCalculation, false);
  wr_post_processing_effect_pass_set_output_texture_count(weightCalculation, 1);
  wr_post_processing_effect_pass_set_output_texture_format(weightCalculation, 0, WR_TEXTURE_INTERNAL_FORMAT_RGBA8);
  wr_post_processing_effect_append_pass(smaaEffect, weightCalculation);

  WrPostProcessingEffectPass *finalBlend = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(finalBlend, "FinalBlend");
  wr_post_processing_effect_pass_set_program(finalBlend, WbWrenShaders::smaaFinalBlendShader());
  wr_post_processing_effect_pass_set_output_size(finalBlend, width, height);
  wr_post_processing_effect_pass_set_alpha_blending(finalBlend, false);
  wr_post_processing_effect_pass_set_input_texture_count(finalBlend, 2);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(finalBlend, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(finalBlend, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_output_texture_count(finalBlend, 1);
  wr_post_processing_effect_pass_set_output_texture_format(finalBlend, 0, WR_TEXTURE_INTERNAL_FORMAT_RGB8);
  wr_post_processing_effect_append_pass(smaaEffect, finalBlend);

  wr_post_processing_effect_connect(smaaEffect, throughPass, 0, finalBlend, 0);
  wr_post_processing_effect_connect(smaaEffect, edgeDetection, 0, weightCalculation, 0);
  wr_post_processing_effect_connect(smaaEffect, weightCalculation, 0, finalBlend, 1);

  wr_post_processing_effect_set_result_program(smaaEffect, WbWrenShaders::passThroughShader());

  return smaaEffect;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::gtao(float width, float height, WrTextureInternalFormat textureFormat,
                                                          WrTexture *depthTexture, WrTexture *normalTexture, bool halfRes) {
  WrPostProcessingEffect *gtaoEffect = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(gtaoEffect, WbWrenRenderingContext::PP_GTAO);

  WrPostProcessingEffectPass *colorPassThrough = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(colorPassThrough, "colorPassThrough");
  wr_post_processing_effect_pass_set_program(colorPassThrough, WbWrenShaders::passThroughShader());
  wr_post_processing_effect_pass_set_output_size(colorPassThrough, width, height);
  wr_post_processing_effect_pass_set_alpha_blending(colorPassThrough, false);
  wr_post_processing_effect_pass_set_input_texture_count(colorPassThrough, 1);
  wr_post_processing_effect_pass_set_input_texture_interpolation(colorPassThrough, 0, false);
  wr_post_processing_effect_pass_set_output_texture_count(colorPassThrough, 1);
  wr_post_processing_effect_pass_set_output_texture_format(colorPassThrough, 0, textureFormat);
  wr_post_processing_effect_append_pass(gtaoEffect, colorPassThrough);

  WrPostProcessingEffectPass *depthDownsamplePassThrough, *normalDownsamplePassThrough = NULL;
  if (halfRes) {
    depthDownsamplePassThrough = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(depthDownsamplePassThrough, "depthDownsamplePassThrough");
    wr_post_processing_effect_pass_set_program(depthDownsamplePassThrough, WbWrenShaders::passThroughShader());
    wr_post_processing_effect_pass_set_output_size(depthDownsamplePassThrough, width / 2, height / 2);
    wr_post_processing_effect_pass_set_input_texture_count(depthDownsamplePassThrough, 1);
    wr_post_processing_effect_pass_set_alpha_blending(depthDownsamplePassThrough, false);
    wr_post_processing_effect_pass_set_input_texture(depthDownsamplePassThrough, 0, WR_TEXTURE(depthTexture));
    wr_post_processing_effect_pass_set_input_texture_interpolation(depthDownsamplePassThrough, 0, false);
    wr_post_processing_effect_pass_set_output_texture_count(depthDownsamplePassThrough, 1);
    wr_post_processing_effect_pass_set_output_texture_format(depthDownsamplePassThrough, 0, WR_TEXTURE_INTERNAL_FORMAT_R32F);
    wr_post_processing_effect_append_pass(gtaoEffect, depthDownsamplePassThrough);

    normalDownsamplePassThrough = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(normalDownsamplePassThrough, "normalDownsamplePassThrough");
    wr_post_processing_effect_pass_set_program(normalDownsamplePassThrough, WbWrenShaders::passThroughShader());
    wr_post_processing_effect_pass_set_alpha_blending(normalDownsamplePassThrough, false);
    wr_post_processing_effect_pass_set_output_size(normalDownsamplePassThrough, width / 2, height / 2);
    wr_post_processing_effect_pass_set_input_texture_count(normalDownsamplePassThrough, 1);
    wr_post_processing_effect_pass_set_input_texture(normalDownsamplePassThrough, 0, WR_TEXTURE(normalTexture));
    wr_post_processing_effect_pass_set_input_texture_interpolation(normalDownsamplePassThrough, 0, false);
    wr_post_processing_effect_pass_set_output_texture_count(normalDownsamplePassThrough, 1);
    wr_post_processing_effect_pass_set_output_texture_format(normalDownsamplePassThrough, 0, textureFormat);
    wr_post_processing_effect_append_pass(gtaoEffect, normalDownsamplePassThrough);
  }

  WrPostProcessingEffectPass *gtaoForwardPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(gtaoForwardPass, "gtaoForwardPass");
  wr_post_processing_effect_pass_set_program(gtaoForwardPass, WbWrenShaders::gtaoShader());
  wr_post_processing_effect_pass_set_input_texture_count(gtaoForwardPass, 3);

  if (halfRes)
    wr_post_processing_effect_pass_set_output_size(gtaoForwardPass, width / 2, height / 2);
  else {
    wr_post_processing_effect_pass_set_output_size(gtaoForwardPass, width, height);
    wr_post_processing_effect_pass_set_input_texture(gtaoForwardPass, 0, WR_TEXTURE(depthTexture));
    wr_post_processing_effect_pass_set_input_texture(gtaoForwardPass, 1, WR_TEXTURE(normalTexture));
  }

  wr_post_processing_effect_pass_set_input_texture(gtaoForwardPass, 2, WR_TEXTURE(gtaoNoiseTexture));
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(gtaoForwardPass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(gtaoForwardPass, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(gtaoForwardPass, 2, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_interpolation(gtaoForwardPass, 0, false);
  wr_post_processing_effect_pass_set_input_texture_interpolation(gtaoForwardPass, 1, false);
  wr_post_processing_effect_pass_set_input_texture_interpolation(gtaoForwardPass, 2, false);
  wr_post_processing_effect_pass_set_clear_before_draw(gtaoForwardPass, true);
  wr_post_processing_effect_pass_set_alpha_blending(gtaoForwardPass, false);
  wr_post_processing_effect_pass_set_output_texture_count(gtaoForwardPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(gtaoForwardPass, 0, WR_TEXTURE_INTERNAL_FORMAT_RED);
  wr_post_processing_effect_append_pass(gtaoEffect, gtaoForwardPass);

  WrPostProcessingEffectPass *spatialDenoise = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(spatialDenoise, "spatialDenoise");
  wr_post_processing_effect_pass_set_program(spatialDenoise, WbWrenShaders::gtaoSpatialDenoiseShader());

  if (halfRes)
    wr_post_processing_effect_pass_set_output_size(spatialDenoise, width / 2, height / 2);
  else
    wr_post_processing_effect_pass_set_output_size(spatialDenoise, width, height);

  wr_post_processing_effect_pass_set_input_texture_count(spatialDenoise, 2);
  wr_post_processing_effect_pass_set_input_texture(spatialDenoise, 1, WR_TEXTURE(depthTexture));
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(spatialDenoise, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(spatialDenoise, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_interpolation(spatialDenoise, 0, true);
  wr_post_processing_effect_pass_set_input_texture_interpolation(spatialDenoise, 1, false);
  wr_post_processing_effect_pass_set_alpha_blending(spatialDenoise, false);
  wr_post_processing_effect_pass_set_clear_before_draw(spatialDenoise, true);
  wr_post_processing_effect_pass_set_output_texture_count(spatialDenoise, 1);
  wr_post_processing_effect_pass_set_output_texture_format(spatialDenoise, 0, WR_TEXTURE_INTERNAL_FORMAT_RED);
  wr_post_processing_effect_append_pass(gtaoEffect, spatialDenoise);

  WrPostProcessingEffectPass *temporalDenoise = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(temporalDenoise, "temporalDenoise");
  wr_post_processing_effect_pass_set_program(temporalDenoise, WbWrenShaders::gtaoTemporalDenoiseShader());
  wr_post_processing_effect_pass_set_output_size(temporalDenoise, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(temporalDenoise, 4);
  wr_post_processing_effect_pass_set_input_texture(temporalDenoise, 3, WR_TEXTURE(depthTexture));
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(temporalDenoise, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(temporalDenoise, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(temporalDenoise, 2, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(temporalDenoise, 3, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_interpolation(temporalDenoise, 0, true);
  wr_post_processing_effect_pass_set_input_texture_interpolation(temporalDenoise, 1, true);
  wr_post_processing_effect_pass_set_input_texture_interpolation(temporalDenoise, 2, false);
  wr_post_processing_effect_pass_set_input_texture_interpolation(temporalDenoise, 3, false);
  wr_post_processing_effect_pass_set_alpha_blending(temporalDenoise, false);
  wr_post_processing_effect_pass_set_clear_before_draw(temporalDenoise, true);
  wr_post_processing_effect_pass_set_output_texture_count(temporalDenoise, 1);
  wr_post_processing_effect_pass_set_output_texture_format(temporalDenoise, 0, WR_TEXTURE_INTERNAL_FORMAT_RED);
  wr_post_processing_effect_append_pass(gtaoEffect, temporalDenoise);

  WrPostProcessingEffectPass *finalBlend = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(finalBlend, "FinalBlend");
  wr_post_processing_effect_pass_set_program(finalBlend, WbWrenShaders::gtaoCombineShader());
  wr_post_processing_effect_pass_set_output_size(finalBlend, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(finalBlend, 3);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(finalBlend, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(finalBlend, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(finalBlend, 2, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_input_texture_interpolation(finalBlend, 0, false);
  wr_post_processing_effect_pass_set_input_texture_interpolation(finalBlend, 1, false);
  wr_post_processing_effect_pass_set_input_texture_interpolation(finalBlend, 2, false);
  wr_post_processing_effect_pass_set_clear_before_draw(finalBlend, true);
  wr_post_processing_effect_pass_set_input_texture(finalBlend, 2, WR_TEXTURE(depthTexture));
  wr_post_processing_effect_pass_set_output_texture_count(finalBlend, 3);
  wr_post_processing_effect_pass_set_output_texture_format(finalBlend, 0, textureFormat);
  wr_post_processing_effect_pass_set_output_texture_format(finalBlend, 1, WR_TEXTURE_INTERNAL_FORMAT_RED);
  wr_post_processing_effect_pass_set_output_texture_format(finalBlend, 2, WR_TEXTURE_INTERNAL_FORMAT_R32F);
  wr_post_processing_effect_append_pass(gtaoEffect, finalBlend);

  // color texture for blending at the end
  wr_post_processing_effect_connect(gtaoEffect, colorPassThrough, 0, finalBlend, 0);

  // downsampled textures for half-res AO
  if (halfRes) {
    wr_post_processing_effect_connect(gtaoEffect, depthDownsamplePassThrough, 0, gtaoForwardPass, 0);
    wr_post_processing_effect_connect(gtaoEffect, normalDownsamplePassThrough, 0, gtaoForwardPass, 1);
  }

  // denoising
  wr_post_processing_effect_connect(gtaoEffect, gtaoForwardPass, 0, spatialDenoise, 0);
  wr_post_processing_effect_connect(gtaoEffect, spatialDenoise, 0, temporalDenoise, 1);
  wr_post_processing_effect_connect(gtaoEffect, temporalDenoise, 0, finalBlend, 1);

  // loopbacks for temporal
  wr_post_processing_effect_connect(gtaoEffect, finalBlend, 1, temporalDenoise, 0);
  wr_post_processing_effect_connect(gtaoEffect, finalBlend, 2, temporalDenoise, 2);

  wr_post_processing_effect_set_result_program(gtaoEffect, WbWrenShaders::passThroughShader());

  return gtaoEffect;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::bloom(float width, float height, WrTextureInternalFormat textureFormat) {
  WrPostProcessingEffect *bloomEffect = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(bloomEffect, WbWrenRenderingContext::PP_BLOOM);

  WrPostProcessingEffectPass *colorPassThrough = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(colorPassThrough, "colorPassThrough");
  wr_post_processing_effect_pass_set_program(colorPassThrough, WbWrenShaders::passThroughShader());
  wr_post_processing_effect_pass_set_output_size(colorPassThrough, width, height);
  wr_post_processing_effect_pass_set_alpha_blending(colorPassThrough, false);
  wr_post_processing_effect_pass_set_input_texture_count(colorPassThrough, 1);
  wr_post_processing_effect_pass_set_output_texture_count(colorPassThrough, 1);
  wr_post_processing_effect_pass_set_output_texture_format(colorPassThrough, 0, textureFormat);
  wr_post_processing_effect_append_pass(bloomEffect, colorPassThrough);

  WrPostProcessingEffectPass *brightPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(brightPass, "brightPassFilter");
  wr_post_processing_effect_pass_set_program(brightPass, WbWrenShaders::brightPassShader());
  wr_post_processing_effect_pass_set_output_size(brightPass, width, height);
  wr_post_processing_effect_pass_set_alpha_blending(brightPass, false);
  wr_post_processing_effect_pass_set_input_texture_count(brightPass, 1);
  wr_post_processing_effect_pass_set_output_texture_count(brightPass, 1);
  wr_post_processing_effect_pass_set_input_texture_wrap_mode(brightPass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
  wr_post_processing_effect_pass_set_output_texture_format(brightPass, 0, textureFormat);
  wr_post_processing_effect_append_pass(bloomEffect, brightPass);

  QVector<WrPostProcessingEffectPass *> blurPasses;
  QVector<WrPostProcessingEffectPass *> downsamplePasses;

  for (int i = 0; i < 6; ++i) {
    WrPostProcessingEffectPass *blurPass = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(blurPass, (QString("blurPass") + QString::number(i)).toUtf8().constData());
    wr_post_processing_effect_pass_set_program(blurPass, WbWrenShaders::gaussianBlur13TapShader());
    wr_post_processing_effect_pass_set_output_size(blurPass, width / (1 << i), height / (1 << i));
    wr_post_processing_effect_pass_set_input_texture_count(blurPass, 2);
    wr_post_processing_effect_pass_set_alpha_blending(blurPass, false);
    wr_post_processing_effect_pass_set_input_texture_interpolation(blurPass, 0, true);
    wr_post_processing_effect_pass_set_input_texture_interpolation(blurPass, 1, true);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(blurPass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(blurPass, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_output_texture_count(blurPass, 1);
    wr_post_processing_effect_pass_set_output_texture_format(blurPass, 0, textureFormat);
    wr_post_processing_effect_pass_set_iteration_count(blurPass, 2);
    wr_post_processing_effect_append_pass(bloomEffect, blurPass);
    blurPasses.push_back(blurPass);

    WrPostProcessingEffectPass *downsamplePass = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(downsamplePass,
                                            (QString("downsamplePass") + QString::number(i)).toUtf8().constData());
    wr_post_processing_effect_pass_set_program(downsamplePass, WbWrenShaders::passThroughShader());
    wr_post_processing_effect_pass_set_output_size(downsamplePass, width / (2 << i), height / (2 << i));
    wr_post_processing_effect_pass_set_input_texture_count(downsamplePass, 1);
    wr_post_processing_effect_pass_set_alpha_blending(downsamplePass, false);
    wr_post_processing_effect_pass_set_input_texture_interpolation(downsamplePass, 0, true);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(downsamplePass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_output_texture_count(downsamplePass, 1);
    wr_post_processing_effect_pass_set_output_texture_format(downsamplePass, 0, textureFormat);
    wr_post_processing_effect_append_pass(bloomEffect, downsamplePass);
    downsamplePasses.push_back(downsamplePass);
  }

  WrPostProcessingEffectPass *blendPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(blendPass, "blendBloom");
  wr_post_processing_effect_pass_set_alpha_blending(blendPass, false);
  wr_post_processing_effect_pass_set_program(blendPass, WbWrenShaders::bloomBlendShader());
  wr_post_processing_effect_pass_set_output_size(blendPass, width, height);
  wr_post_processing_effect_pass_set_input_texture_count(blendPass, 7);

  for (int i = 0; i < 7; ++i)
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(blendPass, i, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);

  wr_post_processing_effect_pass_set_output_texture_count(blendPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(blendPass, 0, textureFormat);
  wr_post_processing_effect_append_pass(bloomEffect, blendPass);

  wr_post_processing_effect_connect(bloomEffect, colorPassThrough, 0, blendPass, 0);
  wr_post_processing_effect_connect(bloomEffect, brightPass, 0, blurPasses.at(0), 0);
  for (int i = 0; i < 5; ++i) {
    wr_post_processing_effect_connect(bloomEffect, blurPasses.at(i), 0, downsamplePasses.at(i), 0);
    wr_post_processing_effect_connect(bloomEffect, blurPasses.at(i), 0, blurPasses.at(i), 1);
    wr_post_processing_effect_connect(bloomEffect, downsamplePasses.at(i), 0, blurPasses.at(i + 1), 0);
    wr_post_processing_effect_connect(bloomEffect, blurPasses.at(i), 0, blendPass, i + 1);
  }

  wr_post_processing_effect_set_result_program(bloomEffect, WbWrenShaders::passThroughShader());

  return bloomEffect;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::hdrResolve(float width, float height) {
  WrPostProcessingEffect *hdrResolveEffect = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(hdrResolveEffect, WbWrenRenderingContext::PP_HDR);

  WrPostProcessingEffectPass *hdrPass = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(hdrPass, "hdrResolve");
  wr_post_processing_effect_pass_set_program(hdrPass, WbWrenShaders::hdrResolveShader());
  wr_post_processing_effect_pass_set_output_size(hdrPass, width, height);
  wr_post_processing_effect_pass_set_alpha_blending(hdrPass, false);
  wr_post_processing_effect_pass_set_input_texture_count(hdrPass, 1);
  wr_post_processing_effect_pass_set_output_texture_count(hdrPass, 1);
  wr_post_processing_effect_pass_set_output_texture_format(hdrPass, 0, WR_TEXTURE_INTERNAL_FORMAT_RGB8);
  wr_post_processing_effect_append_pass(hdrResolveEffect, hdrPass);

  wr_post_processing_effect_set_result_program(hdrResolveEffect, WbWrenShaders::passThroughShader());

  return hdrResolveEffect;
}

WrPostProcessingEffect *WbWrenPostProcessingEffects::passThrough(float width, float height) {
  WrPostProcessingEffect *passThroughEffect = wr_post_processing_effect_new();
  wr_post_processing_effect_set_drawing_index(passThroughEffect, WbWrenRenderingContext::PP_PASS_THROUGH);

  WrPostProcessingEffectPass *passThrough = wr_post_processing_effect_pass_new();
  wr_post_processing_effect_pass_set_name(passThrough, "PassThrough");
  wr_post_processing_effect_pass_set_program(passThrough, WbWrenShaders::passThroughShader());
  wr_post_processing_effect_pass_set_output_size(passThrough, width, height);
  wr_post_processing_effect_pass_set_alpha_blending(passThrough, false);
  wr_post_processing_effect_pass_set_input_texture_count(passThrough, 1);
  wr_post_processing_effect_pass_set_output_texture_count(passThrough, 1);
  wr_post_processing_effect_pass_set_output_texture_format(passThrough, 0, WR_TEXTURE_INTERNAL_FORMAT_RGB8);
  wr_post_processing_effect_append_pass(passThroughEffect, passThrough);

  wr_post_processing_effect_set_result_program(passThroughEffect, WbWrenShaders::passThroughShader());

  return passThroughEffect;
}
