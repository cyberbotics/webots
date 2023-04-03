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

#ifndef WB_WREN_SHADERS_HPP
#define WB_WREN_SHADERS_HPP

struct WrShaderProgram;

namespace WbWrenShaders {
  WrShaderProgram *blendLensFlareShader();
  WrShaderProgram *bloomBlendShader();
  WrShaderProgram *brightPassShader();
  WrShaderProgram *boundingVolumeShader();
  WrShaderProgram *colorNoiseShader();
  WrShaderProgram *coordinateSystemShader();
  WrShaderProgram *defaultShader();
  WrShaderProgram *depthOfFieldShader();
  WrShaderProgram *depthOnlyShader();
  WrShaderProgram *depthResolutionShader();
  WrShaderProgram *encodeDepthShader();
  WrShaderProgram *fogShader();
  WrShaderProgram *gaussianBlurShader();
  WrShaderProgram *gaussianBlur5TapShader();
  WrShaderProgram *gaussianBlur9TapShader();
  WrShaderProgram *gaussianBlur13TapShader();
  WrShaderProgram *gtaoShader();
  WrShaderProgram *gtaoSpatialDenoiseShader();
  WrShaderProgram *gtaoTemporalDenoiseShader();
  WrShaderProgram *gtaoCombineShader();
  WrShaderProgram *handlesShader();
  WrShaderProgram *handlesPickingShader();
  WrShaderProgram *hdrClearShader();
  WrShaderProgram *hdrResolveShader();
  WrShaderProgram *iblDiffuseIrradianceBakingShader();
  WrShaderProgram *iblSpecularIrradianceBakingShader();
  WrShaderProgram *iblBrdfBakingShader();
  WrShaderProgram *lensDistortionShader();
  WrShaderProgram *lensFlareShader();
  WrShaderProgram *lightRepresentationShader();
  WrShaderProgram *lineSetShader();
  WrShaderProgram *mergeSphericalShader();
  WrShaderProgram *motionBlurShader();
  WrShaderProgram *noiseMaskShader();
  WrShaderProgram *overlayShader();
  WrShaderProgram *passThroughShader();
  WrShaderProgram *pbrShader();
  WrShaderProgram *pbrStencilAmbientEmissiveShader();
  WrShaderProgram *pbrStencilDiffuseSpecularShader();
  WrShaderProgram *phongShader();
  WrShaderProgram *phongStencilAmbientEmissiveShader();
  WrShaderProgram *phongStencilDiffuseSpecularShader();
  WrShaderProgram *pickingShader();
  WrShaderProgram *pointSetShader();
  WrShaderProgram *rangeNoiseShader();
  WrShaderProgram *segmentationShader();
  WrShaderProgram *shadowVolumeShader();
  WrShaderProgram *simpleShader();
  WrShaderProgram *skyboxShader();
  WrShaderProgram *smaaEdgeDetectionShader();
  WrShaderProgram *smaaBlendingWeightCalculationShader();
  WrShaderProgram *smaaFinalBlendShader();

  void deleteShaders();
};  // namespace WbWrenShaders

#endif
