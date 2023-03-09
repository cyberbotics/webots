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

#include "GlslLayout.hpp"

namespace wren {
  namespace GlslLayout {

    const unsigned int gLocationCoords = 0;
    const unsigned int gLocationNormals = 1;
    const unsigned int gLocationTexCoords = 2;
    const unsigned int gLocationColors = 3;
    const unsigned int gLocationUnwrappedTexCoords = 4;

    const std::vector<const char *> gUniformNames = {
      "inputTextures[0]",  "inputTextures[1]", "inputTextures[2]", "inputTextures[3]", "inputTextures[4]",  "inputTextures[5]",
      "inputTextures[6]",  "inputTextures[7]", "inputTextures[8]", "inputTextures[9]", "inputTextures[10]", "inputTextures[11]",
      "inputTextures[12]", "cubeTextures[0]",  "cubeTextures[1]",  "iterationNumber",  "modelTransform",    "textureTransform",
      "viewportSize",      "colorPerVertex",   "pointSize",        "channelCount",     "gtaoTexture"};

    const std::vector<const char *> gUniformBufferNames = {"PhongMaterial",    "PbrMaterial", "Lights", "LightRenderable",
                                                           "CameraTransforms", "Fog",         "Overlay"};

    const std::vector<int> gUniformBufferSizes = {sizeof(PhongMaterial),   sizeof(PbrMaterial),      sizeof(Lights),
                                                  sizeof(LightRenderable), sizeof(CameraTransforms), sizeof(Fog),
                                                  sizeof(Overlay)};

  }  // namespace GlslLayout
}  // namespace wren
