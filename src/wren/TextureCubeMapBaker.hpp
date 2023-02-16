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

#ifndef TEXTURE_CUBE_MAP_BAKER_HPP
#define TEXTURE_CUBE_MAP_BAKER_HPP

#include "Constants.hpp"
#include "GlslLayout.hpp"

#include <memory>
#include <vector>

namespace wren {
  class Texture2d;
  class TextureRtt;
  class TextureCubeMap;
  class ShaderProgram;
  namespace texturecubemapbaker {
    void renderCube();
    void renderQuad();
    TextureCubeMap *bakeDiffuseIrradiance(TextureCubeMap *inputCube, ShaderProgram *irradianceShader, unsigned int size);
    TextureCubeMap *bakeSpecularIrradiance(TextureCubeMap *inputCube, ShaderProgram *irradianceShader, unsigned int size);
    TextureRtt *bakeBrdf(ShaderProgram *brdfShader, unsigned int size);

  }  // namespace texturecubemapbaker
}  // namespace wren

#endif  // TEXTURE_CUBE_MAP_BAKER_HPP
