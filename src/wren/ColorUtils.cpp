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

#include "ColorUtils.hpp"

namespace wren {
  namespace colorutils {
    glm::vec4 srgbToLinear(const glm::vec4 &inputColor) {
      const glm::vec3 bLess = step(glm::vec3(0.04045), glm::vec3(inputColor.x, inputColor.y, inputColor.z));
      const glm::vec3 linearOutput = glm::mix(
        glm::vec3(inputColor.x, inputColor.y, inputColor.z) / glm::vec3(12.92),
        glm::pow((glm::vec3(inputColor.x, inputColor.y, inputColor.z) + glm::vec3(0.055)) / glm::vec3(1.055), glm::vec3(2.4)),
        bLess);
      return glm::vec4(linearOutput, inputColor.w);
    }
  }  // namespace colorutils
}  // namespace wren
