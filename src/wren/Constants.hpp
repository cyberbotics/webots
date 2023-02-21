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

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#ifndef __APPLE__
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#endif

#define GLM_FORCE_RADIANS

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <cassert>  // for assert
#include <cstddef>  // for size_t

namespace wren {

  constexpr bool MEASURE_PERFORMANCE = false;

  namespace primitive {
    struct Aabb;
    struct Sphere;
  }  // namespace primitive

  const glm::vec3 gVec3Zeros(0.0f, 0.0f, 0.0f);
  const glm::vec3 gVec3Ones(1.0f, 1.0f, 1.0f);
  const glm::vec3 gVec3UnitX(1.0f, 0.0f, 0.0f);
  const glm::vec3 gVec3UnitY(0.0f, 1.0f, 0.0f);
  const glm::vec3 gVec3UnitZ(0.0f, 0.0f, 1.0f);

  const glm::vec4 gVec4Zeros(0.0f, 0.0f, 0.0f, 0.0f);
  const glm::vec4 gVec4Ones(1.0f, 1.0f, 1.0f, 1.0f);
  const glm::vec4 gVec4ColorBlack(0.0f, 0.0f, 0.0f, 1.0f);

  const glm::mat4 gMat4Identity(glm::mat4(1.0f));

  extern const primitive::Aabb gAabbInf;
  extern const primitive::Aabb gAabbEmpty;
  extern const primitive::Sphere gSphereInf;

  // Max. active lights per frame (needs to be kept in sync with the shaders)
  const int gMaxActiveDirectionalLights = 48;
  const int gMaxActivePointLights = 48;
  const int gMaxActiveSpotLights = 48;

  // OpenGL constants
  const int gMaxShaderTextures = 13;        // conservative, OpenGL 3.3 specification requires at least 16
  const int gMaxShaderCubemapTextures = 2;  // conservative, OpenGL 3.3 specification requires at least 16
  const int gMaxTextureUnits = 48;          // taken from OpenGL 3.3 specification, section 6.2

  const int gMaxVerticesPerMeshForShadowRendering =
    65535;  // don't cast shadows for meshes with more than this number of vertices

}  // namespace wren

#endif  // CONSTANTS_HPP
