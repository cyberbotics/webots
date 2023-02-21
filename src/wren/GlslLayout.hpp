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

#ifndef GLSL_LAYOUT_HPP
#define GLSL_LAYOUT_HPP

#include "Constants.hpp"

#include <vector>

#include <wren/glsl_layout.h>

// Describes the semantics between C++ code and GLSL code,
// such as the location of vertex shader attributes and the names and sizes of uniform buffers.

// Procedure for adding a new uniform:
//
// 1. Add an entry to the WrGlslLayoutUniform enum
// 2. When creating a shader that uses this uniform, call ShaderProgram::useUniform with the enum value as argument
// 3. When modifying the data using glUniform, call ShaderProgram::uniformLocation with the enum value to get the location

// Procedure for adding a new uniform buffer or extending an existing one:
//
// 1. Add an entry to the WrGlslLayoutUniformBuffer enum
// 2. Define the uniform buffer struct below, following GLSL std140 rules for its member offsets
// 3. Add the name used in the shader definition to gUniformBufferNames, respecting the order in enum UniformBuffer
// 4. Add the size of the buffer to gUniformBufferSizes, respecting the order in enum UniformBuffer
//
// After following these steps, the buffer will be automatically created on initialization and its binding point
// will be associated with the correct index for each shader program (see ShaderProgram::prepareGl).
//
// 5. When creating a shader that uses this uniform buffer, call ShaderProgram::useUniformBuffer with the enum value as argument
// 6. Modify the data inside the buffer by calling UniformBuffer::writeValue

namespace wren {
  namespace GlslLayout {
    // Uniform buffer structs (GLSL std140 layout rules)
    // https://khronos.org/registry/OpenGL/specs/gl/glspec45.core.pdf#page=159
    struct PhongMaterial {
      glm::vec4 mAmbient;
      glm::vec4 mDiffuse;
      glm::vec4 mSpecularAndExponent;
      glm::vec4 mEmissiveAndOpacity;
      glm::vec4 mTextureFlags;  // x, y, z, w: materialTexture[0]..[3]
    };

    struct PbrMaterial {
      glm::vec4 mBaseColorAndTransparency;
      glm::vec4 mRoughnessMetalnessNormalMapFactorOcclusion;
      glm::vec4 mBackgroundColorAndIblStrength;
      glm::vec4 mEmissiveColorAndIntensity;
      glm::vec4 mBaseColorRoughnessMetalnessOcclusionMapFlags;  // x, y, z, w: materialTexture[0]..[3]
      glm::vec4 mNormalBrdfEmissiveBackgroundFlags;
      glm::vec4 mPenFlags;
      glm::vec4 mCubeTextureFlags;
    };

    struct DirectionalLight {
      glm::vec4 mColorAndIntensity;
      glm::vec4 mDirection;
    };

    struct PointLight {
      glm::vec4 mColorAndIntensity;
      glm::vec4 mPosition;
      glm::vec4 mAttenuationAndRadius;
    };

    struct SpotLight {
      glm::vec4 mColorAndIntensity;
      glm::vec4 mPosition;
      glm::vec4 mDirection;
      glm::vec4 mAttenuationAndRadius;
      glm::vec4 mSpotParams;
    };

    struct Lights {
      DirectionalLight mDirectionalLights[gMaxActiveDirectionalLights];
      PointLight mPointLights[gMaxActivePointLights];
      SpotLight mSpotLights[gMaxActiveSpotLights];
      glm::vec4 mAmbientLight;
      glm::ivec3 mLightCount;
      float pad;  // pad the struct to get the same size as the std140 struct in shaders where vec3 is counted as a vec4
    };

    // Used for doing one pass per light
    struct LightRenderable {
      glm::ivec4 mActiveLights;
    };

    struct Fog {
      glm::vec2 mMode;    // x: disabled, exp, exp2, y: depth, distance
      glm::vec2 padding;  // unused
      glm::vec4 mParams;  // x: density, y: density^2, z: end, w: inv. scale
      glm::vec4 mColor;
    };

    struct Overlay {
      glm::vec4 mPositionAndSize;  // in percentage of the OpenGL viewport size
      glm::vec4 mDefaultSize;      // x,y: size, z: render default size instead of actual overlay
      glm::vec4 mBorderColor;
      glm::vec4 mBackgroundColor;
      glm::vec4 mTextureFlags;  // x: flip vertically, y: additional texture count, z:  maxRange (depth textures only),
                                // w: overlay transparency
      glm::uvec2 mActiveFlags;  // x: active textures, y: border
      glm::vec2 mSizeInPixels;  // x,y: size in screen pixels
      glm::vec2 mBorderSize;    // x: vertical size, y: horizontal size
    };

    struct CameraTransforms {
      glm::mat4 mView;
      glm::mat4 mProjection;
      glm::mat4 mInfiniteProjection;
    };

    // Vertex attribute locations in vertex shader
    extern const unsigned int gLocationCoords;
    extern const unsigned int gLocationNormals;
    extern const unsigned int gLocationTexCoords;
    extern const unsigned int gLocationColors;
    extern const unsigned int gLocationUnwrappedTexCoords;

    extern const std::vector<const char *> gUniformNames;
    extern const std::vector<const char *> gUniformBufferNames;
    extern const std::vector<int> gUniformBufferSizes;

  }  // namespace GlslLayout
}  // namespace wren

#endif
