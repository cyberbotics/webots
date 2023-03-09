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

#ifndef GL_STATE_HPP
#define GL_STATE_HPP

#include "Constants.hpp"
#include "GlslLayout.hpp"

#include <memory>
#include <vector>

namespace wren {

  class UniformBuffer;

  namespace cache {
    struct PhongMaterialData;
    struct PbrMaterialData;
  }  // namespace cache

  // Stores part of the current state of the OpenGL state machine in order
  // to avoid unecessary state changes.
  namespace glstate {

    void init();
    bool isInitialized();
    bool isContextActive();

    void setContextActive(bool active);
    void setDefaultState();
    void setDepthTest(bool enable);
    void setDepthClamp(bool enable);
    void setDepthFunc(unsigned int func);
    void setDepthMask(bool enable);
    void setStencilTest(bool enable);
    void setStencilFunc(unsigned int func, int ref, unsigned int mask);
    void setStencilOp(unsigned int sfail, unsigned int dpfail, unsigned int dppass);
    void setStencilOpFront(unsigned int sfail, unsigned int dpfail, unsigned int dppass);
    void setStencilOpBack(unsigned int sfail, unsigned int dpfail, unsigned int dppass);
    void setStencilMask(unsigned int mask);
    void setColorMask(bool red, bool green, bool blue, bool alpha);
    void setCullFace(bool enable);
    void setCullFaceMode(unsigned int mode);
    void setFrontFace(unsigned int mode);
    void setBlend(bool enable);
    void setBlendFunc(unsigned int srcFactor, unsigned int destFactor);
    void setBlendEquation(unsigned int mode);
    void setBlendEquationSeparate(unsigned int modeRgb, unsigned int modeAlpha);
    void setPolygonMode(unsigned int polygonMode);
    void setPolygonOffset(bool enable, float factor, float units);
    void setClearColor(const glm::vec4 &clearColor);
    // Activate gl_PointSize in vertex shaders
    void enablePointSize(bool enable);

    // A texture must be bound to a texture unit in order to modify its parameters
    void activateTextureUnit(int textureUnit);
    void setTextureWrapS(unsigned int glName, int textureUnit, int mode);
    void setTextureWrapT(unsigned int glName, int textureUnit, int mode);
    void setTextureBorderColor(unsigned int glName, int textureUnit, const glm::vec4 &color);
    void setTextureAnisotropy(unsigned int glName, int textureUnit, float anisotropy);
    void setTextureInterpolation(unsigned int glName, int textureUnit, bool useInterpolation, bool useMipMapping);

    void initializeTextureParams(unsigned int glName);
    void clearTextureParams(unsigned int glName);

    void bindProgram(unsigned int glName);
    void bindVertexArrayObject(unsigned int glName);
    void bindElementArrayBuffer(unsigned int glName);
    void bindTexture(unsigned int glName, int textureUnit);
    void bindTextureCubeMap(unsigned int glName, int textureUnit);
    void bindFrameBuffer(unsigned int glName);
    void bindDrawFrameBuffer(unsigned int glName);
    void bindReadFrameBuffer(unsigned int glName);
    void bindPixelPackBuffer(unsigned int glName);
    void bindRenderBuffer(unsigned int glName);
    void bindUniformBuffer(unsigned int glName, unsigned int binding);
    void bindPhongMaterial(const cache::PhongMaterialData *materialData);
    void bindPbrMaterial(const cache::PbrMaterialData *materialData);

    void releaseProgram(unsigned int glName);
    void releaseVertexArrayObject(unsigned int glName);
    void releaseElementArrayBuffer(unsigned int glName);
    void releaseTexture(unsigned int glName, int textureUnit);
    void releaseTextureCubeMap(unsigned int glName, int textureUnit);
    void releaseFrameBuffer(unsigned int glName);
    void releaseDrawFrameBuffer(unsigned int glName);
    void releaseReadFrameBuffer(unsigned int glName);
    void releasePixelPackBuffer(unsigned int glName);
    void releaseRenderBuffer(unsigned int glName);
    void releaseUniformBuffer(unsigned int glName, unsigned int binding);
    void releasePhongMaterial(const cache::PhongMaterialData *materialData);
    void releasePbrMaterial(const cache::PbrMaterialData *materialData);

    unsigned int boundReadFrameBuffer();
    unsigned int boundDrawFrameBuffer();
    unsigned int boundPixelPackBuffer();

    unsigned int blendSrcFactor();
    unsigned int blendDestFactor();

    unsigned int getFrontFace();

    const char *vendor();
    const char *renderer();
    const char *version();
    const char *glslVersion();

    // This function returns different values for each GPU vendor:
    //  - NVIDIA: returns the total memory.
    //  - AMD: returns the free memory.
    //  - intel: returns always 0 (no dedicated memory)
    int gpuMemory();

    int maxCombinedTextureUnits();
    int maxFrameBufferDrawBuffers();
    float maxTextureAnisotropy();

    unsigned int activeProgram();
    const UniformBuffer *uniformBuffer(WrGlslLayoutUniformBuffer buffer);

    // Prints the error code(s) if OpenGL errors have ocurred since the last invocation
    void checkError(int ignore = 0);

  }  // namespace glstate
}  // namespace wren

#endif  // GL_STATE_HPP
