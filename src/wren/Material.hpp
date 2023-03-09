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

#ifndef MATERIAL_HPP
#define MATERIAL_HPP

#include "Cache.hpp"
#include "Constants.hpp"
#include "Texture.hpp"

#include <unordered_map>

#include <wren/material.h>

namespace wren {

  class ShaderProgram;
  class TextureTransform;
  class TextureCubeMap;

  // Abstract Container class that stores the Material parameters, Shader and Texture used by a Renderable.
  // Inherited by: PhongMaterial, PBRMaterial
  class Material {
  public:
    enum MaterialProgram {
      MATERIAL_PROGRAM_DEFAULT,
      MATERIAL_PROGRAM_STENCIL_AMBIENT_EMISSIVE,
      MATERIAL_PROGRAM_STENCIL_DIFFUSE_SPECULAR,
    };

    virtual WrMaterialType type() const { return WR_MATERIAL_NONE; }

    // Clear all the data but preserve the program
    virtual void clearMaterial() = 0;

    virtual void setColor(const glm::vec3 &color) {}
    virtual void setTransparency(float transparency) = 0;

    virtual void setTexture(Texture *texture, size_t index);
    void setTextureWrapS(WrTextureWrapMode mode, size_t index) {
      assert(index < mTextures.size());
      mTextures[index].second.mWrapS = mode;
    }

    void setTextureWrapT(WrTextureWrapMode mode, size_t index) {
      assert(index < mTextures.size());
      mTextures[index].second.mWrapT = mode;
    }

    void setTextureAnisotropy(float anisotropy, size_t index) {
      assert(index < mTextures.size());
      mTextures[index].second.mAnisotropy = anisotropy;
    }

    void enableTextureInterpolation(bool enable, size_t index) {
      assert(index < mTextures.size());
      mTextures[index].second.mIsInterpolationEnabled = enable;
    }

    void enableTextureMipMaps(bool enable, size_t index) {
      assert(index < mTextures.size());
      mTextures[index].second.mAreMipMapsEnabled = enable;
    }

    WrMaterial *materialStructure() const { return mMaterialStructure; }
    void setMaterialStructure(WrMaterial *material) { mMaterialStructure = material; }

    bool hasPremultipliedAlpha() const { return mHasPremultipliedAlpha; }

    virtual void setTextureCubeMap(TextureCubeMap *texture, size_t index);
    void setTextureCubeMapWrapR(WrTextureWrapMode mode, int index) { mTextureCubes[index].second.mWrapR = mode; }
    void setTextureCubeMapWrapS(WrTextureWrapMode mode, int index) { mTextureCubes[index].second.mWrapS = mode; }
    void setTextureCubeMapWrapT(WrTextureWrapMode mode, int index) { mTextureCubes[index].second.mWrapT = mode; }
    void setTextureCubeMapAnisotropy(float anisotropy, int index) { mTextureCubes[index].second.mAnisotropy = anisotropy; }
    void enableTextureCubeMapInterpolation(bool enable, int index) {
      mTextureCubes[index].second.mIsInterpolationEnabled = enable;
    }
    void enableTextureCubeMapMipMaps(bool enable, int index) { mTextureCubes[index].second.mAreMipMapsEnabled = enable; }

    void setTextureTransform(TextureTransform *transform);
    void setDefaultProgram(ShaderProgram *program) { mDefaultProgram = program; }
    void setStencilAmbientEmissiveProgram(ShaderProgram *program) { mStencilAmbientEmissiveProgram = program; }
    void setStencilDiffuseSpecularProgram(ShaderProgram *program) { mStencilDiffuseSpecularProgram = program; }
    void setEffectiveProgram(MaterialProgram program) {
      switch (program) {
        case MATERIAL_PROGRAM_DEFAULT:
          mEffectiveProgram = mDefaultProgram;
          break;
        case MATERIAL_PROGRAM_STENCIL_AMBIENT_EMISSIVE:
          mEffectiveProgram = mStencilAmbientEmissiveProgram;
          break;
        case MATERIAL_PROGRAM_STENCIL_DIFFUSE_SPECULAR:
          mEffectiveProgram = mStencilDiffuseSpecularProgram;
          break;
        default:
          assert(false);
      }

      assert(mEffectiveProgram);
    }

    virtual void setColorPerVertex(bool enabled) {}

    ShaderProgram *defaultProgram() const { return mDefaultProgram; }
    ShaderProgram *stencilAmbientEmissiveProgram() const { return mStencilAmbientEmissiveProgram; }
    ShaderProgram *stencilDiffuseSpecularProgram() const { return mStencilDiffuseSpecularProgram; }
    ShaderProgram *effectiveProgram() const { return mEffectiveProgram; }
    Texture *texture(int index) const {
      assert(index < gMaxShaderTextures);
      return mTextures[index].first;
    }

    bool isTranslucent() const { return mIsTranslucent; }

    WrTextureWrapMode textureWrapS(size_t index) const {
      assert(index < mTextures.size());
      return mTextures[index].second.mWrapS;
    }

    WrTextureWrapMode textureWrapT(size_t index) const {
      assert(index < mTextures.size());
      return mTextures[index].second.mWrapT;
    }

    float textureAnisotropy(size_t index) const {
      assert(index < mTextures.size());
      return mTextures[index].second.mAnisotropy;
    }

    bool isTextureInterpolationEnabled(size_t index) const {
      assert(index < mTextures.size());
      return mTextures[index].second.mIsInterpolationEnabled;
    }

    bool areTextureMipMapsEnabled(size_t index) const {
      assert(index < mTextures.size());
      return mTextures[index].second.mAreMipMapsEnabled;
    }
    void removeDeletedTexture(const Texture *texture);
    virtual void bind(bool bindProgram = true) const = 0;
    virtual size_t sortingId() const = 0;

  protected:
    Material();
    virtual ~Material();

    virtual void updateTranslucency();
    virtual void releaseMaterial() = 0;

    void useProgram() const;
    void bindTextures() const;
    virtual void updateUniforms() const;
    int countTextureInstances(const Texture *texture) const;

    bool mHasPremultipliedAlpha;
    bool mIsTranslucent;

    std::vector<std::pair<Texture *, Texture::UsageParams>> mTextures;
    std::vector<std::pair<TextureCubeMap *, Texture::UsageParams>> mTextureCubes;

    TextureTransform *mTextureTransform;
    WrMaterial *mMaterialStructure;

    ShaderProgram *mDefaultProgram;
    ShaderProgram *mStencilAmbientEmissiveProgram;
    ShaderProgram *mStencilDiffuseSpecularProgram;
    ShaderProgram *mEffectiveProgram;
  };

}  // namespace wren

#endif  // MATERIAL_HPP
