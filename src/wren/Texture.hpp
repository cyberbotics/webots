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

#ifndef TEXTURE_HPP
#define TEXTURE_HPP

#include "GlUser.hpp"
#include "TextureTransform.hpp"

#include <wren/texture.h>

namespace wren {

  class Material;
  // Base class defining parameters common to all textures, and virtual functions
  // that subclasses need to implement.
  class Texture : public GlUser {
  public:
    struct UsageParams {
      UsageParams(WrTextureWrapMode wrapS, WrTextureWrapMode wrapT, WrTextureWrapMode wrapR, const glm::vec4 &borderColor,
                  float anisotropy, bool enableInterpolation, bool enableMipMaps) :
        mWrapS(wrapS),
        mWrapT(wrapT),
        mWrapR(wrapR),
        mBorderColor(borderColor),
        mAnisotropy(anisotropy),
        mIsInterpolationEnabled(enableInterpolation),
        mAreMipMapsEnabled(enableMipMaps) {}

      WrTextureWrapMode mWrapS;
      WrTextureWrapMode mWrapT;
      WrTextureWrapMode mWrapR;
      glm::vec4 mBorderColor;
      float mAnisotropy;
      bool mIsInterpolationEnabled;
      bool mAreMipMapsEnabled;
    };

    struct GlFormatParams {
      GlFormatParams(int internalFormat, unsigned int format, unsigned int dataType, int pixelSize, int channelCount) :
        mInternalFormat(internalFormat),
        mFormat(format),
        mDataType(dataType),
        mPixelSize(pixelSize),
        mChannelCount(channelCount) {}

      int mInternalFormat;
      unsigned int mFormat;
      unsigned int mDataType;
      int mPixelSize;
      int mChannelCount;
    };

    static const UsageParams DEFAULT_USAGE_PARAMS;
    static const GlFormatParams GL_FORMAT_PARAMS[WR_TEXTURE_INTERNAL_FORMAT_COUNT];

    static char *generateBlackData(const GlFormatParams &glFormatParams, int width, int height);

    // Encapsulate memory management
    static unsigned int generateNewTexture();
    static void deleteTexture(Texture *texture);

    void setInternalFormat(WrTextureInternalFormat format);
    void setTextureUnit(int textureUnit);
    void setTranslucent(bool translucent) { mIsTranslucent = translucent; }
    void changeData(void *data, int x, int y, int width, int height);

    int textureUnit() const { return mTextureUnit; }
    int width() const { return mWidth; }
    int height() const { return mHeight; }
    const GlFormatParams &glFormatParams() const { return mGlFormatParams; }
    bool isTranslucent() const { return mIsTranslucent; }

    virtual void setSize(int width, int height);
    virtual void setup();
    virtual void generateMipMaps(bool regenerate = false);
    virtual void bind(const Texture::UsageParams &params);
    virtual void release();
    virtual WrTextureType type() const = 0;
    virtual unsigned int glName() const = 0;
    virtual bool hasPremultipliedAlpha() const { return false; }

    void addMaterialUser(Material *material);
    void removeMaterialUser(Material *material);

  protected:
    Texture();
    virtual ~Texture();

    virtual void setGlName(unsigned int glName) = 0;
    void cleanupGl() override;

    GlFormatParams mGlFormatParams;
    int mTextureUnit;
    int mWidth;
    int mHeight;
    glm::vec4 mBorderColor;
    bool mHaveMipMapsBeenGenerated;
    bool mIsTranslucent;
    std::vector<Material *> mMaterialsUsingThisTexture;
  };

}  // namespace wren

#endif  // TEXTURE_HPP
