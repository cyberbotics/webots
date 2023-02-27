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

#ifndef OVERLAY_HPP
#define OVERLAY_HPP

#include "GlslLayout.hpp"
#include "Texture.hpp"

#include <vector>

namespace wren {

  class Renderable;
  class ShaderProgram;
  class StaticMesh;
  class Viewport;

  // Specialized class for rendering a 2d overlay consisting of up to 3 texture layers, with an optional border
  class Overlay {
  public:
    // Encapsulate memory management
    static Overlay *createOverlay() { return new Overlay(); }
    static void deleteOverlay(Overlay *overlay) { delete overlay; }

    static void setDefaultSizeProgram(ShaderProgram *program) { cDefaultSizeProgram = program; }

    void setOrder(int order) { mOrder = order; };
    void putOnTop();
    void setVisible(bool isVisible) { mIsVisible = isVisible; }
    void showDefaultSize(bool show) { mShowDefaultSize = show; }
    void setDefaultSize(float width, float height) {
      mParams.mDefaultSize = glm::vec4(width, height, mParams.mDefaultSize.z, mParams.mDefaultSize.w);
    }
    void setUsePremultipliedAlpha(bool enabled) { mPremultipliedAlpha = enabled; }
    void setAnisotropy(float anisotropy) { mTextureParams.mAnisotropy = anisotropy; }
    void setTranslucency(bool enabled) { mParams.mTextureFlags.w = enabled ? 1.0f : 0.0f; }
    void enableInterpolation(bool enabled) { mTextureParams.mIsInterpolationEnabled = enabled; }
    void enableMipMaps(bool enabled) { mTextureParams.mAreMipMapsEnabled = enabled; }

    void setSize(float width, float height) {
      assert(width >= 0.0f && height >= 0.0f);
      mParams.mPositionAndSize.z = width;
      mParams.mPositionAndSize.w = height;
    }

    void setPosition(float x, float y) {
      mParams.mPositionAndSize.x = x;
      mParams.mPositionAndSize.y = y;
    }

    void setBackgroundColor(const glm::vec4 color);
    void setBorderActive(bool isBorderActive) { mParams.mActiveFlags.y = isBorderActive ? 1u : 0u; }
    void setBorderColor(const glm::vec4 color) { mParams.mBorderColor = color; }
    void setBorderSize(float sizeHorizontal, float sizeVertical) {
      mParams.mBorderSize = glm::vec2(sizeHorizontal, sizeVertical);
    }

    void setTexture(Texture *texture, int role);
    void setTextureFlipVertical(bool flip) { mParams.mTextureFlags.x = flip ? 1.0f : 0.0f; }
    void addAdditionalTexture(Texture *texture);
    void setProgram(ShaderProgram *program) { mProgram = program; }

    void setMaxRange(float maxRange) { mParams.mTextureFlags.z = maxRange; }

    bool isVisible() const { return mIsVisible; }
    float x() const { return mParams.mPositionAndSize.x; }
    float y() const { return mParams.mPositionAndSize.y; }
    float width() const { return mParams.mPositionAndSize.z; }
    float height() const { return mParams.mPositionAndSize.w; }
    int order() const { return mOrder; }

    void setCachePersistency(bool isPersistent);

    void render();

  private:
    Overlay();
    ~Overlay();

    void prepareTexture(Texture *texture, int textureUnitIndex);

    static ShaderProgram *cDefaultSizeProgram;
    static size_t cMaxOrder;
    static std::vector<Overlay *> cOverlays;

    int mOrder;  // overlays are sorted in ascending order before rendering
    bool mIsVisible;
    bool mShowDefaultSize;
    bool mPremultipliedAlpha;

    std::vector<Texture *> mTextures;
    std::vector<Texture *> mAdditionalTextures;
    Texture::UsageParams mTextureParams;

    StaticMesh *mMesh;
    ShaderProgram *mProgram;
    GlslLayout::Overlay mParams;
  };

}  // namespace wren

#endif  // OVERLAY_HPP
