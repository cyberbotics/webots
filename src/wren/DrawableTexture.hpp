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

#ifndef DRAWABLE_TEXTURE_HPP
#define DRAWABLE_TEXTURE_HPP

#include "Texture.hpp"

#include "Font.hpp"

namespace wren {

  class Transform;

  class DrawableTexture : public Texture {
  public:
    static DrawableTexture *createDrawableTexture() { return new DrawableTexture(); }

    void setFont(Font *font) { mFont = font; }
    void setColor(float *color) { mColor = toInt(color); }
    void setAntialiasing(bool antialiasing) { mFontAntiAliasing = antialiasing; }
    void setUsePremultipliedAlpha(bool premultipliedAlpha) { mPremultipliedAlpha = premultipliedAlpha; }

    void clear();
    void drawText(const char *text, int x, int y);
    void drawPixel(int x, int y);

    bool hasPremultipliedAlpha() const override { return mPremultipliedAlpha; }
    void setSize(int width, int height) override;
    WrTextureType type() const override { return WR_TEXTURE_TYPE_DRAWABLE; }
    unsigned int glName() const override { return mGlName; }

  private:
    DrawableTexture();
    ~DrawableTexture();

    void updateDirtyRect(int x, int y);
    void resetDirtyRect() {
      mDirtyMinX = 0;
      mDirtyMaxX = 0;
      mDirtyMinY = 0;
      mDirtyMaxY = 0;
    }

    int drawChar(unsigned long c, int baselineX, int baselineY);  // returns character width in pixels
    int toInt(const float *color) const;

    void setGlName(unsigned int glName) override { mGlName = glName; }
    void prepareGl() override;
    void bind(const Texture::UsageParams &params) override;

    Font *mFont;

    int mClearColor;  // 4 components
    int mColor;
    bool mFontAntiAliasing;
    bool mPremultipliedAlpha;

    int *mData;
    bool mDirty;

    // Dirty rectangle
    int mDirtyMinX;
    int mDirtyMaxX;
    int mDirtyMinY;
    int mDirtyMaxY;

    unsigned int mGlName;
  };

}  // namespace wren

#endif
