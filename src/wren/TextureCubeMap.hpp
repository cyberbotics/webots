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

#ifndef TEXTURE_CUBEMAP_HPP
#define TEXTURE_CUBEMAP_HPP

#include "Texture.hpp"

#include <wren/texture_cubemap.h>

namespace wren {

  class Transform;

  class TextureCubeMap : public Texture {
  public:
    // Encapsulate memory management
    static TextureCubeMap *createTextureCubeMap() { return new TextureCubeMap(); }

    void setData(const char *data, WrTextureOrientation orientation) { mDatas[orientation] = data; }

    WrTextureType type() const override { return WR_TEXTURE_TYPE_CUBEMAP; }
    unsigned int glName() const override { return mGlName; }

    void generateMipMaps(bool regenerate = false) override;
    void bind(const Texture::UsageParams &params) override;
    void release() override;
    void setGlName(unsigned int glName) override { mGlName = glName; }
    void disableAutomaticMipMapGeneration() { mHaveMipMapsBeenGenerated = true; }

  private:
    TextureCubeMap();
    ~TextureCubeMap() {}

    void prepareGl() override;

    const void *mDatas[WR_TEXTURE_CUBEMAP_COUNT];

    unsigned int mGlName;
  };

}  // namespace wren

#endif  // TEXTURE_CUBEMAP_HPP
