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

#ifndef TEXTURE_2D_HPP
#define TEXTURE_2D_HPP

#include "Cache.hpp"
#include "Texture.hpp"

#include <string>
#include <unordered_map>

namespace wren {

  class Transform;

  // Basic 2d texture which takes its data in BGRA format.
  class Texture2d : public Texture {
  public:
    // Encapsulate memory management
    static Texture2d *createTexture2d() { return new Texture2d(); }
    // Return an instance if cached, NULL otherwise
    static Texture2d *copyFromCache(const std::string &filePath);

    static size_t cachedItemCount();
    static void printCacheContents();

    void setCachePersistency(bool persistent) { mIsCachePersistent = persistent; }

    bool isCachePersistent() const { return mIsCachePersistent; }

    void setData(const char *data) { mData = data; }
    // The file path is used to produce a hash instead of hashing the texture data,
    // which can be unbearably slow if there are many/big textures.
    // The user must ensure to always provide a file path, otherwise an assert will be triggered.
    void setFilePath(const std::string &filePath) { mFilePath.assign(filePath); }

    void setup() override;
    WrTextureType type() const override { return WR_TEXTURE_TYPE_2D; }
    unsigned int glName() const override { return mCacheData ? mCacheData->mGlName : 0; }

  private:
    static std::unordered_map<cache::Key, cache::Texture2dData> cCache;

    Texture2d();
    virtual ~Texture2d() {}

    void setGlName(unsigned int glName) override { mCacheData->mGlName = glName; };
    void prepareGl() override;
    void cleanupGl() override;

    const void *mData;

    std::string mFilePath;

    bool mIsCachePersistent;

    cache::Key mCacheKey;
    cache::Texture2dData *mCacheData;
  };

}  // namespace wren

#endif  // TEXTURE_2D_HPP
