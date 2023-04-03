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

#include "Cache.hpp"

#include "Debug.hpp"
#include "StaticMesh.hpp"
#include "Texture2d.hpp"

#include "sip_hash.hpp"

#include <unordered_map>

namespace wren {
  namespace cache {

    static const highwayhash::HH_U64 SIPHASH_KEY[2] = {
      0x4242424242424242ull,
      0x4242424242424242ull,
    };

    uint64_t sipHash13c(const char *bytes, const size_t size) {
      return highwayhash::SipHash13(SIPHASH_KEY, bytes, size);
    }

    Key::Key() : mHash(0) {
    }

    Key::Key(const uint64_t hashValue) {
      mHash = hashValue;
    }

    bool Key::operator==(const Key &other) const {
      return mHash == other.mHash;
    }

    PhongMaterialData::PhongMaterialData(const GlslLayout::PhongMaterial &material) : mNumUsers(1), mMaterial(material) {
    }

    PbrMaterialData::PbrMaterialData(const GlslLayout::PbrMaterial &material) : mNumUsers(1), mMaterial(material) {
    }

    MeshData::MeshData() :
      mNumUsers(1),
      mGlNameVertexArrayObject(0),
      mGlNameBufferCoords(0),
      mGlNameBufferNormals(0),
      mGlNameBufferTexCoords(0),
      mGlNameBufferIndices(0),
      mGlNameBufferColors(0),
      mGlNameBufferUnwrappedTexCoords(0),
      mGlNameVertexArrayObjectShadow(0),
      mGlNameBufferShadowCoords(0),
      mIndexCount(0),
      mVertexCount(0),
      mIsCachePersistent(false),
      mSupportShadows(true) {
    }

    Texture2dData::Texture2dData(const Texture2d &texture) :
      mNumUsers(1),
      mGlName(0),
      mWidth(texture.width()),
      mHeight(texture.height()),
      mIsTranslucent(texture.isTranslucent()),
      mIsCachePersistent(texture.isCachePersistent()) {
    }

  }  // namespace cache
}  // namespace wren

namespace std {

  size_t hash<wren::cache::Key>::operator()(const wren::cache::Key &key) const {
    return key.mHash;
  }

  size_t hash<glm::vec3>::operator()(const glm::vec3 &key) const {
    static constexpr size_t bitcount = 8 * sizeof(size_t);
    size_t val = *(reinterpret_cast<const char *>(&key.x));
    size_t result = val;

    val = *reinterpret_cast<const char *>(&key.y);
    size_t shift = bitcount / 3;
    result ^= val << shift | val >> (bitcount - shift);

    val = *reinterpret_cast<const char *>(&key.z);
    shift *= 2;
    result ^= val << shift | val >> (bitcount - shift);

    return result;
  }

  size_t hash<std::pair<size_t, size_t>>::operator()(const std::pair<size_t, size_t> &key) const {
    return std::hash<size_t>{}(key.first) ^ (std::hash<size_t>{}(key.second) << (sizeof(size_t) >> 1));
  }

}  // namespace std
