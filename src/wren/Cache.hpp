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

#ifndef CACHE_HPP
#define CACHE_HPP

#include "Constants.hpp"
#include "GlslLayout.hpp"
#include "Id.hpp"
#include "Mesh.hpp"
#include "Primitive.hpp"

namespace wren {

  class StaticMesh;
  class Texture2d;

  namespace cache {

    uint64_t sipHash13c(const char *bytes, const size_t size);

    // Generic key struct for hashmaps
    struct Key {
      Key();
      explicit Key(const uint64_t hashValue);
      bool operator==(const Key &other) const;

      size_t mHash;
    };

    // The actual data necessary to use a cached resource
    struct PhongMaterialData : public IdPhongMaterial {
      explicit PhongMaterialData(const GlslLayout::PhongMaterial &material);

      size_t mNumUsers;
      GlslLayout::PhongMaterial mMaterial;
    };

    // The actual data necessary to use a cached resource
    struct PbrMaterialData : public IdPbrMaterial {
      explicit PbrMaterialData(const GlslLayout::PbrMaterial &material);

      size_t mNumUsers;
      GlslLayout::PbrMaterial mMaterial;
    };

    struct MeshData : public IdMesh {
      MeshData();
      ~MeshData() {}

      size_t mNumUsers;

      unsigned int mGlNameVertexArrayObject;
      unsigned int mGlNameBufferCoords;
      unsigned int mGlNameBufferNormals;
      unsigned int mGlNameBufferTexCoords;
      unsigned int mGlNameBufferIndices;
      unsigned int mGlNameBufferColors;
      unsigned int mGlNameBufferUnwrappedTexCoords;

      unsigned int mGlNameVertexArrayObjectShadow;
      unsigned int mGlNameBufferShadowCoords;

      unsigned int mIndexCount;
      unsigned int mVertexCount;

      bool mIsCachePersistent;

      bool mSupportShadows;
      std::vector<glm::vec4> mShadowCoords;

      std::vector<Mesh::Triangle> mTriangles;
      std::vector<Mesh::Edge> mEdges;

      // bounding volumes for unit mesh
      primitive::Sphere mBoundingSphere;
      primitive::Aabb mAabb;
    };

    struct Texture2dData {
      explicit Texture2dData(const Texture2d &texture);

      size_t mNumUsers;
      unsigned int mGlName;
      int mWidth;
      int mHeight;
      bool mIsTranslucent;
      bool mIsCachePersistent;
    };

  }  // namespace cache
}  // namespace wren

namespace std {

  // Overloaded methods for std::hash
  template<> struct hash<wren::cache::Key> { size_t operator()(const wren::cache::Key &key) const; };

  template<> struct hash<glm::vec3> { size_t operator()(const glm::vec3 &key) const; };

  template<> struct hash<std::pair<size_t, size_t>> { size_t operator()(const std::pair<size_t, size_t> &key) const; };

}  // namespace std

#endif  // CACHE_UTILS_HPP
