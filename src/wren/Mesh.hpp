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

#ifndef MESH_HPP
#define MESH_HPP

#include "Constants.hpp"
#include "GlUser.hpp"

#include <vector>

namespace wren {

  class Mesh : public GlUser {
  public:
    struct Triangle {
      Triangle(unsigned int vertex0, unsigned int vertex1, unsigned int vertex2, glm::vec3 normal) :
        mVertexIndices{vertex0, vertex1, vertex2},
        mNormal(normal),
        mIsFacingLight(false) {}

      unsigned int mVertexIndices[3];
      glm::vec3 mNormal;
      mutable bool mIsFacingLight;
    };

    struct Edge {
      Edge(size_t triangle0, size_t triangle1, unsigned int vertex0, unsigned int vertex1) :
        mTriangleIndices{triangle0, triangle1},
        mVertexIndices{vertex0, vertex1} {}

      size_t mTriangleIndices[2];
      unsigned int mVertexIndices[2];
    };

    static void deleteMesh(Mesh *mesh);

    void estimateVertexCount(int count) {
      mCoords.reserve(count);
      mNormals.reserve(count);
      mTexCoords.reserve(count);
    }

    void estimateIndexCount(int count) { mIndices.reserve(count); }

    void setCoords(const std::vector<glm::vec3> &coords) { mCoords = coords; }
    void setNormals(const std::vector<glm::vec3> &normals) { mNormals = normals; }
    void setTexCoords(const std::vector<glm::vec2> &texCoords) { mTexCoords = texCoords; }
    void setIndices(const std::vector<unsigned int> &indices) { mIndices = indices; }
    void setColors(const std::vector<glm::vec3> &colors) { mColors = colors; }

    virtual void addCoord(const glm::vec3 &coord, bool forceShadowCoords = false) { mCoords.push_back(coord); }
    virtual void addNormal(const glm::vec3 &normal) { mNormals.push_back(normal); }
    virtual void addTexCoord(const glm::vec2 &texCoord) { mTexCoords.push_back(texCoord); }
    virtual void addUnwrappedTexCoord(const glm::vec2 &texCoord) { mUnwrappedTexCoords.push_back(texCoord); }
    virtual void addIndex(unsigned int index) { mIndices.push_back(index); }
    virtual void addColor(const glm::vec3 &color) { mColors.push_back(color); }

    const std::vector<glm::vec3> &coords() const { return mCoords; }
    const std::vector<glm::vec3> &normals() const { return mNormals; }
    const std::vector<glm::vec2> &texCoords() const { return mTexCoords; }
    const std::vector<glm::vec2> &unwrappedTexCoords() const { return mUnwrappedTexCoords; }
    const std::vector<unsigned int> &indices() const { return mIndices; }
    const std::vector<glm::vec3> &colors() const { return mColors; }

    std::vector<glm::vec3> &coords() { return mCoords; }
    std::vector<glm::vec3> &normals() { return mNormals; }
    std::vector<glm::vec2> &texCoords() { return mTexCoords; }
    std::vector<glm::vec2> &unwrappedTexCoords() { return mUnwrappedTexCoords; }
    std::vector<unsigned int> &indices() { return mIndices; }
    std::vector<glm::vec3> &colors() { return mColors; }

    virtual const std::vector<glm::vec4> &shadowCoords() const = 0;
    virtual const std::vector<Edge> &edges() const = 0;
    virtual const std::vector<Mesh::Triangle> &triangles() const = 0;
    virtual const Triangle &triangle(size_t index) const = 0;

    void setup();
    virtual void bind() = 0;
    virtual void release() = 0;
    virtual void render(unsigned int drawingMode) = 0;
    virtual void clear() {
      mCoords.clear();
      mNormals.clear();
      mTexCoords.clear();
      mIndices.clear();
      mColors.clear();
      mUnwrappedTexCoords.clear();
    };

    virtual void bindShadowVolume() = 0;
    virtual void releaseShadowVolume() = 0;

    virtual size_t sortingId() const = 0;

    virtual primitive::Aabb recomputeAabb(const glm::vec3 &scale = gVec3Ones) = 0;
    virtual primitive::Sphere recomputeBoundingSphere(const glm::vec3 &scale = gVec3Ones) = 0;

    virtual bool isAabbDirty() const { return false; };
    virtual bool isBoundingSphereDirty() const { return false; };
    virtual bool isDynamic() const { return false; }
    virtual bool supportShadows() const = 0;

  protected:
    Mesh() {}
    virtual ~Mesh(){};

    std::vector<glm::vec3> mCoords;
    std::vector<glm::vec3> mNormals;
    std::vector<glm::vec2> mTexCoords;
    std::vector<unsigned int> mIndices;
    std::vector<glm::vec3> mColors;
    std::vector<glm::vec2> mUnwrappedTexCoords;
  };

}  // namespace wren

#endif  // MESH_HPP
