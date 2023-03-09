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

#ifndef DYNAMIC_MESH_HPP
#define DYNAMIC_MESH_HPP

#include "Constants.hpp"
#include "Id.hpp"
#include "Mesh.hpp"
#include "Primitive.hpp"
#include "ShadowVolumeCaster.hpp"
#include "Skeleton.hpp"

#include <vector>

/**
  Class implementation info:

  The class is optimised under the assumption that the mesh topology will not change (or not very often), the contrary may lead
  to severe performance drop since the mesh triangles & edges have to be recomputed for the shadow volume.
*/

namespace wren {

  class DynamicMesh : public Mesh {
  public:
    static DynamicMesh *createDynamicMesh(bool hasNormals, bool hasTextureCoordinates, bool hasColorPerVertex) {
      return new DynamicMesh(hasNormals, hasTextureCoordinates, hasColorPerVertex);
    }

    void addCoord(const glm::vec3 &coord, bool forceShadowCoords = false) override;
    void addNormal(const glm::vec3 &normal) override;
    void addTexCoord(const glm::vec2 &texCoord) override;
    void addIndex(unsigned int index) override;
    void addColor(const glm::vec3 &color) override;

    void setSkeleton(Skeleton *skeleton) { mSkeleton = skeleton; }

    void bind() override;
    void release() override;
    void render(unsigned int drawingMode) override;
    void clear() override {
      clear(true, mHasNormals, mHasTextureCoordinates, mHasColorPerVertex);
      Mesh::clear();
    }
    void clear(bool vertices, bool normals, bool textureCoordinates, bool colors);

    void bindShadowVolume() override;
    void releaseShadowVolume() override;
    void setShadowVolume(ShadowVolumeCaster *shadowVolume) { mShadowVolume = shadowVolume; }

    const std::vector<glm::vec4> &shadowCoords() const override { return mShadowCoords; }
    virtual unsigned int glNameShadowCoords() const { return mGlNameBufferShadowCoords; }
    const std::vector<Edge> &edges() const override { return mEdges; }
    const std::vector<Mesh::Triangle> &triangles() const override { return mTriangles; }
    const Triangle &triangle(size_t index) const override { return mTriangles[index]; }

    size_t sortingId() const override;

    primitive::Aabb recomputeAabb(const glm::vec3 &scale = gVec3Ones) override;
    primitive::Sphere recomputeBoundingSphere(const glm::vec3 &scale = gVec3Ones) override;

    bool isAabbDirty() const override { return mAabbDirty; };
    bool isBoundingSphereDirty() const override { return mBoundingSphereDirty; }
    bool isDynamic() const override { return true; }
    bool supportShadows() const override { return mSupportShadows; }

    void applySkeletonOffset();
    void updateSkeletonOffset(const glm::mat4 &offsetMatrix);

    void applySkeletonTransform();
    void notifySkeletonDirty() { mSkeletonDirty = true; }

  protected:
    DynamicMesh(bool hasNormals, bool hasTextureCoordinates, bool hasColorPerVertex);
    virtual ~DynamicMesh(){};

  private:
    void updateGl();
    void updateGlShadow();
    void computeTrianglesAndEdges();

    void prepareGl() override;
    void cleanupGl() override;

    const bool mHasNormals;
    const bool mHasTextureCoordinates;
    const bool mHasColorPerVertex;

    bool mCoordsDirty;
    bool mIndicesDirty;
    bool mNormalsDirty;
    bool mTexCoordsDirty;
    bool mColorsDirty;
    bool mAabbDirty;
    bool mAabbComputed;
    bool mBoundingSphereDirty;
    bool mSkeletonDirty;
    bool mSupportShadows;

    bool mShadowCoordsDirty;
    bool mShadowIndicesDirty;

    unsigned int mGlNameVertexArrayObject;
    unsigned int mGlNameBufferIndices;
    unsigned int mGlNameBufferCoords;
    unsigned int mGlNameBufferNormals;
    unsigned int mGlNameBufferTexCoords;
    unsigned int mGlNameBufferColors;

    unsigned int mGlNameVertexArrayObjectShadow;
    unsigned int mGlNameBufferShadowCoords;

    std::vector<glm::vec4> mShadowCoords;
    std::vector<unsigned int> mShadowIndices;
    std::vector<Mesh::Triangle> mTriangles;
    std::vector<Mesh::Edge> mEdges;

    // Used for skeletal animation
    // These contain the original vertices & normals of the mesh
    std::vector<glm::vec3> mCoordsCopy;
    std::vector<glm::vec3> mNormalsCopy;

    // These contain the vertices & normals transformed by the skeleton transform
    std::vector<glm::vec3> mSkinCoords;
    std::vector<glm::vec3> mSkinNormals;

    primitive::Aabb mAabb;
    primitive::Sphere mBoundingSphere;
    IdMesh mMeshId;

    ShadowVolumeCaster *mShadowVolume;

    Skeleton *mSkeleton;
  };

}  // namespace wren

#endif  // MESH_HPP
