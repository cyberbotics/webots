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

#ifndef STATIC_MESH_HPP
#define STATIC_MESH_HPP

#include "Cache.hpp"
#include "Constants.hpp"
#include "Mesh.hpp"

#include <unordered_map>
#include <vector>

namespace wren {

  // Mesh class, creates a cache entry on creation if necessary and stores a pointer on the cached data needed for rendering
  class StaticMesh : public Mesh {
  public:
    // Encapsulate memory management
    static StaticMesh *createStaticMesh() { return new StaticMesh(); }
    static bool createOrRetrieveFromCache(StaticMesh **mesh, const cache::Key key);  // returns true on cache hit

    // Static methods for creating meshes with a specific geometry
    static StaticMesh *createUnitBox(bool outline);
    static StaticMesh *createUnitCone(int subdivision, bool hasSide, bool hasBottom);
    static StaticMesh *createUnitCylinder(int subdivision, bool hasSide, bool hasTop, bool hasBottom, bool outline);
    static StaticMesh *createUnitElevationGrid(int dimensionX, int dimensionY, const float *heightData, float thickness,
                                               bool outline);
    static StaticMesh *createUnitRectangle(bool outline);
    static StaticMesh *createUnitIcosphere(int subdivision, bool outline);
    static StaticMesh *createUnitUVSphere(int subdivision, bool outline);
    static StaticMesh *createCapsule(int subdivision, float radius, float height, bool hasSide, bool hasTop, bool hasBottom,
                                     bool outline);
    static StaticMesh *createQuad();
    static StaticMesh *createLineSet(int coordCount, const float *coordData, const float *colorData);
    static StaticMesh *createPointSet(int coordCount, const float *coordData, const float *colorData);
    static StaticMesh *createTriangleMesh(int coordCount, int indexCount, const float *coordData, const float *normalData,
                                          const float *texCoordData, const float *unwrappedTexCoordData,
                                          const unsigned int *indexData, bool outline);

    static size_t cachedItemCount();
    static void printCacheContents();

    void setCachePersistency(bool persistent);

    bool isCachePersistent() const { return mIsCachePersistent; }

    const std::vector<glm::vec4> &shadowCoords() const override { return mCacheData->mShadowCoords; }
    const std::vector<Mesh::Edge> &edges() const override { return mCacheData->mEdges; }
    const std::vector<Mesh::Triangle> &triangles() const override { return mCacheData->mTriangles; }
    const Mesh::Triangle &triangle(size_t index) const override { return mCacheData->mTriangles[index]; }

    void readData(float *coordData, float *normalData, float *texCoordData, unsigned int *indexData);
    int vertexCount() const;
    int indexCount() const;

    void bind() override;
    void release() override;
    void render(unsigned int drawingMode) override;

    void bindShadowVolume() override;
    void releaseShadowVolume() override;

    size_t sortingId() const override;

    primitive::Aabb recomputeAabb(const glm::vec3 &scale = gVec3Ones) override;
    primitive::Sphere recomputeBoundingSphere(const glm::vec3 &scale = gVec3Ones) override;
    void computeBoundingVolumes();

    bool supportShadows() const override { return mCacheData->mSupportShadows; }

  protected:
    StaticMesh();
    virtual ~StaticMesh() {}

  private:
    void computeTrianglesAndEdges();

    void prepareGl() override;
    void cleanupGl() override;

    static std::unordered_map<cache::Key, cache::MeshData> cCache;

    bool mIsCachePersistent;

    cache::Key mCacheKey;
    cache::MeshData *mCacheData;
  };

}  // namespace wren

#endif  // STATIC_MESH_HPP
