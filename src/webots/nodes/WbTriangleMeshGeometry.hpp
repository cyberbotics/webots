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

#ifndef WB_TRIANGLE_MESH_GEOMETRY_HPP
#define WB_TRIANGLE_MESH_GEOMETRY_HPP

#include "WbGeometry.hpp"
#include "WbTriangleMeshCache.hpp"

#include <unordered_map>

class WbTriangleMesh;
class WbVector3;

typedef struct dxTriMeshData *dTriMeshDataID;

typedef std::unordered_map<WbTriangleMeshCache::TriangleMeshGeometryKey, WbTriangleMeshCache::TriangleMeshInfo,
                           WbTriangleMeshCache::TriangleMeshGeometryKeyHasher>
  WbTriangleMeshMap;

class WbTriangleMeshGeometry : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  virtual ~WbTriangleMeshGeometry();

  // reimplemented public functions
  void preFinalize() override;
  void createWrenObjects() override;
  // cppcheck-suppress virtualCallInConstructor
  void deleteWrenRenderable() override;
  void setScaleNeedUpdate() override;
  dGeomID createOdeGeom(dSpaceID space) override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void buildGeomIntoBuffers(WbWrenMeshBuffers *buffers, const WbMatrix4 &m, bool generateUserTexCoords = true) const override;

  // Rescaling and translating
  double max(int coordinate) const;
  double min(int coordinate) const;
  double range(int coordinate) const { return max(coordinate) - min(coordinate); }

  virtual void updateTriangleMesh(bool issueWarnings = true) = 0;

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const WbRay &ray) const override;

  // friction
  WbVector3 computeFrictionDirection(const WbVector3 &normal) const override;

  // Non-recursive texture mapping
  WbVector2 nonRecursiveTextureSizeFactor() const override;

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  // WbTriangleMesh management (see WbTriangleMeshCache.hpp)
  virtual uint64_t computeHash() const = 0;

  WbTriangleMeshCache::TriangleMeshInfo createTriangleMesh();
  virtual void setTriangleMesh(WbTriangleMesh *triangleMesh) { mTriangleMesh = triangleMesh; }
  virtual void updateOdeData();

  WbTriangleMeshMap &getTriangleMeshMap() { return cTriangleMeshMap; }
  WbTriangleMeshCache::TriangleMeshGeometryKey &getMeshKey() { return mMeshKey; }

signals:
  void validTriangleMeshGeometryInserted();

protected:
  // All constructors are reserved for derived classes only
  WbTriangleMeshGeometry(const QString &modelName, WbTokenizer *tokenizer);
  WbTriangleMeshGeometry(const WbTriangleMeshGeometry &other);
  WbTriangleMeshGeometry(const WbNode &other);

  virtual int indexSize() const { return 0; }

  // WREN
  void buildWrenMesh(bool updateCache);
  void setCcw(bool ccw);

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;

  // triangle mesh
  WbTriangleMesh *mTriangleMesh;
  QString mTriangleMeshError;
  dTriMeshDataID mTrimeshData;

  // Hashmap containing triangle meshes, shared by all instances
  static WbTriangleMeshMap cTriangleMeshMap;

  // Hashmap key for this instance's mesh
  WbTriangleMeshCache::TriangleMeshGeometryKey mMeshKey;

private:
  WbTriangleMeshGeometry &operator=(const WbTriangleMeshGeometry &);  // non copyable
  // Only derived classes can be cloned
  WbNode *clone() const override = 0;

  // normals representation
  WrRenderable *mNormalsRenderable;
  WrMaterial *mNormalsMaterial;
  WrStaticMesh *mNormalsMesh;

  void init();

  // WREN
  int estimateVertexCount(bool isOutlineMesh = false) const;
  int estimateIndexCount(bool isOutlineMesh = false) const;
  bool mCcw;

  // ODE
  void setOdeTrimeshData();
  void clearTrimeshResources();
  bool mCorrectSolidMass;
  bool mIsOdeDataApplied;

  // ray tracing
  // compute local collision point and return the distance
  double computeLocalCollisionPoint(WbVector3 &point, int &triangleIndex, const WbRay &ray) const;
  void updateScaledCoordinates() const;
  mutable bool mScaledCoordinatesNeedUpdate;

private slots:
  void updateOptionalRendering(int option);
  void updateNormalsRepresentation();
};

#endif
