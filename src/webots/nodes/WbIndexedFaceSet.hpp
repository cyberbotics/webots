// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_INDEXED_FACE_SET_HPP
#define WB_INDEXED_FACE_SET_HPP

#include "WbGeometry.hpp"
#include "WbTriangleMeshCache.hpp"

#include <unordered_map>

class WbCoordinate;
class WbTextureCoordinate;
class WbTriangleMesh;
class WbVector3;

typedef struct dxTriMeshData *dTriMeshDataID;

typedef std::unordered_map<WbTriangleMeshCache::IndexedFaceSetKey, WbTriangleMeshCache::TriangleMeshInfo,
                           WbTriangleMeshCache::IndexedFaceSetKeyHasher>
  WbTriangleMeshMap;

class WbIndexedFaceSet : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbIndexedFaceSet(WbTokenizer *tokenizer = NULL);
  WbIndexedFaceSet(const WbIndexedFaceSet &other);
  explicit WbIndexedFaceSet(const WbNode &other);
  virtual ~WbIndexedFaceSet();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_INDEXED_FACE_SET; }
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void setScaleNeedUpdate() override;
  dGeomID createOdeGeom(dSpaceID space) override;
  void createResizeManipulator() override;
  void attachResizeManipulator() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void buildGeomIntoBuffers(WbWrenMeshBuffers *buffers, const WbMatrix4 &m, bool generateUserTexCoords = true) const override;
  void reset() override;

  // field accessors
  WbCoordinate *coord() const;
  WbTextureCoordinate *texCoord() const;
  const WbMFInt *coordIndex() const { return static_cast<const WbMFInt *>(mCoordIndex); }
  const WbMFInt *texCoordIndex() const { return static_cast<const WbMFInt *>(mTexCoordIndex); }
  const WbSFDouble *creaseAngle() const { return static_cast<const WbSFDouble *>(mCreaseAngle); }
  const WbSFBool *ccw() const { return static_cast<const WbSFBool *>(mCcw); }

  // Rescaling and translating
  void rescale(const WbVector3 &v) override;
  void rescaleAndTranslate(int coordinate, double scale, double translation);
  void rescaleAndTranslate(double factor, const WbVector3 &t);
  void rescaleAndTranslate(const WbVector3 &scale, const WbVector3 &translation);
  void translate(const WbVector3 &v);
  double max(int coordinate) const;
  double min(int coordinate) const;
  double range(int coordinate) const { return max(coordinate) - min(coordinate); }

  void updateTriangleMesh(bool issueWarnings = true);

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
  WbTriangleMeshCache::TriangleMeshInfo createTriangleMesh();
  virtual void setTriangleMesh(WbTriangleMesh *triangleMesh) { mTriangleMesh = triangleMesh; }
  virtual void updateOdeData();

  WbTriangleMeshMap &getTriangleMeshMap() { return cTriangleMeshMap; }
  WbTriangleMeshCache::IndexedFaceSetKey &getMeshKey() { return mMeshKey; }

signals:
  void validIndexedFaceSetInserted();

protected:
  virtual int indexSize() const { return 0; }
  bool areSizeFieldsVisibleAndNotRegenerator() const override;
  void exportNodeContents(WbVrmlWriter &writer) const override;
  bool exportNodeHeader(WbVrmlWriter &writer) const override;

private:
  WbIndexedFaceSet &operator=(const WbIndexedFaceSet &);  // non copyable
  WbNode *clone() const override { return new WbIndexedFaceSet(*this); }
  void init();

  // user accessible fields
  WbSFNode *mCoord;
  WbSFNode *mTexCoord;
  WbSFBool *mCcw;
  WbMFInt *mCoordIndex;
  WbMFInt *mTexCoordIndex;
  WbSFDouble *mCreaseAngle;

  // other variables
  WbTriangleMesh *mTriangleMesh;
  QString mTriangleMeshError;
  dTriMeshDataID mTrimeshData;

  // WREN
  void buildWrenMesh(bool updateCache);
  int estimateVertexCount(bool isOutlineMesh = false) const;
  int estimateIndexCount(bool isOutlineMesh = false) const;

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;
  void setOdeTrimeshData();
  void clearTrimeshResources();
  bool mCorrectSolidMass;
  bool mIsOdeDataApplied;

  // ray tracing
  // compute local collision point and return the distance
  enum { X, Y, Z };
  double computeLocalCollisionPoint(WbVector3 &point, int &triangleIndex, const WbRay &ray) const;
  void updateScaledVertices() const;
  mutable bool mScaledVerticesNeedUpdate;

  // Hashmap key for this instance's mesh
  WbTriangleMeshCache::IndexedFaceSetKey mMeshKey;

  // Hashmap containing triangle meshes, shared by all instances
  static WbTriangleMeshMap cTriangleMeshMap;

private slots:
  void updateCoord();
  void updateTexCoord();
  void updateCcw();
  void updateCoordIndex();
  void updateTexCoordIndex();
  void updateCreaseAngle();
};

#endif
