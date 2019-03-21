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

#include "WbIndexedFaceSet.hpp"

#include "WbBoundingSphere.hpp"
#include "WbCoordinate.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbMFInt.hpp"
#include "WbMatter.hpp"
#include "WbNodeUtilities.hpp"
#include "WbRay.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSFBool.hpp"
#include "WbSFDouble.hpp"
#include "WbSFNode.hpp"
#include "WbSimulationState.hpp"
#include "WbTextureCoordinate.hpp"
#include "WbTransform.hpp"
#include "WbTriangleMesh.hpp"
#include "WbWrenMeshBuffers.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <ode/ode.h>

WbTriangleMeshMap WbIndexedFaceSet::cTriangleMeshMap;

void WbIndexedFaceSet::init() {
  mTrimeshData = NULL;
  mTriangleMesh = NULL;
  mScaledVerticesNeedUpdate = true;
  mCorrectSolidMass = true;
  mIsOdeDataApplied = false;

  mCoord = findSFNode("coord");
  mTexCoord = findSFNode("texCoord");
  mCcw = findSFBool("ccw");
  mCoordIndex = findMFInt("coordIndex");
  mTexCoordIndex = findMFInt("texCoordIndex");
  mCreaseAngle = findSFDouble("creaseAngle");
}

WbIndexedFaceSet::WbIndexedFaceSet(WbTokenizer *tokenizer) : WbGeometry("IndexedFaceSet", tokenizer) {
  init();
}

WbIndexedFaceSet::WbIndexedFaceSet(const WbIndexedFaceSet &other) : WbGeometry(other) {
  init();
}

WbIndexedFaceSet::WbIndexedFaceSet(const WbNode &other) : WbGeometry(other) {
  init();
}

WbIndexedFaceSet::~WbIndexedFaceSet() {
  wr_static_mesh_delete(mWrenMesh);

  if (mTriangleMesh) {
    WbTriangleMeshCache::releaseTriangleMesh(this);
    clearTrimeshResources();
  }
}

void WbIndexedFaceSet::preFinalize() {
  if (isPreFinalizedCalled())
    return;

  WbGeometry::preFinalize();

  WbFieldChecker::checkDoubleIsNonNegative(this, mCreaseAngle, 0.0);

  mMeshKey.set(this);
  WbTriangleMeshCache::useTriangleMesh(this);
}

void WbIndexedFaceSet::postFinalize() {
  WbGeometry::postFinalize();

  connect(mCoord, &WbSFNode::changed, this, &WbIndexedFaceSet::updateCoord);
  connect(mTexCoord, &WbSFNode::changed, this, &WbIndexedFaceSet::updateTexCoord);
  connect(mCcw, &WbSFBool::changed, this, &WbIndexedFaceSet::updateCcw);
  connect(mCoordIndex, &WbMFInt::changed, this, &WbIndexedFaceSet::updateCoordIndex);
  connect(mTexCoordIndex, &WbMFInt::changed, this, &WbIndexedFaceSet::updateTexCoordIndex);
  connect(mCreaseAngle, &WbSFDouble::changed, this, &WbIndexedFaceSet::updateCreaseAngle);

  if (coord())
    connect(coord(), &WbCoordinate::fieldChanged, this, &WbIndexedFaceSet::updateCoord, Qt::UniqueConnection);

  if (texCoord())
    connect(texCoord(), &WbTextureCoordinate::fieldChanged, this, &WbIndexedFaceSet::updateTexCoord, Qt::UniqueConnection);
}

void WbIndexedFaceSet::reset() {
  WbGeometry::reset();

  WbNode *const coord = mCoord->value();
  if (coord)
    coord->reset();
  WbNode *const texCoord = mTexCoord->value();
  if (texCoord)
    texCoord->reset();
}

WbTriangleMeshCache::TriangleMeshInfo WbIndexedFaceSet::createTriangleMesh() {
  delete mTriangleMesh;
  mTriangleMesh = new WbTriangleMesh();
  updateTriangleMesh(false);

  return WbTriangleMeshCache::TriangleMeshInfo(mTriangleMesh);
}

void WbIndexedFaceSet::updateTriangleMesh(bool issueWarnings) {
  mTriangleMeshError =
    mTriangleMesh->init(coord() ? &(coord()->point()) : NULL, mCoordIndex, texCoord() ? &(texCoord()->point()) : NULL,
                        mTexCoordIndex, mCreaseAngle->value(), mCcw->value());

  if (issueWarnings) {
    foreach (QString warning, mTriangleMesh->warnings())
      warn(warning);

    if (!mTriangleMeshError.isEmpty())
      warn(tr("Cannot create IndexedFaceSet because: \"%1\".").arg(mTriangleMeshError));
  }
}

void WbIndexedFaceSet::clearTrimeshResources() {
  if (mTrimeshData) {
    dGeomTriMeshDataDestroy(mTrimeshData);
    mTrimeshData = NULL;
  }
}

WbCoordinate *WbIndexedFaceSet::coord() const {
  return static_cast<WbCoordinate *>(mCoord->value());
}

WbTextureCoordinate *WbIndexedFaceSet::texCoord() const {
  return static_cast<WbTextureCoordinate *>(mTexCoord->value());
}

void WbIndexedFaceSet::createWrenObjects() {
  if (WbNodeUtilities::findContainingProto(this))
    updateTriangleMesh(false);

  foreach (QString warning, mTriangleMesh->warnings())
    warn(warning);

  if (!mTriangleMeshError.isEmpty())
    warn(tr("Cannot create IndexedFaceSet because: \"%1\".").arg(mTriangleMeshError));

  WbGeometry::createWrenObjects();

  buildWrenMesh(false);

  emit wrenObjectsCreated();
}

void WbIndexedFaceSet::setResizeManipulatorDimensions() {
  WbVector3 scale(1.0f, 1.0f, 1.0f);

  WbTransform *transform = upperTransform();
  if (transform)
    scale *= transform->matrix().scale();

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

void WbIndexedFaceSet::createResizeManipulator() {
  mResizeManipulator =
    new WbRegularResizeManipulator(uniqueId(), WbWrenAbstractResizeManipulator::ResizeConstraint::NO_CONSTRAINT);
}

bool WbIndexedFaceSet::areSizeFieldsVisibleAndNotRegenerator() const {
  const WbField *const coordinates = findField("coord", true);
  return WbNodeUtilities::isVisible(coordinates) && !WbNodeUtilities::isTemplateRegeneratorField(coordinates);
}

void WbIndexedFaceSet::attachResizeManipulator() {
  if (coord())
    WbGeometry::attachResizeManipulator();
}

void WbIndexedFaceSet::buildWrenMesh(bool updateCache) {
  if (updateCache) {
    WbTriangleMeshCache::releaseTriangleMesh(this);
    mMeshKey.set(this);
    WbTriangleMeshCache::useTriangleMesh(this);
  }

  const bool resizeManipulator = mResizeManipulator && mResizeManipulator->isAttached();
  WbGeometry::deleteWrenRenderable();

  wr_static_mesh_delete(mWrenMesh);
  mWrenMesh = NULL;

  if (!mTriangleMesh->isValid())
    return;

  const bool createOutlineMesh = isInBoundingObject();

  WbGeometry::computeWrenRenderable();

  // Reattach resize manipulator
  if (mResizeManipulator) {
    mResizeManipulator->attachTo(mWrenScaleTransform);
    if (resizeManipulator)
      mResizeManipulator->show();
  }

  // Restore pickable state
  setPickable(isPickable());

  WbWrenMeshBuffers *buffers = WbGeometry::createMeshBuffers(estimateVertexCount(), estimateIndexCount());
  buildGeomIntoBuffers(buffers, WbMatrix4(), !mTriangleMesh->areTextureCoordinatesValid());

  mWrenMesh = wr_static_mesh_new(buffers->verticesCount(), buffers->indicesCount(), buffers->vertexBuffer(),
                                 buffers->normalBuffer(), buffers->texCoordBuffer(), buffers->unwrappedTexCoordBuffer(),
                                 buffers->indexBuffer(), createOutlineMesh);

  delete buffers;

  wr_renderable_set_mesh(mWrenRenderable, WR_MESH(mWrenMesh));
}

void WbIndexedFaceSet::updateCoord() {
  if (coord())
    connect(coord(), &WbCoordinate::fieldChanged, this, &WbIndexedFaceSet::updateCoord, Qt::UniqueConnection);

  buildWrenMesh(true);

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();  // Must be called after updateTriangleMesh()

  emit changed();
}

void WbIndexedFaceSet::updateTexCoord() {
  if (texCoord())
    connect(texCoord(), &WbTextureCoordinate::fieldChanged, this, &WbIndexedFaceSet::updateTexCoord, Qt::UniqueConnection);

  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::updateCcw() {
  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::updateCoordIndex() {
  buildWrenMesh(true);

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();  // Must be called after updateTriangleMesh()

  emit changed();
}

void WbIndexedFaceSet::updateTexCoordIndex() {
  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::updateCreaseAngle() {
  if (WbFieldChecker::checkDoubleIsNonNegative(this, mCreaseAngle, 0.0))
    return;

  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::buildGeomIntoBuffers(WbWrenMeshBuffers *buffers, const WbMatrix4 &m, bool generateUserTexCoords) const {
  assert(mTriangleMesh->isValid());
  const WbMatrix3 rm = m.extracted3x3Matrix();
  const int n = mTriangleMesh->numberOfTriangles();

  int start = buffers->vertexIndex() / 3;
  float *vBuf = buffers->vertexBuffer();
  if (vBuf) {
    int i = buffers->vertexIndex();
    for (int t = 0; t < n; ++t) {    // foreach triangle
      for (int v = 0; v < 3; ++v) {  // foreach vertex
        WbWrenMeshBuffers::writeCoordinates(mTriangleMesh->vertexAt(t, v, 0), mTriangleMesh->vertexAt(t, v, 1),
                                            mTriangleMesh->vertexAt(t, v, 2), m, vBuf, i);
        i += 3;
      }
    }
  }
  float *nBuf = buffers->normalBuffer();
  if (nBuf) {
    int i = buffers->vertexIndex();
    for (int t = 0; t < n; ++t) {    // foreach triangle
      for (int v = 0; v < 3; ++v) {  // foreach vertex
        WbWrenMeshBuffers::writeNormal(mTriangleMesh->normalAt(t, v, 0), mTriangleMesh->normalAt(t, v, 1),
                                       mTriangleMesh->normalAt(t, v, 2), rm, nBuf, i);
        i += 3;
      }
    }
  }
  float *tBuf = buffers->texCoordBuffer();
  float *utBuf = buffers->unwrappedTexCoordBuffer();
  if (tBuf) {
    int i = start * buffers->texCoordSetsCount() * 2;
    for (int t = 0; t < n; ++t) {    // foreach triangle
      for (int v = 0; v < 3; ++v) {  // foreach vertex
        tBuf[i] = mTriangleMesh->textureCoordinateAt(t, v, 0);
        tBuf[i + 1] = mTriangleMesh->textureCoordinateAt(t, v, 1);

        if (generateUserTexCoords) {
          utBuf[i] = mTriangleMesh->nonRecursiveTextureCoordinateAt(t, v, 0);
          utBuf[i + 1] = mTriangleMesh->nonRecursiveTextureCoordinateAt(t, v, 1);
        } else {
          utBuf[i] = mTriangleMesh->textureCoordinateAt(t, v, 0);
          utBuf[i + 1] = mTriangleMesh->textureCoordinateAt(t, v, 1);
        }
        i += 2;
      }
    }
  }
  unsigned int *iBuf = buffers->indexBuffer();
  if (iBuf) {
    int start = buffers->vertexIndex() / 3;
    int i = buffers->index();
    for (int t = 0; t < n; ++t) {  // foreach triangle
      for (int v = 0; v < 3; ++v)  // foreach vertex
        iBuf[i++] = start + mTriangleMesh->indexAt(t, v);
    }
    buffers->setIndex(i);
  }
  buffers->setVertexIndex(buffers->vertexIndex() + estimateVertexCount() * 3);
}

int WbIndexedFaceSet::estimateVertexCount(bool isOutlineMesh) const {
  assert(mTriangleMesh->isValid());
  return 3 * mTriangleMesh->numberOfTriangles();
}

int WbIndexedFaceSet::estimateIndexCount(bool isOutlineMesh) const {
  assert(mTriangleMesh->isValid());
  return 3 * mTriangleMesh->numberOfTriangles();
}

/////////////////
// ODE objects //
/////////////////

// works only for IndexedFaceSet made up of triangles
dGeomID WbIndexedFaceSet::createOdeGeom(dSpaceID space) {
  if (!isPreFinalizedCalled())  // needed because preFinalize comes after insertion and insertion triggers ODE dGeom creation
    preFinalize();

  if (!mTriangleMesh->isValid()) {
    clearTrimeshResources();  // delete trimesh resources if any
    return NULL;
  }

  if (WbNodeUtilities::findUpperMatter(this)->nodeType() == WB_NODE_FLUID)
    checkFluidBoundingObjectOrientation();

  setOdeTrimeshData();

  dGeomID g = dCreateTriMesh(space, mTrimeshData, NULL, NULL, NULL);
  assert(g);
  return g;
}

void WbIndexedFaceSet::setOdeTrimeshData() {
  assert(mTriangleMesh->isValid());
  clearTrimeshResources();  // delete trimesh resources if any
  const int n = coord()->pointSize();
  const int nt = mTriangleMesh->numberOfTriangles();
  mTrimeshData = dGeomTriMeshDataCreate();
  mScaledVerticesNeedUpdate = true;
  updateScaledVertices();

  dGeomTriMeshDataBuildDouble(mTrimeshData, mTriangleMesh->scaledVerticesData(), 3 * sizeof(double), n,
                              mTriangleMesh->indicesData(), 3 * nt, 3 * sizeof(int));
}

// works only for IndexedFaceSet made up of triangles
void WbIndexedFaceSet::applyToOdeData(bool correctSolidMass) {
  mCorrectSolidMass = correctSolidMass;

  if (mTriangleMesh->isValid() == false)
    return;

  setOdeTrimeshData();
  if (mOdeGeom == NULL) {
    if (areOdeObjectsCreated())
      emit validIndexedFaceSetInserted();

    return;
  }

  assert(mTriangleMesh->isValid());
  assert(dGeomGetClass(mOdeGeom) == dTriMeshClass);

  dGeomTriMeshSetData(mOdeGeom, mTrimeshData);

  if (mCorrectSolidMass)
    applyToOdeMass();

  mIsOdeDataApplied = true;
}

void WbIndexedFaceSet::updateOdeData() {
  if (mIsOdeDataApplied)
    applyToOdeData(mCorrectSolidMass);
}

bool WbIndexedFaceSet::isSuitableForInsertionInBoundingObject(bool warning) const {
  return true;
}

bool WbIndexedFaceSet::isAValidBoundingObject(bool checkOde, bool warning) const {
  return mTriangleMesh->isValid() && WbGeometry::isAValidBoundingObject(checkOde, warning);
}
/////////////////
// Ray tracing //
/////////////////

WbVector2 WbIndexedFaceSet::nonRecursiveTextureSizeFactor() const {
  if (mTriangleMesh->areTextureCoordinatesValid())
    // user-specified mapping
    return WbVector2(1, 1);

  // default
  return WbVector2(4, 2);
}

bool WbIndexedFaceSet::pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet) const {
  WbVector3 localCollisionPoint;
  int t;
  bool collisionExists = computeLocalCollisionPoint(localCollisionPoint, t, ray);
  if (!collisionExists)
    return false;

  WbVector3 v0(mTriangleMesh->vertexAt(t, 0, 0), mTriangleMesh->vertexAt(t, 0, 1), mTriangleMesh->vertexAt(t, 0, 2));
  WbVector3 v1(mTriangleMesh->vertexAt(t, 1, 0), mTriangleMesh->vertexAt(t, 1, 1), mTriangleMesh->vertexAt(t, 1, 2));
  WbVector3 v2(mTriangleMesh->vertexAt(t, 2, 0), mTriangleMesh->vertexAt(t, 2, 1), mTriangleMesh->vertexAt(t, 2, 2));

  const WbTransform *const transform = upperTransform();
  if (transform) {
    const WbMatrix4 &m = transform->matrix();
    v0 = m * v0;
    v1 = m * v1;
    v2 = m * v2;
  }

  double u, v;
  ray.intersects(v0, v1, v2, false, u, v);
  WbVector2 tc0, tc1, tc2;
  if (textureCoordSet == 0 || mTriangleMesh->areTextureCoordinatesValid()) {
    tc0.setXy(mTriangleMesh->textureCoordinateAt(t, 0, 0), mTriangleMesh->textureCoordinateAt(t, 0, 1));
    tc1.setXy(mTriangleMesh->textureCoordinateAt(t, 1, 0), mTriangleMesh->textureCoordinateAt(t, 1, 1));
    tc2.setXy(mTriangleMesh->textureCoordinateAt(t, 2, 0), mTriangleMesh->textureCoordinateAt(t, 2, 1));
  } else {  // textureCoordSet == 1 and no user-specified mapping
    tc0.setXy(mTriangleMesh->nonRecursiveTextureCoordinateAt(t, 0, 0), mTriangleMesh->nonRecursiveTextureCoordinateAt(t, 0, 1));
    tc1.setXy(mTriangleMesh->nonRecursiveTextureCoordinateAt(t, 1, 0), mTriangleMesh->nonRecursiveTextureCoordinateAt(t, 1, 1));
    tc2.setXy(mTriangleMesh->nonRecursiveTextureCoordinateAt(t, 2, 0), mTriangleMesh->nonRecursiveTextureCoordinateAt(t, 2, 1));
  }
  uv = (1 - u - v) * tc0 + u * tc1 + v * tc2;

  return true;
}

double WbIndexedFaceSet::computeDistance(const WbRay &ray) const {
  WbVector3 localCollisionPoint;
  int triangleIndex;
  return computeLocalCollisionPoint(localCollisionPoint, triangleIndex, ray);
}

double WbIndexedFaceSet::computeLocalCollisionPoint(WbVector3 &point, int &triangleIndex, const WbRay &ray) const {
  if (!mTriangleMesh)
    return false;

  WbRay localRay(ray);
  WbTransform *transform = upperTransform();
  if (transform) {
    localRay.setDirection(ray.direction() * transform->matrix());
    WbVector3 origin = transform->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
    localRay.setOrigin(origin);
    localRay.normalize();
  }

  int nTriangles = mTriangleMesh->numberOfTriangles();
  double closestDistance = std::numeric_limits<double>::infinity();
  bool found = false;
  updateScaledVertices();
  for (int t = 0; t < nTriangles; ++t) {
    WbVector4 v0(mTriangleMesh->scaledVertexAt(t, 0, 0), mTriangleMesh->scaledVertexAt(t, 0, 1),
                 mTriangleMesh->scaledVertexAt(t, 0, 2), 1.0);
    WbVector4 v1(mTriangleMesh->scaledVertexAt(t, 1, 0), mTriangleMesh->scaledVertexAt(t, 1, 1),
                 mTriangleMesh->scaledVertexAt(t, 1, 2), 1.0);
    WbVector4 v2(mTriangleMesh->scaledVertexAt(t, 2, 0), mTriangleMesh->scaledVertexAt(t, 2, 1),
                 mTriangleMesh->scaledVertexAt(t, 2, 2), 1.0);

    double u, v;
    std::pair<bool, double> result = localRay.intersects(v0.toVector3(), v1.toVector3(), v2.toVector3(), true, u, v);
    if (result.first && result.second > 0.0 && result.second < closestDistance) {
      found = true;
      closestDistance = result.second;
      triangleIndex = t;
    }
  }

  if (found) {
    point = localRay.origin() + closestDistance * localRay.direction();
    return closestDistance;
  }

  return -1;
}

void WbIndexedFaceSet::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();
  if (mTriangleMesh->numberOfTriangles() == 0)
    return;

  // Ritter's bounding sphere approximation:
  // 1. Pick a point x from P, search a point y in P, which has the largest distance from x;
  // 2. Search a point z in P, which has the largest distance from y. set up an
  //    initial sphere B, with its centre as the midpoint of y and z, the radius as
  //    half of the distance between y and z;
  // 3. If all points in P are within sphere B, then we get a bounding sphere.
  //    Otherwise, let p be a point outside the sphere, construct a new sphere covering
  //    both point p and previous sphere. Repeat this step until all points are covered.
  // Note that steps 1. and 2. help in computing a better fitting (smaller) sphere by
  // estimating the center of the final sphere and thus reducing the bias due to the enclosed
  // vertices order.
  const int nbTriangles = mTriangleMesh->numberOfTriangles();
  WbVector3 p2(mTriangleMesh->vertexAt(0, 0, 0), mTriangleMesh->vertexAt(0, 0, 1), mTriangleMesh->vertexAt(0, 0, 2));
  WbVector3 p1;
  double maxDistance;  // squared distance
  for (int i = 0; i < 2; ++i) {
    maxDistance = 0.0;
    p1 = p2;
    for (int t = 0; t < nbTriangles; ++t) {
      for (int v = 0; v < 3; ++v) {
        const WbVector3 point(mTriangleMesh->vertexAt(t, v, 0), mTriangleMesh->vertexAt(t, v, 1),
                              mTriangleMesh->vertexAt(t, v, 2));
        const double d = p1.distance2(point);
        if (d > maxDistance) {
          maxDistance = d;
          p2 = point;
        }
      }
    }
  }
  mBoundingSphere->set((p2 + p1) * 0.5, sqrt(maxDistance) * 0.5);

  for (int t = 0; t < nbTriangles; ++t) {
    for (int v = 0; v < 3; ++v) {
      const WbVector3 point(mTriangleMesh->vertexAt(t, v, 0), mTriangleMesh->vertexAt(t, v, 1),
                            mTriangleMesh->vertexAt(t, v, 2));
      mBoundingSphere->enclose(point);
    }
  }
}

void WbIndexedFaceSet::updateScaledVertices() const {
  if (mScaledVerticesNeedUpdate) {
    const WbVector3 &s = absoluteScale();
    mTriangleMesh->updateScaledVertices(s.x(), s.y(), s.z());
    mScaledVerticesNeedUpdate = false;
    return;
  }
}

void WbIndexedFaceSet::setScaleNeedUpdate() {
  mScaledVerticesNeedUpdate = true;
}

/////////////////////////////////////////////////////////////
//  WREN related methods for resizing by pulling handles   //
/////////////////////////////////////////////////////////////
void WbIndexedFaceSet::rescale(const WbVector3 &v) {
  coord()->rescale(v);
}

void WbIndexedFaceSet::rescaleAndTranslate(int coordinate, double scale, double translation) {
  coord()->rescaleAndTranslate(coordinate, scale, translation);
}

void WbIndexedFaceSet::rescaleAndTranslate(double factor, const WbVector3 &t) {
  coord()->rescaleAndTranslate(WbVector3(factor, factor, factor), t);
}

void WbIndexedFaceSet::rescaleAndTranslate(const WbVector3 &scale, const WbVector3 &translation) {
  coord()->rescaleAndTranslate(scale, translation);
}

void WbIndexedFaceSet::translate(const WbVector3 &v) {
  coord()->translate(v);
}

double WbIndexedFaceSet::max(int coordinate) const {
  return mTriangleMesh->max(coordinate);
}

double WbIndexedFaceSet::min(int coordinate) const {
  return mTriangleMesh->min(coordinate);
}

bool WbIndexedFaceSet::exportNodeHeader(WbVrmlWriter &writer) const {
  if (!writer.isX3d())
    return WbGeometry::exportNodeHeader(writer);

  // reduce the number of exported IndexedFaceSets by automatically
  // using a def-use based on the mesh hash
  writer << "<" << vrmlName() << " id=\'n" << QString::number(uniqueId()) << "\'";
  if (writer.indexedFaceSetDefMap().contains(mMeshKey.mHash)) {
    writer << " USE=\'" + writer.indexedFaceSetDefMap().value(mMeshKey.mHash) + "\'></" + vrmlName() + ">";
    return true;
  }

  if (cTriangleMeshMap.at(mMeshKey).mNumUsers > 1) {
    writer << " DEF=\'" + QString::number(mMeshKey.mHash) + "\'";
    writer.indexedFaceSetDefMap().insert(mMeshKey.mHash, QString::number(mMeshKey.mHash));
  } else if (!defName().isEmpty())
    writer << " DEF=\'" << defName() << "\'";
  return false;
}

void WbIndexedFaceSet::exportNodeContents(WbVrmlWriter &writer) const {
  // before exporting the vertex, normal and texture coordinates, we
  // need to remove duplicates from the arrays to save space in the
  // saved file and adapt the indexes consequently

  // export the original loaded mesh if we're not writing to x3DOM
  if (!writer.isX3d()) {
    WbNode::exportNodeContents(writer);
    return;
  }

  // x3DOM doesn't support creaseAngles other than 0 or 3.14 (woo) so export
  // our pre-creased mesh with calculated normals
  const int n = mTriangleMesh->numberOfTriangles();
  const int n3 = n * 3;
  int *const coordIndex = new int[n3];
  int *const normalIndex = new int[n3];
  int *const texCoordIndex = new int[n3];
  double *const vertex = new double[n * 9];
  double *const normal = new double[n * 9];
  double *const texture = new double[n * 6];
  int indexCount = 0;
  int vertexCount = 0;
  int normalCount = 0;
  int textureCount = 0;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double x = mTriangleMesh->vertexAt(i, j, 0);
      const double y = mTriangleMesh->vertexAt(i, j, 1);
      const double z = mTriangleMesh->vertexAt(i, j, 2);
      bool found = false;
      for (int l = 0; l < vertexCount; ++l) {
        const int k = 3 * l;
        if (vertex[k] == x && vertex[k + 1] == y && vertex[k + 2] == z) {
          coordIndex[indexCount] = l;
          found = true;
          break;
        }
      }
      if (!found) {
        const int v = 3 * vertexCount;
        vertex[v] = x;
        vertex[v + 1] = y;
        vertex[v + 2] = z;
        coordIndex[indexCount] = vertexCount;
        ++vertexCount;
      }
      const double nx = mTriangleMesh->normalAt(i, j, 0);
      const double ny = mTriangleMesh->normalAt(i, j, 1);
      const double nz = mTriangleMesh->normalAt(i, j, 2);
      found = false;
      for (int l = 0; l < normalCount; ++l) {
        const int k = 3 * l;
        if (normal[k] == nx && normal[k + 1] == ny && normal[k + 2] == nz) {
          normalIndex[indexCount] = l;
          found = true;
          break;
        }
      }
      if (!found) {
        const int v = 3 * normalCount;
        normal[v] = nx;
        normal[v + 1] = ny;
        normal[v + 2] = nz;
        normalIndex[indexCount] = normalCount;
        ++normalCount;
      }
      if (mTexCoord->value() != NULL) {
        const double tu = mTriangleMesh->textureCoordinateAt(i, j, 0);
        const double tv = mTriangleMesh->textureCoordinateAt(i, j, 1);
        found = false;
        for (int l = 0; l < textureCount; ++l) {
          const int k = 2 * l;
          if (texture[k] == tu && texture[k + 1] == tv) {
            texCoordIndex[indexCount] = l;
            found = true;
            break;
          }
        }
        if (!found) {
          const int v = 2 * textureCount;
          texture[v] = tu;
          texture[v + 1] = tv;
          texCoordIndex[indexCount] = textureCount;
          ++textureCount;
        }
      }
      ++indexCount;
    }
  }

  findField("solid", true)->write(writer);

  writer << " coordIndex=\'";

  for (int i = 0; i < indexCount; ++i) {
    if (i != 0) {
      writer << " ";
      if (i % 3 == 0)
        writer << "-1 ";
    }
    writer << coordIndex[i];
  }

  writer << " -1\'";
  writer << " normalIndex=\'";
  for (int i = 0; i < indexCount; ++i) {
    if (i != 0) {
      writer << " ";
      if (i % 3 == 0)
        writer << "-1 ";
    }
    writer << normalIndex[i];
  }
  writer << " -1\'";

  if (mTexCoord->value() != NULL) {
    writer << " texCoordIndex=\'";

    for (int i = 0; i < indexCount; ++i) {
      if (i != 0) {
        writer << " ";
        if (i % 3 == 0)
          writer << "-1 ";
      }
      writer << texCoordIndex[i];
    }
    writer << " -1\'";
  }

  writer << ">";  // end of fields, beginning of nodes

  writer << "<Coordinate point=\'";
  const int precision = 4;
  for (int i = 0; i < vertexCount; ++i) {
    if (i != 0)
      writer << ", ";
    const int j = 3 * i;
    writer << QString::number(vertex[j], 'f', precision)
           << " "  // write with limited precision to reduce the size of the X3D/HTML file
           << QString::number(vertex[j + 1], 'f', precision) << " " << QString::number(vertex[j + 2], 'f', precision);
  }

  writer << "\'></Coordinate>";

  writer << "<Normal vector=\'";
  for (int i = 0; i < normalCount; ++i) {
    if (i != 0)
      writer << ", ";
    const int j = 3 * i;
    writer << QString::number(normal[j], 'f', precision) << " " << QString::number(normal[j + 1], 'f', precision) << " "
           << QString::number(normal[j + 2], 'f', precision);
  }
  writer << "\'></Normal>";

  if (mTexCoord->value() != NULL) {
    writer << "<TextureCoordinate point=\'";
    for (int i = 0; i < textureCount; ++i) {
      if (i != 0)
        writer << ", ";
      const int j = 2 * i;
      writer << QString::number(texture[j], 'f', precision) << " " << QString::number(1.0 - texture[j + 1], 'f', precision);
    }
    writer << "\'></TextureCoordinate>";
  }

  delete[] coordIndex;
  delete[] normalIndex;
  delete[] texCoordIndex;
  delete[] vertex;
  delete[] normal;
  delete[] texture;
}

////////////////////////
// Friction Direction //
////////////////////////

WbVector3 WbIndexedFaceSet::computeFrictionDirection(const WbVector3 &normal) const {
  warn(tr("A IndexedFaceSet is used in a Bounding object using an asymmetric friction. IndexedFaceSet does not support "
          "asymmetric friction"));
  return WbVector3(0, 0, 0);
}
