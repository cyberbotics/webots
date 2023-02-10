// Copyright 1996-2023 Cyberbotics Ltd.
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
#include "WbNodeUtilities.hpp"
#include "WbNormal.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSFBool.hpp"
#include "WbSFDouble.hpp"
#include "WbSFNode.hpp"
#include "WbTextureCoordinate.hpp"
#include "WbTriangleMesh.hpp"
#include "WbVrmlNodeUtilities.hpp"

void WbIndexedFaceSet::init() {
  mCoord = findSFNode("coord");
  mNormal = findSFNode("normal");
  mTexCoord = findSFNode("texCoord");
  mCcw = findSFBool("ccw");
  mNormalPerVertex = findSFBool("normalPerVertex");
  mCoordIndex = findMFInt("coordIndex");
  mNormalIndex = findMFInt("normalIndex");
  mTexCoordIndex = findMFInt("texCoordIndex");
  mCreaseAngle = findSFDouble("creaseAngle");
  setCcw(mCcw->value());
}

WbIndexedFaceSet::WbIndexedFaceSet(WbTokenizer *tokenizer) : WbTriangleMeshGeometry("IndexedFaceSet", tokenizer) {
  init();
}

WbIndexedFaceSet::WbIndexedFaceSet(const WbIndexedFaceSet &other) : WbTriangleMeshGeometry(other) {
  init();
}

WbIndexedFaceSet::WbIndexedFaceSet(const WbNode &other) : WbTriangleMeshGeometry(other) {
  init();
}

WbIndexedFaceSet::~WbIndexedFaceSet() {
}

void WbIndexedFaceSet::preFinalize() {
  if (isPreFinalizedCalled())
    return;

  WbTriangleMeshGeometry::preFinalize();

  WbFieldChecker::resetDoubleIfNegative(this, mCreaseAngle, 0.0);
}

void WbIndexedFaceSet::postFinalize() {
  WbTriangleMeshGeometry::postFinalize();

  connect(mCoord, &WbSFNode::changed, this, &WbIndexedFaceSet::updateCoord);
  connect(mNormal, &WbSFNode::changed, this, &WbIndexedFaceSet::updateNormal);
  connect(mTexCoord, &WbSFNode::changed, this, &WbIndexedFaceSet::updateTexCoord);
  connect(mCcw, &WbSFBool::changed, this, &WbIndexedFaceSet::updateCcw);
  connect(mNormalPerVertex, &WbSFBool::changed, this, &WbIndexedFaceSet::updateNormalPerVertex);
  connect(mCoordIndex, &WbMFInt::changed, this, &WbIndexedFaceSet::updateCoordIndex);
  connect(mNormalIndex, &WbMFInt::changed, this, &WbIndexedFaceSet::updateNormalIndex);
  connect(mTexCoordIndex, &WbMFInt::changed, this, &WbIndexedFaceSet::updateTexCoordIndex);
  connect(mCreaseAngle, &WbSFDouble::changed, this, &WbIndexedFaceSet::updateCreaseAngle);

  if (coord())
    connect(coord(), &WbCoordinate::fieldChanged, this, &WbIndexedFaceSet::updateCoord, Qt::UniqueConnection);

  if (normal())
    connect(normal(), &WbNormal::fieldChanged, this, &WbIndexedFaceSet::updateNormal, Qt::UniqueConnection);

  if (texCoord())
    connect(texCoord(), &WbTextureCoordinate::fieldChanged, this, &WbIndexedFaceSet::updateTexCoord, Qt::UniqueConnection);
}

void WbIndexedFaceSet::reset(const QString &id) {
  WbTriangleMeshGeometry::reset(id);

  WbNode *const coord = mCoord->value();
  if (coord)
    coord->reset(id);
  WbNode *const normal = mNormal->value();
  if (normal)
    normal->reset(id);
  WbNode *const texCoord = mTexCoord->value();
  if (texCoord)
    texCoord->reset(id);
}

void WbIndexedFaceSet::updateTriangleMesh(bool issueWarnings) {
  mTriangleMeshError = mTriangleMesh->init(
    coord() ? &(coord()->point()) : NULL, mCoordIndex, normal() ? &(normal()->vector()) : NULL, mNormalIndex,
    texCoord() ? &(texCoord()->point()) : NULL, mTexCoordIndex, mCreaseAngle->value(), mNormalPerVertex->value());

  if (issueWarnings) {
    foreach (QString warning, mTriangleMesh->warnings())
      parsingWarn(warning);

    if (!mTriangleMeshError.isEmpty())
      parsingWarn(tr("Cannot create IndexedFaceSet because: \"%1\".").arg(mTriangleMeshError));
  }
}

uint64_t WbIndexedFaceSet::computeHash() const {
  uint64_t hash = 0;

  if (coord() && coord()->pointSize()) {
    const double *startCoord = coord()->point().item(0).ptr();
    int size = 3 * coord()->pointSize();
    hash ^= WbTriangleMeshCache::sipHash13x(startCoord, size);
  }

  if (normal() && normal()->vectorSize()) {
    const double *startNormal = normal()->vector().item(0).ptr();
    int size = 2 * normal()->vectorSize();
    hash ^= WbTriangleMeshCache::sipHash13x(startNormal, size);
  }

  if (texCoord() && texCoord()->pointSize()) {
    const double *startTexCoord = texCoord()->point().item(0).ptr();
    int size = 2 * texCoord()->pointSize();
    hash ^= WbTriangleMeshCache::sipHash13x(startTexCoord, size);
  }

  if (coordIndex() && coordIndex()->size()) {
    const int *startCoordIndex = &(coordIndex()->item(0));
    hash ^= WbTriangleMeshCache::sipHash13x(startCoordIndex, coordIndex()->size());
  }

  if (normalIndex() && normalIndex()->size()) {
    const int *startNormalIndex = &(normalIndex()->item(0));
    hash ^= WbTriangleMeshCache::sipHash13x(startNormalIndex, normalIndex()->size());
  }

  if (texCoordIndex() && texCoordIndex()->size()) {
    const int *startTexCoordIndex = &(texCoordIndex()->item(0));
    hash ^= WbTriangleMeshCache::sipHash13x(startTexCoordIndex, texCoordIndex()->size());
  }

  hash ^= WbTriangleMeshCache::sipHash13x(creaseAngle()->valuePointer(), 1);
  hash ^= WbTriangleMeshCache::sipHash13x(ccw()->valuePointer(), 1);
  hash ^= WbTriangleMeshCache::sipHash13x(normalPerVertex()->valuePointer(), 1);

  return hash;
}

WbCoordinate *WbIndexedFaceSet::coord() const {
  return static_cast<WbCoordinate *>(mCoord->value());
}

WbNormal *WbIndexedFaceSet::normal() const {
  return static_cast<WbNormal *>(mNormal->value());
}

WbTextureCoordinate *WbIndexedFaceSet::texCoord() const {
  return static_cast<WbTextureCoordinate *>(mTexCoord->value());
}

void WbIndexedFaceSet::createResizeManipulator() {
  mResizeManipulator =
    new WbRegularResizeManipulator(uniqueId(), WbWrenAbstractResizeManipulator::ResizeConstraint::NO_CONSTRAINT);
}

bool WbIndexedFaceSet::areSizeFieldsVisibleAndNotRegenerator() const {
  const WbField *const coordinates = findField("coord", true);
  return WbVrmlNodeUtilities::isVisible(coordinates) && !WbNodeUtilities::isTemplateRegeneratorField(coordinates);
}

void WbIndexedFaceSet::attachResizeManipulator() {
  if (coord())
    WbTriangleMeshGeometry::attachResizeManipulator();
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

void WbIndexedFaceSet::updateNormal() {
  if (normal()) {
    connect(normal(), &WbNormal::fieldChanged, this, &WbIndexedFaceSet::updateNormal, Qt::UniqueConnection);
    for (int i = 0; i < normal()->vector().size(); ++i) {
      if (normal()->vector(i).isNull()) {
        normal()->setVector(i, WbVector3(0.0, 1.0, 0.0));
        parsingWarn(tr("Normal values can't be null."));
      }
    }
  }

  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::updateTexCoord() {
  if (texCoord())
    connect(texCoord(), &WbTextureCoordinate::fieldChanged, this, &WbIndexedFaceSet::updateTexCoord, Qt::UniqueConnection);

  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::updateCcw() {
  setCcw(mCcw->value());

  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::updateNormalPerVertex() {
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

void WbIndexedFaceSet::updateNormalIndex() {
  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::updateTexCoordIndex() {
  buildWrenMesh(true);

  emit changed();
}

void WbIndexedFaceSet::updateCreaseAngle() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mCreaseAngle, 0.0))
    return;

  buildWrenMesh(true);

  emit changed();
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

bool WbIndexedFaceSet::exportNodeHeader(WbWriter &writer) const {
  if (!writer.isX3d())
    return WbGeometry::exportNodeHeader(writer);

  // reduce the number of exported TriangleMeshGeometrys by automatically
  // using a def-use based on the mesh hash
  writer << "<" << x3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
  if (writer.indexedFaceSetDefMap().contains(mMeshKey.mHash)) {
    writer << " USE=\'" + writer.indexedFaceSetDefMap().value(mMeshKey.mHash) + "\'></" + x3dName() + ">";
    return true;
  }

  if (cTriangleMeshMap.at(mMeshKey).mNumUsers > 1)
    writer.indexedFaceSetDefMap().insert(mMeshKey.mHash, QString::number(uniqueId()));
  return false;
}

void WbIndexedFaceSet::exportNodeContents(WbWriter &writer) const {
  // before exporting the vertex, normal and texture coordinates, we
  // need to remove duplicates from the arrays to save space in the
  // saved file and adapt the indexes consequently

  // export the original loaded mesh if we're not writing to X3D
  if (!writer.isX3d()) {
    WbNode::exportNodeContents(writer);
    return;
  }

  // To avoid differences due to normal computations export the computed triangle mesh.
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
      const double x = mTriangleMesh->vertex(i, j, 0);
      const double y = mTriangleMesh->vertex(i, j, 1);
      const double z = mTriangleMesh->vertex(i, j, 2);
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
      const double nx = mTriangleMesh->normal(i, j, 0);
      const double ny = mTriangleMesh->normal(i, j, 1);
      const double nz = mTriangleMesh->normal(i, j, 2);
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

      const double tu = mTriangleMesh->textureCoordinate(i, j, 0);
      const double tv = mTriangleMesh->textureCoordinate(i, j, 1);
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
      ++indexCount;
    }
  }

  const WbField *solidField = findField("solid", true);
  if (solidField)
    solidField->write(writer);

  const WbField *ccwField = findField("ccw", true);
  if (ccwField)
    ccwField->write(writer);

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

  writer << "<TextureCoordinate point=\'";
  for (int i = 0; i < textureCount; ++i) {
    if (i != 0)
      writer << ", ";
    const int j = 2 * i;
    writer << QString::number(texture[j], 'f', precision) << " " << QString::number(1.0 - texture[j + 1], 'f', precision);
  }
  writer << "\'></TextureCoordinate>";

  delete[] coordIndex;
  delete[] normalIndex;
  delete[] texCoordIndex;
  delete[] vertex;
  delete[] normal;
  delete[] texture;
}
