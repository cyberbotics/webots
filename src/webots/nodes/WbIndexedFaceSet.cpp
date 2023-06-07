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

  WbNode *const coordNode = mCoord->value();
  if (coordNode)
    coordNode->reset(id);
  WbNode *const normalNode = mNormal->value();
  if (normalNode)
    normalNode->reset(id);
  WbNode *const texCoordNode = mTexCoord->value();
  if (texCoordNode)
    texCoordNode->reset(id);
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

QStringList WbIndexedFaceSet::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "ccw"
         << "normalPerVertex"
         << "coordIndex"
         << "normalIndex"
         << "texCoordIndex"
         << "creaseAngle";
  return fields;
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
