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

#include "WbPose.hpp"

#include "WbNodeUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbSolid.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/node.h>
#include <wren/transform.h>

void WbPose::init() {
  mPoseChangedSignalEnabled = false;

  // store position
  // note: this cannot be put into the preFinalize function because
  //       of the copy constructor last initialization
  if (nodeModelName() != "TrackWheel") {
    mSavedTranslations[stateId()] = translation();
    mSavedRotations[stateId()] = rotation();
  }
}

WbPose::WbPose(WbTokenizer *tokenizer) : WbGroup("Pose", tokenizer), WbAbstractPose(this) {
  init();
}

WbPose::WbPose(const WbPose &other) : WbGroup(other), WbAbstractPose(this) {
  init();
}

WbPose::WbPose(const WbNode &other) : WbGroup(other), WbAbstractPose(this) {
  init();
}

WbPose::WbPose(const QString &modelName, WbTokenizer *tokenizer) : WbGroup(modelName, tokenizer), WbAbstractPose(this) {
  init();
}

WbPose::~WbPose() {
  if (areWrenObjectsInitialized())
    wr_node_delete(WR_NODE(wrenNode()));
}

void WbPose::reset(const QString &id) {
  WbGroup::reset(id);
  // note: for solids, the set of these parameters has to occur only if mJointParents.size() == 0 and it is handled in
  // WbSolid::reset, otherwise it breaks the reset of hinge based joints
  if (nodeType() != WB_NODE_TRACK_WHEEL && !dynamic_cast<WbSolid *>(this)) {
    setTranslation(mSavedTranslations[id]);
    setRotation(mSavedRotations[id]);
  }
}

void WbPose::save(const QString &id) {
  WbGroup::save(id);
  if (nodeType() != WB_NODE_TRACK_WHEEL) {
    mSavedTranslations[id] = translation();
    mSavedRotations[id] = rotation();
  }
}

void WbPose::preFinalize() {
  WbGroup::preFinalize();
}

void WbPose::postFinalize() {
  WbGroup::postFinalize();

  connect(mTranslation, &WbSFVector3::changed, this, &WbPose::updateTranslation);
  connect(mTranslation, &WbSFVector3::changedByUser, this, &WbPose::translationOrRotationChangedByUser);
  if (!isInBoundingObject())
    connect(this, &WbPose::translationOrRotationChangedByUser, this, &WbPose::notifyJerk);
  connect(mRotation, &WbSFRotation::changed, this, &WbPose::updateRotation);
  connect(mRotation, &WbSFRotation::changedByUser, this, &WbPose::translationOrRotationChangedByUser);
}

void WbPose::updateTranslation() {
  WbAbstractPose::updateTranslation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomPosition();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void WbPose::updateRotation() {
  WbAbstractPose::updateRotation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomRotation();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void WbPose::updateTranslationAndRotation() {
  WbAbstractPose::updateTranslationAndRotation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomRotation();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void WbPose::notifyJerk() {
  WbSolid *s = upperSolid();
  if (s)
    s->notifyChildJerk(this);
}

void WbPose::emitTranslationOrRotationChangedByUser() {
  // supervisor = false, because this function is currently only called from the drag events.
  emit mTranslation->changedByUser(false);
  emit mRotation->changedByUser(false);
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void WbPose::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  WrTransform *transform = wr_transform_new();
  wr_transform_attach_child(wrenNode(), WR_NODE(transform));
  setWrenNode(transform);

  const int size = children().size();
  for (int i = 0; i < size; ++i) {
    WbBaseNode *const n = child(i);
    n->createWrenObjects();
  }

  applyTranslationToWren();
  applyRotationToWren();
}

void WbPose::setMatrixNeedUpdate() {
  WbAbstractPose::setMatrixNeedUpdateFlag();
  WbGroup::setMatrixNeedUpdate();
}

// WREN Material Methods

// for WbPose lying into a bounding object only
void WbPose::updateCollisionMaterial(bool isColliding, bool onSelection) {
  WbGeometry *const g = geometry();
  if (g)
    g->updateCollisionMaterial(isColliding, onSelection);
}

void WbPose::setSleepMaterial() {
  WbGeometry *const g = geometry();
  if (g)
    g->setSleepMaterial();
}

///////////////////////////////////////////////////////////////
//  ODE related methods for WbPoses in boundingObjects  //
///////////////////////////////////////////////////////////////

void WbPose::listenToChildrenField() {
  connect(childrenField(), &WbMFNode::itemChanged, this, &WbPose::createOdeGeom, Qt::UniqueConnection);
  connect(childrenField(), &WbMFNode::itemInserted, this, &WbPose::createOdeGeom, Qt::UniqueConnection);
  const WbGeometry *const g = geometry();
  if (g)
    connect(g, &WbGeometry::destroyed, this, &WbPose::createOdeGeomIfNeeded, Qt::UniqueConnection);
  const WbShape *const s = shape();
  if (s) {
    s->connectGeometryField();
    connect(s, &WbShape::geometryInShapeInserted, this, &WbPose::geometryInPoseInserted, Qt::UniqueConnection);
  }
}

void WbPose::createOdeGeomIfNeeded() {
  if (isBeingDeleted())
    return;

  if (childCount() != 0) {
    createOdeGeom();  // a physically inactive WbGeometry or WbShape pop up and the corresponding ODE dGeom has to be created

    WbGeometry *const g = geometry();
    if (g && g->isPostFinalizedCalled())
      g->computeWrenRenderable();
  }
}

void WbPose::createOdeGeom(int index) {
  if (index > 0)
    return;

  if (index == 0 && childCount() > 1)
    destroyPreviousOdeGeoms();  // a child node was inserted at the first position in a non-empty list; if the second child
                                // contained a WbGeometry descendant then it has to be destroyed

  const WbShape *const s = shape();
  if (s) {
    s->connectGeometryField();
    connect(s, &WbShape::geometryInShapeInserted, this, &WbPose::geometryInPoseInserted, Qt::UniqueConnection);
  }

  emit geometryInPoseInserted();
}

void WbPose::destroyPreviousOdeGeoms() {
  WbNode *const secondChild = child(1);
  const WbShape *const s = dynamic_cast<WbShape *>(secondChild);
  WbGeometry *g = NULL;
  if (s) {
    g = s->geometry();
    s->disconnectGeometryField();
  } else
    g = dynamic_cast<WbGeometry *>(secondChild);
  if (g && g->odeGeom()) {
    g->destroyOdeObjects();
    g->deleteWrenRenderable();
  }
}

bool WbPose::isSuitableForInsertionInBoundingObject(bool warning) const {
  if (childCount() == 0)
    return true;

  const WbGeometry *const g = geometry();
  return g ? g->isSuitableForInsertionInBoundingObject(warning) : true;
}

bool WbPose::isAValidBoundingObject(bool checkOde, bool warning) const {
  assert(isInBoundingObject());
  const int cc = childCount();

  if (cc == 0) {
    if (warning)
      parsingInfo(tr("A Transform placed in 'boundingObject' needs a Geometry or Shape as its first child to be valid."));
    return false;
  }

  if (cc > 1 && warning)
    parsingWarn(tr("A Transform placed inside a 'boundingObject' can only contain one child. Remaining children are ignored."));

  const WbGeometry *const g = geometry();
  if (g == NULL) {
    if (warning)
      parsingInfo(
        tr("The first child of a Transform placed in 'boundingObject' must be a Geometry or a Shape filled with a Geometry."));
    return false;
  }

  if (g->isAValidBoundingObject(checkOde, warning) == false)
    return false;

  if (checkOde && g->odeGeom() == NULL)
    return false;

  return true;
}

// Updates the ODE geometry: only for WbPose lying into a bounding object
void WbPose::applyToOdeData(bool correctMass) {
  WbGeometry *const g = geometry();
  if (g) {
    applyToOdeGeomPosition(false);   // updates the position relative to the Solid parent
    g->applyToOdeData(correctMass);  // updates the ODE dGeom itself
  }
}

void WbPose::applyToOdeGeomPosition(bool correctMass) {
  WbGeometry *const g = geometry();

  dGeomID geom = g->odeGeom();
  assert(geom);

  if (dGeomGetBody(geom) == NULL)
    g->setOdePosition(position());

  if (correctMass)
    applyToOdeMass(g, geom);
}

void WbPose::applyToOdeGeomRotation() {
  WbGeometry *const g = geometry();

  dGeomID geom = g->odeGeom();
  assert(geom);

  if (dGeomGetBody(geom) == NULL)
    g->setOdeRotation(rotationMatrix());

  applyToOdeMass(g, geom);
}

void WbPose::applyToOdeMass(WbGeometry *g, dGeomID geom) {
  const WbOdeGeomData *const odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(geom));
  assert(odeGeomData);
  WbSolid *const solid = odeGeomData->solid();
  const dMass *odeMass = g->odeMass();
  if (solid && solid->physics() && odeMass->mass > 0.0)
    solid->correctOdeMass(odeMass, this);
}

WbShape *WbPose::shape() const {
  if (childCount() == 0)
    return NULL;

  return dynamic_cast<WbShape *>(child(0));
}

////////////
// Export //
////////////

void WbPose::exportBoundingObjectToX3D(WbWriter &writer) const {
  assert(writer.isX3d());

  if (isUseNode() && defNode())
    writer << "<" << x3dName() << " role='boundingObject' USE=\'n" + QString::number(defNode()->uniqueId()) + "\'/>";
  else {
    writer << QString("<Pose translation='%1' rotation='%2' role='boundingObject'")
                .arg(translation().toString(WbPrecision::DOUBLE_MAX))
                .arg(rotation().toString(WbPrecision::DOUBLE_MAX))
           << " id=\'n" << QString::number(uniqueId()) << "\'>";
    ;

    WbMFNode::Iterator it(children());
    while (it.hasNext()) {
      const WbNode *const childNode = static_cast<WbNode *>(it.next());
      childNode->write(writer);
    }

    writer << "</Pose>";
  }
}

QStringList WbPose::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "translation"
         << "rotation";
  return fields;
}

WbVector3 WbPose::translationFrom(const WbNode *fromNode) const {
  const WbPose *parentNode = WbNodeUtilities::findUpperPose(this);
  const WbPose *childNode = this;
  QList<const WbPose *> poseList;

  poseList.append(childNode);
  while (parentNode != fromNode) {
    childNode = parentNode;
    parentNode = WbNodeUtilities::findUpperPose(parentNode);
    poseList.append(childNode);
    assert(parentNode);
  }

  WbPose *previousPose = const_cast<WbPose *>(poseList.takeLast());
  WbVector3 translationResult = previousPose->translation();
  while (poseList.size() > 0) {
    const WbPose *pose = poseList.takeLast();
    translationResult += previousPose->rotation().toMatrix3() * pose->translation();
    previousPose = const_cast<WbPose *>(pose);
  }

  return translationResult;
}

WbMatrix3 WbPose::rotationMatrixFrom(const WbNode *fromNode) const {
  const WbPose *parentNode = WbNodeUtilities::findUpperPose(this);
  const WbPose *childNode = this;

  QList<const WbPose *> poseList;
  poseList.append(childNode);
  while (parentNode != fromNode) {
    childNode = parentNode;
    parentNode = WbNodeUtilities::findUpperPose(parentNode);
    poseList.append(childNode);
    assert(parentNode);
  }

  WbMatrix3 rotationResult = poseList.takeLast()->rotation().toMatrix3();
  while (poseList.size() > 0)
    rotationResult *= poseList.takeLast()->rotation().toMatrix3();

  return rotationResult;
}
