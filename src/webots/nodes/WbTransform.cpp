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

#include "WbTransform.hpp"

#include "WbNodeUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSolid.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/node.h>
#include <wren/transform.h>

void WbTransform::init() {
  mPoseChangedSignalEnabled = false;
}

WbTransform::WbTransform(WbTokenizer *tokenizer) : WbGroup("Transform", tokenizer), WbAbstractTransform(this) {
  init();
}

WbTransform::WbTransform(const WbTransform &other) : WbGroup(other), WbAbstractTransform(this) {
  init();
}

WbTransform::WbTransform(const WbNode &other) : WbGroup(other), WbAbstractTransform(this) {
  init();
}

WbTransform::WbTransform(const QString &modelName, WbTokenizer *tokenizer) :
  WbGroup(modelName, tokenizer),
  WbAbstractTransform(this) {
  init();
}

WbTransform::~WbTransform() {
  disconnect(childrenField(), &WbMFNode::changed, this, &WbTransform::updateConstrainedHandleMaterials);
  if (areWrenObjectsInitialized())
    wr_node_delete(WR_NODE(wrenNode()));
}

void WbTransform::preFinalize() {
  WbGroup::preFinalize();

  WbAbstractTransform::checkScale();
}

void WbTransform::postFinalize() {
  WbGroup::postFinalize();

  connect(mTranslation, &WbSFVector3::changed, this, &WbTransform::updateTranslation);
  connect(mTranslation, &WbSFVector3::changedByUser, this, &WbTransform::translationOrRotationChangedByUser);
  if (!isInBoundingObject())
    connect(this, &WbTransform::translationOrRotationChangedByUser, this, &WbTransform::notifyJerk);
  connect(mRotation, &WbSFRotation::changed, this, &WbTransform::updateRotation);
  // The following connection may be interesting to add in the future, but it is not used yet.
  // connect(mRotation, &WbSFRotation::changedByUser, this, &WbTransform::translationOrRotationChangedByUser);
  connect(mScale, SIGNAL(changed()), this, SLOT(updateScale()));
}

void WbTransform::updateTranslation() {
  WbAbstractTransform::updateTranslation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomPosition();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void WbTransform::updateRotation() {
  WbAbstractTransform::updateRotation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomRotation();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void WbTransform::updateTranslationAndRotation() {
  WbAbstractTransform::updateTranslationAndRotation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomRotation();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void WbTransform::applyToScale() {
  WbAbstractTransform::applyToScale();

  if (isInBoundingObject() && isAValidBoundingObject())
    applyToOdeScale();
}

void WbTransform::updateScale(bool warning) {
  WbAbstractTransform::updateScale(warning);

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void WbTransform::updateConstrainedHandleMaterials() {
  WbAbstractTransform::updateConstrainedHandleMaterials();
}

void WbTransform::notifyJerk() {
  WbSolid *s = upperSolid();
  if (s)
    s->notifyChildJerk(this);
}

void WbTransform::emitTranslationOrRotationChangedByUser() {
  // supervisor = false, because this function is currently only called from the drag events.
  emit mTranslation->changedByUser(false);
  emit mRotation->changedByUser(false);
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void WbTransform::createWrenObjects() {
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
  applyScaleToWren();
}

void WbTransform::createScaleManipulator() {
  const int constraint = constraintType();
  mScaleManipulator = new WbScaleManipulator(uniqueId(), (WbScaleManipulator::ResizeConstraint)constraint);
  if (constraint) {
    connect(childrenField(), &WbMFNode::destroyed, mScaleManipulator, &WbScaleManipulator::hide);
    connect(childrenField(), &WbMFNode::changed, this, &WbTransform::updateConstrainedHandleMaterials);
  }
}

int WbTransform::constraintType() const {
  static const int CONSTRAINT = WbWrenAbstractResizeManipulator::NO_CONSTRAINT;
  const WbGeometry *const g = geometry();

  if (g && (nodeUse() & WbNode::BOUNDING_OBJECT_USE))
    return g->constraintType();
  return CONSTRAINT;
}

void WbTransform::setScaleNeedUpdate() {
  WbAbstractTransform::setScaleNeedUpdateFlag();
  WbGroup::setScaleNeedUpdate();
}

void WbTransform::setMatrixNeedUpdate() {
  WbAbstractTransform::setMatrixNeedUpdateFlag();
  WbGroup::setMatrixNeedUpdate();
}

// WREN Material Methods

// for WbTransform lying into a bounding object only
void WbTransform::updateCollisionMaterial(bool isColliding, bool onSelection) {
  WbGeometry *const g = geometry();
  if (g)
    g->updateCollisionMaterial(isColliding, onSelection);
}

void WbTransform::setSleepMaterial() {
  WbGeometry *const g = geometry();
  if (g)
    g->setSleepMaterial();
}

///////////////////////////////////////////////////////////////
//  ODE related methods for WbTransforms in boundingObjects  //
///////////////////////////////////////////////////////////////

void WbTransform::listenToChildrenField() {
  connect(childrenField(), &WbMFNode::itemInserted, this, &WbTransform::createOdeGeom, Qt::UniqueConnection);
  const WbGeometry *const g = geometry();
  if (g)
    connect(g, &WbGeometry::destroyed, this, &WbTransform::createOdeGeomIfNeeded, Qt::UniqueConnection);
  const WbShape *const s = shape();
  if (s) {
    s->connectGeometryField();
    connect(s, &WbShape::geometryInShapeInserted, this, &WbTransform::geometryInTransformInserted, Qt::UniqueConnection);
  }
}

void WbTransform::createOdeGeomIfNeeded() {
  if (isBeingDeleted())
    return;

  if (childCount() != 0) {
    createOdeGeom();  // a physically inactive WbGeometry or WbShape pop up and the corresponding ODE dGeom has to be created

    WbGeometry *const g = geometry();
    if (g && g->isPostFinalizedCalled())
      g->computeWrenRenderable();
  }
}

void WbTransform::createOdeGeom(int index) {
  if (index > 0)
    return;

  if (index == 0 && childCount() > 1)
    destroyPreviousOdeGeoms();  // a child node was inserted at the first position in a non-empty list; if the second child
                                // contained a WbGeometry descendant then it has to be destroyed

  const WbShape *const s = shape();
  if (s) {
    s->connectGeometryField();
    connect(s, &WbShape::geometryInShapeInserted, this, &WbTransform::geometryInTransformInserted, Qt::UniqueConnection);
  }

  checkScale(true, true);

  emit geometryInTransformInserted();
}

void WbTransform::destroyPreviousOdeGeoms() {
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

bool WbTransform::isSuitableForInsertionInBoundingObject(bool warning) const {
  if (childCount() == 0)
    return true;

  const WbGeometry *const g = geometry();
  return g ? g->isSuitableForInsertionInBoundingObject(warning) : true;
}

bool WbTransform::isAValidBoundingObject(bool checkOde, bool warning) const {
  assert(isInBoundingObject());
  const int cc = childCount();

  if (cc == 0) {
    if (warning)
      info(tr("A Transform placed in 'boundingObject' needs a Geometry or Shape as its first child to be valid."));
    return false;
  }

  if (cc > 1 && warning)
    warn(tr("A Transform placed inside a 'boundingObject' can only contain one child. Remaining children are ignored."));

  const WbGeometry *const g = geometry();
  if (g == NULL) {
    if (warning)
      info(
        tr("The first child of a Transform placed in 'boundingObject' must be a Geometry or a Shape filled with a Geometry."));
    return false;
  }

  if (g->isAValidBoundingObject(checkOde, warning) == false)
    return false;

  if (checkOde && g->odeGeom() == NULL)
    return false;

  return true;
}

// Updates the ODE geometry: only for WbTransform lying into a bounding object
void WbTransform::applyToOdeData(bool correctMass) {
  WbGeometry *const g = geometry();
  if (g) {
    applyToOdeGeomPosition(false);   // updates the position relative to the Solid parent
    g->applyToOdeData(correctMass);  // updates the ODE dGeom itself
  }
}

void WbTransform::applyToOdeGeomPosition(bool correctMass) {
  WbGeometry *const g = geometry();

  dGeomID geom = g->odeGeom();
  assert(geom);

  if (dGeomGetBody(geom) == NULL)
    g->setOdePosition(position());

  if (correctMass)
    applyToOdeMass(g, geom);
}

void WbTransform::applyToOdeGeomRotation() {
  WbGeometry *const g = geometry();

  dGeomID geom = g->odeGeom();
  assert(geom);

  if (dGeomGetBody(geom) == NULL)
    g->setOdeRotation(WbRotation(rotationMatrix()));

  applyToOdeMass(g, geom);
}

void WbTransform::applyToOdeMass(WbGeometry *g, dGeomID geom) {
  const WbOdeGeomData *const odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(geom));
  assert(odeGeomData);
  WbSolid *const solid = odeGeomData->solid();
  const dMass *odeMass = g->odeMass();
  if (solid->physics() && odeMass->mass > 0.0)
    solid->correctOdeMass(odeMass, this);
}

void WbTransform::applyToOdeScale() {
  geometry()->applyToOdeData();
}

WbShape *WbTransform::shape() const {
  if (childCount() == 0)
    return NULL;

  return dynamic_cast<WbShape *>(child(0));
}

///////////////////////////////////////////////////////
//  WREN methods related to WbTransform manipulators //
///////////////////////////////////////////////////////

void WbTransform::showResizeManipulator(bool enabled) {
  WbAbstractTransform::showResizeManipulator(enabled);
  emit visibleHandlesChanged(enabled);
}

////////////
// Export //
////////////

void WbTransform::exportBoundingObjectToX3D(WbVrmlWriter &writer) const {
  assert(writer.isX3d());

  writer << QString("<Transform translation='%1' rotation='%2'>")
              .arg(translation().toString(WbPrecision::DOUBLE_MAX))
              .arg(rotation().toString(WbPrecision::DOUBLE_MAX));

  WbMFNode::Iterator it(children());
  while (it.hasNext()) {
    const WbNode *const childNode = static_cast<WbNode *>(it.next());
    const WbGeometry *const childGeom = dynamic_cast<const WbGeometry *>(childNode);

    if (childGeom)
      writer << "<Shape>";

    childNode->exportBoundingObjectToX3D(writer);

    if (childGeom)
      writer << "</Shape>";
  }

  writer << "</Transform>";
}

QStringList WbTransform::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "translation"
         << "rotation";
  return fields;
}
