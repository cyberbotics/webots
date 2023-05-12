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

//
//  WbMatter.cpp
//

#include "WbMatter.hpp"

#include "WbElevationGrid.hpp"
#include "WbField.hpp"
#include "WbGeometry.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbMFNode.hpp"
#include "WbMFVector3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMatrix3.hpp"
#include "WbMatrix4.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbPlane.hpp"
#include "WbResizeManipulator.hpp"
#include "WbRotation.hpp"
#include "WbSelection.hpp"
#include "WbSimulationState.hpp"
#include "WbSolidUtilities.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbVector4.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <ode/ode.h>

#include <wren/config.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <QtCore/QStringList>

bool WbMatter::cShowMatterCenter = false;

void WbMatter::init() {
  // Flags
  mBoundingObjectHasChanged = false;
  mSelected = false;
  mNeedToHandleJerk = false;

  mMatterCenterTransform = NULL;
  mMatterCenterRenderable = NULL;
  mMatterCenterMaterial = NULL;
  mMatterCenterMesh = NULL;

  // user fields
  mName = findSFString("name");
  mModel = findSFString("model");
  mDescription = findSFString("description");
  mBoundingObject = findSFNode("boundingObject");
  mLocked = findSFBool("locked");
}

WbMatter::WbMatter(const WbMatter &other) : WbPose(other) {
  init();
}

WbMatter::WbMatter(const WbNode &other) : WbPose(other) {
  init();
}

WbMatter::WbMatter(const QString &modelName, WbTokenizer *tokenizer) : WbPose(modelName, tokenizer) {
  init();
}

WbMatter::~WbMatter() {
  if (areWrenObjectsInitialized()) {
    wr_node_delete(WR_NODE(mMatterCenterRenderable));
    wr_node_delete(WR_NODE(mMatterCenterTransform));
    wr_material_delete(mMatterCenterMaterial);
    wr_static_mesh_delete(mMatterCenterMesh);
  }

  disconnectFromBoundingObjectUpdates(mBoundingObject->value());
}

void WbMatter::disconnectFromBoundingObjectUpdates(const WbNode *node) const {
  if (!node)
    return;

  const WbGroup *const group = dynamic_cast<const WbGroup *>(node);
  // cppcheck-suppress knownConditionTrueFalse
  if (group) {
    for (int i = 0; i < group->childCount(); ++i)
      disconnectFromBoundingObjectUpdates(group->child(i));
    return;
  }

  const WbShape *const shape = dynamic_cast<const WbShape *>(node);
  if (shape) {
    disconnectFromBoundingObjectUpdates(shape->geometry());
    return;
  }

  const WbGeometry *const geometry = dynamic_cast<const WbGeometry *>(node);
  if (geometry)
    disconnect(geometry, &WbGeometry::boundingGeometryRemoved, this, &WbMatter::removeBoundingGeometry);
}

dSpaceID WbMatter::space() const {
  return WbOdeContext::instance()->space();
}

void WbMatter::postFinalize() {
  WbPose::postFinalize();

  if (mBoundingObject->value())
    boundingObject()->postFinalize();

  connectNameUpdates();
  // in case of follow solid option we need also to listen to parameter nodes (non instantiated) updates
  if (protoParameterNode()) {
    WbNode *parameter = protoParameterNode();
    while (parameter->protoParameterNode())
      parameter = parameter->protoParameterNode();
    WbMatter *matter = dynamic_cast<WbMatter *>(parameter);
    if (matter)
      matter->connectNameUpdates();
  }

  connect(mLocked, &WbSFBool::changed, this, &WbMatter::updateLocked);
  connect(mBoundingObject, &WbSFNode::changed, this, &WbMatter::updateBoundingObject, Qt::UniqueConnection);
  connect(mModel, &WbSFString::changed, this, &WbMatter::matterModelChanged);
  updateManipulatorVisibility();
}

void WbMatter::setBoundingObject(WbNode *boundingObject) {
  mBoundingObject->removeValue();
  mBoundingObject->setValue(boundingObject);
}

void WbMatter::reset(const QString &id) {
  WbPose::reset(id);

  WbNode *const b = mBoundingObject->value();
  if (b)
    b->reset(id);
}

void WbMatter::save(const QString &id) {
  WbPose::save(id);

  WbNode *const b = mBoundingObject->value();
  if (b)
    b->save(id);
}

void WbMatter::connectNameUpdates() const {
  connect(mName, &WbSFString::changed, this, &WbMatter::updateName, Qt::UniqueConnection);
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void WbMatter::createWrenObjects() {
  WbPose::createWrenObjects();

  if (mBoundingObject->value())
    boundingObject()->createWrenObjects();

  mMatterCenterTransform = wr_transform_new();
  mMatterCenterRenderable = wr_renderable_new();
  mMatterCenterMaterial = wr_phong_material_new();
  wr_phong_material_set_color_per_vertex(mMatterCenterMaterial, true);
  wr_material_set_default_program(mMatterCenterMaterial, WbWrenShaders::lineSetShader());

  const float coords[18] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const float colors[18] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0,  1.0f, 0.0f,
                            0.0,  1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};
  mMatterCenterMesh = wr_static_mesh_line_set_new(6, coords, colors);

  wr_renderable_set_mesh(mMatterCenterRenderable, WR_MESH(mMatterCenterMesh));
  wr_renderable_set_material(mMatterCenterRenderable, mMatterCenterMaterial, NULL);
  wr_renderable_set_drawing_mode(mMatterCenterRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_visibility_flags(mMatterCenterRenderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
  wr_renderable_set_drawing_order(mMatterCenterRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);
  wr_renderable_set_cast_shadows(mMatterCenterRenderable, false);
  wr_renderable_set_receive_shadows(mMatterCenterRenderable, false);

  wr_transform_attach_child(mMatterCenterTransform, WR_NODE(mMatterCenterRenderable));
  wr_transform_attach_child(wrenNode(), WR_NODE(mMatterCenterTransform));

  const float lineScale = wr_config_get_line_scale() * WbWrenRenderingContext::SOLID_LINE_SCALE_FACTOR;
  const float scale[3] = {lineScale, lineScale, lineScale};
  wr_transform_set_scale(mMatterCenterTransform, scale);
  wr_node_set_visible(WR_NODE(mMatterCenterTransform), isSelected());
}

////////////////////////////
//   Create ODE Objects   //
////////////////////////////

bool WbMatter::isBoundingObjectFinalizationCompleted(WbBaseNode *node) {
  if (!node)
    return false;

  if (node->isPostFinalizedCalled())
    return true;

  connect(node, &WbBaseNode::finalizationCompleted, this, &WbMatter::boundingObjectFinalizationCompleted);
  return false;
}

void WbMatter::boundingObjectFinalizationCompleted(WbBaseNode *node) {
  disconnect(node, &WbBaseNode::finalizationCompleted, this, &WbMatter::boundingObjectFinalizationCompleted);
  updateBoundingObject();
}

dSpaceID WbMatter::groupSpace() const {
  const WbGroup *const gp = dynamic_cast<WbGroup *>(mBoundingObject->value());
  return gp ? gp->odeSpace() : NULL;
}

dSpaceID WbMatter::upperSpace() const {
  assert(areOdeObjectsCreated());
  dSpaceID s = groupSpace();
  if (s)
    return s;

  return odeGeom() ? dGeomGetSpace(odeGeom()) : WbOdeContext::instance()->space();
}

dGeomID WbMatter::odeGeom() const {
  WbBaseNode *const bo = boundingObject();
  if (bo == NULL)
    return NULL;

  WbGeometry *g = NULL;

  const WbPose *const p = dynamic_cast<WbPose *>(bo);
  // cppcheck-suppress knownConditionTrueFalse
  if (p)
    g = p->geometry();
  else {
    const WbShape *const s = dynamic_cast<WbShape *>(bo);
    g = s ? s->geometry() : dynamic_cast<WbGeometry *>(bo);
  }

  if (g)
    return g->odeGeom();

  const WbGroup *const gp = dynamic_cast<WbGroup *>(bo);
  if (gp)
    return (dGeomID)gp->odeSpace();

  return NULL;
}

dGeomID WbMatter::createOdeGeomFromGeometry(dSpaceID space, WbGeometry *geometry, bool setOdeData) {
  if (geometry == NULL)
    return NULL;

  dGeomID geom = geometry->createOdeGeom(space);

  if (geom && setOdeData) {
    geometry->setOdeData(geom, this);
    connect(geometry, &WbGeometry::boundingGeometryRemoved, this, &WbMatter::removeBoundingGeometry, Qt::UniqueConnection);
  }

  return geom;
}

dGeomID WbMatter::createOdeGeomFromPose(dSpaceID space, WbPose *pose) {
  assert(space);

  // Listens to insertion/deletion in the children field of the WbPose
  connect(pose, &WbPose::geometryInPoseInserted, this, &WbMatter::createOdeGeomFromInsertedPoseItem, Qt::UniqueConnection);
  pose->listenToChildrenField();

  const int n = pose->childCount();
  if (n == 0) {
    parsingInfo(tr("A child to the Transform placed in 'boundingObject' is expected."));
    return NULL;
  }

  if (n != 1)
    pose->parsingWarn(tr("A Pose node inside a 'boundingObject' can only contain one child. Remaining children are ignored."));

  WbBaseNode *const poseChild = pose->child(0);
  const WbShape *const shape = dynamic_cast<WbShape *>(poseChild);
  if (shape) {
    const WbIndexedFaceSet *const ifs = dynamic_cast<WbIndexedFaceSet *>(shape->geometry());
    if (ifs)
      connect(ifs, &WbIndexedFaceSet::validIndexedFaceSetInserted, shape, &WbShape::geometryInShapeInserted,
              Qt::UniqueConnection);
    const WbElevationGrid *const eg = dynamic_cast<WbElevationGrid *>(shape->geometry());
    if (eg)
      connect(eg, &WbElevationGrid::validElevationGridInserted, shape, &WbShape::geometryInShapeInserted, Qt::UniqueConnection);
    connect(shape, &WbShape::geometryInShapeInserted, this, &WbMatter::createOdeGeomFromInsertedShapeItem,
            Qt::UniqueConnection);
    shape->connectGeometryField();
  } else if (dynamic_cast<WbGeometry *>(poseChild) == NULL) {
    pose->parsingWarn(tr("A Pose node inside a 'boundingObject' can only contain one Shape or one Geometry node. The child "
                         "node is ignored."));
  }

  WbGeometry *const geometry = pose->geometry();
  if (geometry == NULL)
    return NULL;

  const WbIndexedFaceSet *const ifs = dynamic_cast<WbIndexedFaceSet *>(geometry);
  // cppcheck-suppress knownConditionTrueFalse
  if (ifs)
    connect(ifs, &WbIndexedFaceSet::validIndexedFaceSetInserted, pose, &WbPose::geometryInPoseInserted, Qt::UniqueConnection);

  const WbElevationGrid *const eg = dynamic_cast<WbElevationGrid *>(geometry);
  if (eg)  // TODO: rename slot?
    connect(eg, &WbElevationGrid::validElevationGridInserted, pose, &WbPose::geometryInPoseInserted, Qt::UniqueConnection);

  dGeomID geom = createOdeGeomFromGeometry(space, geometry, false);
  if (geom == NULL)
    return NULL;

  // Stores a pointer to the ODE geometry into the WbGeometry node & sets the WbGeometry node and its WbMatter parent node as
  // reference data
  geometry->setOdeData(geom, this);
  connect(geometry, &WbGeometry::boundingGeometryRemoved, this, &WbMatter::removeBoundingGeometry, Qt::UniqueConnection);

  return geom;
}

void WbMatter::createOdeGeomFromInsertedPoseItem() {
  assert(dynamic_cast<WbPose *>(sender()));
  WbPose *const pose = static_cast<WbPose *>(sender());
  dGeomID g = createOdeGeomFromPose(upperSpace(), pose);
  if (g) {
    setGeomMatter(g, pose);
    if (isInsertedOdeGeomPositionUpdateRequired())
      updateOdeGeomPosition(g);
  }
}

void WbMatter::createOdeGeomFromInsertedShapeItem() {
  assert(dynamic_cast<WbShape *>(sender()));
  WbShape *const shape = static_cast<WbShape *>(sender());
  WbGeometry *const geometry = shape->geometry();
  WbPose *const pose = shape->upperPose();

  dGeomID insertedGeom;
  if (pose && pose->isInBoundingObject()) {
    insertedGeom = createOdeGeomFromPose(upperSpace(), pose);
    if (insertedGeom)
      setGeomMatter(insertedGeom);
  } else {  // no Pose in the boundingObject is a parent of this Shape
    WbIndexedFaceSet *const ifs = dynamic_cast<WbIndexedFaceSet *>(geometry);
    if (ifs)
      connect(ifs, &WbIndexedFaceSet::validIndexedFaceSetInserted, shape, &WbShape::geometryInShapeInserted,
              Qt::UniqueConnection);

    WbElevationGrid *const eg = dynamic_cast<WbElevationGrid *>(geometry);
    if (eg)
      connect(eg, &WbElevationGrid::validElevationGridInserted, shape, &WbShape::geometryInShapeInserted, Qt::UniqueConnection);

    insertedGeom = createOdeGeomFromGeometry(upperSpace(), geometry);
    if (insertedGeom == NULL) {
      assert(ifs || eg);
      return;
    }
    // Stores a pointer to the ODE geometry into the WbGeometry node & sets the WbGeometry node and its WbMatter parent node as
    // reference data
    geometry->setOdeData(insertedGeom, this);
    connect(geometry, &WbGeometry::boundingGeometryRemoved, this, &WbMatter::removeBoundingGeometry, Qt::UniqueConnection);
    setGeomMatter(insertedGeom, geometry);
  }

  if (isInsertedOdeGeomPositionUpdateRequired())
    updateOdeGeomPosition(insertedGeom);
}

dGeomID WbMatter::createOdeGeomFromGroup(dSpaceID space, WbGroup *group) {  // group is a *WbGroup but not a *WbPose
  // Connections for updates of items
  connect(group, &WbGroup::finalizedChildAdded, this, &WbMatter::createOdeGeomFromInsertedGroupItem, Qt::UniqueConnection);
  dSpaceID simpleSpace = dSimpleSpaceCreate(space);
  dSpaceSetCleanup(simpleSpace, 0);  // Destroying this space won't destroy the dGeoms contained in it
  group->setOdeData(simpleSpace);

  if (group->childCount() == 0) {
    parsingInfo(tr("A child in the Group placed in 'boundingObject' is missing."));
    return (dGeomID)simpleSpace;
  }

  WbMFNode::Iterator it(group->children());
  while (it.hasNext()) {
    WbNode *const node = it.next();
    WbPose *const pose = dynamic_cast<WbPose *>(node);
    if (pose)
      createOdeGeomFromPose(simpleSpace, pose);
    else {
      const WbShape *const shape = dynamic_cast<WbShape *>(node);
      if (shape) {
        const WbIndexedFaceSet *const ifs = dynamic_cast<WbIndexedFaceSet *>(shape->geometry());
        if (ifs)
          connect(ifs, &WbIndexedFaceSet::validIndexedFaceSetInserted, shape, &WbShape::geometryInShapeInserted,
                  Qt::UniqueConnection);
        const WbElevationGrid *const eg = dynamic_cast<WbElevationGrid *>(shape->geometry());
        if (eg)
          connect(eg, &WbElevationGrid::validElevationGridInserted, shape, &WbShape::geometryInShapeInserted,
                  Qt::UniqueConnection);
        connect(shape, &WbShape::geometryInShapeInserted, this, &WbMatter::createOdeGeomFromInsertedShapeItem,
                Qt::UniqueConnection);
        shape->connectGeometryField();
      }
      WbGeometry *const geometry = WbSolidUtilities::geometry(node);

      if (geometry)
        createOdeGeomFromGeometry(simpleSpace, geometry);
      // else maybe an empty WbShape: ignore
    }
  }

  return (dGeomID)simpleSpace;
}

dGeomID WbMatter::createOdeGeomFromBoundingObject(dSpaceID space) {
  return createOdeGeomFromNode(space, boundingObject());
}

void WbMatter::insertValidGeometryInBoundingObject() {
  assert(dynamic_cast<WbIndexedFaceSet *>(sender()) || dynamic_cast<WbElevationGrid *>(sender()));
  WbGeometry *const geometry = static_cast<WbGeometry *>(sender());
  createOdeGeomFromGeometry(upperSpace(), geometry);
}

dGeomID WbMatter::createOdeGeomFromNode(dSpaceID space, WbBaseNode *node) {
  if (!node)
    return NULL;

  WbPose *const pose = dynamic_cast<WbPose *>(node);
  // cppcheck-suppress knownConditionTrueFalse
  if (pose)
    return createOdeGeomFromPose(space, pose);

  WbGroup *const group = dynamic_cast<WbGroup *>(node);
  if (group)
    return createOdeGeomFromGroup(space, group);

  const WbShape *const shape = dynamic_cast<WbShape *>(node);
  if (shape) {
    const WbIndexedFaceSet *const ifs = dynamic_cast<WbIndexedFaceSet *>(shape->geometry());
    if (ifs)
      connect(ifs, &WbIndexedFaceSet::validIndexedFaceSetInserted, shape, &WbShape::geometryInShapeInserted,
              Qt::UniqueConnection);
    connect(shape, &WbShape::geometryInShapeInserted, this, &WbMatter::createOdeGeomFromInsertedShapeItem,
            Qt::UniqueConnection);
    shape->connectGeometryField();
  }

  WbGeometry *const geometry = WbSolidUtilities::geometry(node);
  if (!geometry)
    return NULL;  // the boundingObject is neither a WbGroup, WbShape nor a WbGeometry => ignored (maybe empty Shape)

  const WbIndexedFaceSet *const ifs = dynamic_cast<WbIndexedFaceSet *>(geometry);
  // cppcheck-suppress knownConditionTrueFalse
  if (ifs)
    connect(ifs, &WbIndexedFaceSet::validIndexedFaceSetInserted, this, &WbMatter::insertValidGeometryInBoundingObject,
            Qt::UniqueConnection);

  const WbElevationGrid *const eg = dynamic_cast<WbElevationGrid *>(geometry);
  if (eg)
    connect(eg, &WbElevationGrid::validElevationGridInserted, this, &WbMatter::insertValidGeometryInBoundingObject,
            Qt::UniqueConnection);

  return createOdeGeomFromGeometry(space, geometry);
}

bool WbMatter::handleJerkIfNeeded() {
  if (mNeedToHandleJerk) {
    mNeedToHandleJerk = false;
    handleJerk();
    return true;
  }
  return false;
}

/////////////////////
// Update Methods  //
/////////////////////

void WbMatter::updateTranslation() {
  WbPose::updateTranslation();

  // the translation of the WbMatter was changed through the GUI, a Supervisor or
  // automatically by Webots (if kinematic mode)
  mNeedToHandleJerk = true;
}

void WbMatter::updateRotation() {
  WbPose::updateRotation();

  // the rotation of the WbMatter was changed through the GUI, a Supervisor or
  // automatically by Webots (if kinematic mode)
  mNeedToHandleJerk = true;
}

void WbMatter::updateLineScale() {
  applyChangesToWren();
}

void WbMatter::updateName() {
  QString nameValue = mName->value();
  if (nameValue.isEmpty()) {
    const QString &defaultName = dynamic_cast<const WbSFString *>(findField("name")->defaultValue())->value();
    parsingWarn(tr("'name' cannot be empty. Default node name '%1' is automatically set.").arg(defaultName));
    mName->blockSignals(true);
    mName->setValue(defaultName);
    mName->blockSignals(false);
  }
  emit matterNameChanged();
}

// Places ODE dGeoms through their absolute coordinates
void WbMatter::updateOdePlaceableGeomPosition(dGeomID g) {
  const WbOdeGeomData *const odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(g));
  assert(odeGeomData);
  WbGeometry *geom = odeGeomData->geometry();
  assert(geom);

  const WbPose *const up = geom->upperPose();
  if (!up)
    return;

  // translates the ODE dGeom
  geom->setOdePosition(up->position());

  // const double *pos = dGeomGetPosition(g);
  // WbMathsUtilities::printVector3("geom position", pos);

  // rotates the ODE dGeom
  geom->setOdeRotation(up->rotationMatrix());
}

void WbMatter::updateOdePlanePosition(dGeomID plane) {
  const WbOdeGeomData *const odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(plane));
  assert(odeGeomData);
  WbPlane *wbplane = dynamic_cast<WbPlane *>(odeGeomData->geometry());
  assert(wbplane);
  wbplane->updateOdePlanePosition();
}

void WbMatter::updateOdeGeomPosition(dGeomID g) {
  if (g == NULL)
    return;

  const dSpaceID spaceGeom = WbSolidUtilities::dynamicCastInSpaceID(g);
  if (spaceGeom) {
    const int n = dSpaceGetNumGeoms(spaceGeom);

    // We need to store geoms in a temp array, because the list order gets
    // modified by calling setPlaceableGeom() while we iterate
    dGeomID geoms[n];
    for (int i = 0; i < n; ++i)
      geoms[i] = dSpaceGetGeom(spaceGeom, i);

    for (int i = 0; i < n; ++i) {
      if (dGeomIsSpace(geoms[i]))
        updateOdeGeomPosition(geoms[i]);
      else if (dGeomGetClass(geoms[i]) != dPlaneClass)
        updateOdePlaceableGeomPosition(geoms[i]);
      else
        updateOdePlanePosition(geoms[i]);
    }
  } else if (dGeomGetClass(g) != dPlaneClass)
    updateOdePlaceableGeomPosition(g);
  else
    updateOdePlanePosition(g);
}

void WbMatter::updateLocked() {
  if (mLocked)
    detachResizeManipulator();

  updateManipulatorVisibility();
}

////////////////////
// Apply Methods  //
////////////////////

// Apply to WREN

void WbMatter::applyChangesToWren() {
  applyMatterCenterToWren();
}

void WbMatter::applyVisibilityFlagsToWren(bool selected) {
  wr_node_set_visible(WR_NODE(mMatterCenterTransform), selected);
}

void WbMatter::applyMatterCenterToWren() {
  if (isSelected()) {
    const float lineScale = wr_config_get_line_scale() * WbWrenRenderingContext::SOLID_LINE_SCALE_FACTOR;
    const float scale[3] = {lineScale, lineScale, lineScale};
    wr_transform_set_scale(mMatterCenterTransform, scale);
  }
}

////////////
// Others //
////////////

WbBaseNode *WbMatter::boundingObject() const {
  return static_cast<WbBaseNode *>(mBoundingObject->value());
}

// Selection management
void WbMatter::select(bool selected) {
  if (mSelected == selected)
    return;

  mSelected = selected;
  propagateSelection(selected);
  if (!mSelected || cShowMatterCenter) {
    applyVisibilityFlagsToWren(selected);
    applyChangesToWren();
  }
}

// Method that checks the validity of a boundingObject
bool WbMatter::checkBoundingObject() const {
  if (boundingObject() == NULL)
    return false;

  const WbGroup *const group = dynamic_cast<WbGroup *>(boundingObject());
  if (group) {
    const WbMFNode &children = group->children();
    const int size = children.size();
    if (size == 0)
      return false;
  }

  return true;
}

// Collision and sleep flags management

void WbMatter::updateSleepFlag() {
  assert(boundingObject());
  mBoundingObjectHasChanged = false;
}

/////////////////////////////////////////////
//  Translation and Rotation manipulator   //
/////////////////////////////////////////////

void WbMatter::updateManipulatorVisibility() {
  if (mSelected) {
    if (isLocked() || WbNodeUtilities::isNodeOrAncestorLocked(this))
      detachTranslateRotateManipulator();
    else {
      updateTranslateRotateHandlesSize();
      attachTranslateRotateManipulator();
    }
  }
}
