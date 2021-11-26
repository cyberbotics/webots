// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "WbColladaShape.hpp"

#include "WbAppearance.hpp"
#include "WbBoundingSphere.hpp"
#include "WbImageTexture.hpp"
#include "WbMaterial.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbPaintTexture.hpp"
#include "WbPbrAppearance.hpp"
#include "WbPreferences.hpp"
#include "WbRay.hpp"
#include "WbRgb.hpp"
#include "WbSFBool.hpp"
#include "WbSFNode.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>

#include <ode/ode.h>

void WbColladaShape::init() {
  mAppearance = findSFNode("appearance");
  mGeometry = findSFNode("geometry");
  mCastShadows = findSFBool("castShadows");
  mIsPickable = findSFBool("isPickable");

  mWrenMaterial = NULL;
}

WbColladaShape::WbColladaShape(WbTokenizer *tokenizer) : WbBaseNode("Shape", tokenizer) {
  init();
}

WbColladaShape::WbColladaShape(const WbColladaShape &other) : WbBaseNode(other) {
  init();
}

WbColladaShape::WbColladaShape(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbColladaShape::~WbColladaShape() {
  if (mWrenMaterial)
    wr_material_delete(mWrenMaterial);
}

void WbColladaShape::downloadAssets() {
  WbBaseNode::downloadAssets();
  if (abstractAppearance())
    abstractAppearance()->downloadAssets();
  if (geometry())
    geometry()->downloadAssets();
}

void WbColladaShape::preFinalize() {
  WbBaseNode::preFinalize();

  // handle both kinds of appearance nodes
  WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(mAppearance->value());
  if (baseNode)
    baseNode->preFinalize();

  if (geometry())
    geometry()->preFinalize();

  updateAppearance();
  updateGeometry();
}

void WbColladaShape::postFinalize() {
  WbBaseNode::postFinalize();

  // handle both kinds of appearance nodes
  WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(mAppearance->value());
  if (baseNode)
    baseNode->postFinalize();

  if (geometry())
    geometry()->postFinalize();

  connect(mAppearance, &WbSFNode::changed, this, &WbColladaShape::updateAppearance);
  connect(mGeometry, &WbSFNode::changed, this, &WbColladaShape::updateGeometry);
  if (!isInBoundingObject()) {
    connect(mCastShadows, &WbSFBool::changed, this, &WbColladaShape::updateCastShadows);
    updateCastShadows();
    connect(mIsPickable, &WbSFBool::changed, this, &WbColladaShape::updateIsPickable);
    updateIsPickable();
  }
  if (geometry())
    connect(geometry(), &WbGeometry::changed, this, &WbColladaShape::updateGeometryMaterial, Qt::UniqueConnection);

  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbColladaShape::updateAppearance);
}

void WbColladaShape::reset(const QString &id) {
  WbBaseNode::reset(id);

  // handle both kinds of appearance nodes
  WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(mAppearance->value());
  if (baseNode)
    baseNode->reset(id);

  WbNode *const geometry = mGeometry->value();
  if (geometry)
    geometry->reset(id);
}

WbAppearance *WbColladaShape::appearance() const {
  return dynamic_cast<WbAppearance *>(mAppearance->value());
}

WbPbrAppearance *WbColladaShape::pbrAppearance() const {
  return dynamic_cast<WbPbrAppearance *>(mAppearance->value());
}

WbAbstractAppearance *WbColladaShape::abstractAppearance() const {
  return dynamic_cast<WbAbstractAppearance *>(mAppearance->value());
}

void WbColladaShape::propagateSelection(bool selected) {
  WbGeometry *const g = geometry();
  if (g)
    geometry()->propagateSelection(selected);
}

void WbColladaShape::updateCollisionMaterial(bool triggerChange, bool onSelection) {  // for WbColladaShapes lying into a bounding object only
  WbGeometry *const g = geometry();
  if (g)
    geometry()->updateCollisionMaterial(triggerChange, onSelection);
}

void WbColladaShape::setScaleNeedUpdate() {  // for any WbColladaShape
  WbGeometry *const g = geometry();
  if (g)
    g->setScaleNeedUpdate();
}

void WbColladaShape::setSleepMaterial() {
  WbGeometry *const g = geometry();
  if (g)
    geometry()->setSleepMaterial();
}

void WbColladaShape::updateAppearance() {
  if (appearance())
    connect(appearance(), &WbAppearance::changed, this, &WbColladaShape::updateAppearance, Qt::UniqueConnection);
  else if (pbrAppearance())
    connect(pbrAppearance(), &WbPbrAppearance::changed, this, &WbColladaShape::updateAppearance, Qt::UniqueConnection);

  if (areWrenObjectsInitialized())
    applyMaterialToGeometry();
  else if (!isInBoundingObject())
    emit wrenMaterialChanged();
}

void WbColladaShape::updateGeometry() {
  if (isInBoundingObject())
    return;

  const WbGeometry *const g = geometry();
  if (g) {
    connect(g, &WbGeometry::wrenObjectsCreated, this, &WbColladaShape::updateGeometryMaterial, Qt::UniqueConnection);
    connect(g, &WbGeometry::wrenObjectsCreated, this, &WbColladaShape::updateIsPickable, Qt::UniqueConnection);
    connect(g, &WbGeometry::changed, this, &WbColladaShape::updateGeometryMaterial, Qt::UniqueConnection);
    if (isPostFinalizedCalled()) {
      if (g->isPostFinalizedCalled())
        WbBoundingSphere::addSubBoundingSphereToParentNode(this);
      else
        connect(g, &WbBaseNode::finalizationCompleted, this, &WbColladaShape::updateBoundingSphere, Qt::UniqueConnection);
    }
  }
}

void WbColladaShape::updateBoundingSphere(WbBaseNode *subNode) {
  disconnect(subNode, &WbBaseNode::finalizationCompleted, this, &WbColladaShape::updateBoundingSphere);
  WbBoundingSphere::addSubBoundingSphereToParentNode(this);
}

void WbColladaShape::updateCastShadows() {
  assert(!isInBoundingObject());

  WbGeometry *const g = geometry();
  if (g) {
    g->computeCastShadows(mCastShadows->value());
    emit castShadowsChanged();
  }
}

void WbColladaShape::updateIsPickable() {
  assert(!isInBoundingObject());
  WbGeometry *const g = geometry();
  if (g)
    g->setPickable(mIsPickable->value());
}

void WbColladaShape::setAppearance(WbAppearance *appearance) {
  mAppearance->removeValue();
  mAppearance->setValue(appearance);
}

void WbColladaShape::setPbrAppearance(WbPbrAppearance *appearance) {
  mAppearance->removeValue();
  mAppearance->setValue(appearance);
}

void WbColladaShape::setGeometry(WbGeometry *geometry) {
  mGeometry->removeValue();
  mGeometry->setValue(geometry);
}

void WbColladaShape::updateGeometryMaterial() {
  if (areWrenObjectsInitialized())
    applyMaterialToGeometry();
}

bool WbColladaShape::isCastShadowsEnabled() const {
  return mCastShadows->value();
}

WbBoundingSphere *WbColladaShape::boundingSphere() const {
  const WbGeometry *const g = geometry();
  if (g)
    return g->boundingSphere();

  return NULL;
}

void WbColladaShape::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  // handle both kinds of appearance nodes
  WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(mAppearance->value());
  if (baseNode)
    baseNode->createWrenObjects();

  WbGeometry *const g = geometry();
  if (g)
    g->createWrenObjects();
}

void WbColladaShape::applyMaterialToGeometry() {
  WbGeometry *const g = geometry();
  if (!mWrenMaterial)
    createWrenMaterial(WR_MATERIAL_PHONG);

  if (g) {
    if (appearance()) {
      if (appearance()->areWrenObjectsInitialized()) {
        mWrenMaterial = appearance()->modifyWrenMaterial(mWrenMaterial);
        if (appearance()->material() && appearance()->material()->transparency() == 1)
          g->setTransparent(true);
        else
          g->setTransparent(false);
      } else {
        mWrenMaterial = WbAppearance::fillWrenDefaultMaterial(mWrenMaterial);
        // We need to call setTransparent for default appearance in case a previous transparent appearance was existing and
        // replaced by the default one.
        g->setTransparent(false);
      }
    } else if (pbrAppearance() && g->nodeType() != WB_NODE_POINT_SET) {
      createWrenMaterial(WR_MATERIAL_PBR);
      if (pbrAppearance()->areWrenObjectsInitialized()) {
        mWrenMaterial = pbrAppearance()->modifyWrenMaterial(mWrenMaterial);
        if (pbrAppearance()->transparency() == 1)
          g->setTransparent(true);
        else
          g->setTransparent(false);
      }
    } else {
      mWrenMaterial = WbAppearance::fillWrenDefaultMaterial(mWrenMaterial);
      // We need to call setTransparent for default appearance in case a previous transparent appearance was existing and
      // replaced by the default one.
      g->setTransparent(false);
    }

    if (!g->isInBoundingObject())
      g->setWrenMaterial(mWrenMaterial, mCastShadows->value());

    emit wrenMaterialChanged();
  }
}

void WbColladaShape::createWrenMaterial(int type) {
  const float defaultColor[] = {1.0f, 1.0f, 1.0f};
  if (mWrenMaterial)
    wr_material_delete(mWrenMaterial);

  if (type == WR_MATERIAL_PHONG) {
    mWrenMaterial = wr_phong_material_new();
    wr_phong_material_set_color(mWrenMaterial, defaultColor);
    wr_material_set_default_program(mWrenMaterial, WbWrenShaders::defaultShader());
  } else {
    mWrenMaterial = wr_pbr_material_new();
    wr_pbr_material_set_base_color(mWrenMaterial, defaultColor);
    wr_material_set_default_program(mWrenMaterial, WbWrenShaders::pbrShader());
  }
}

QList<const WbBaseNode *> WbColladaShape::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const WbBaseNode *> list;
  const WbGeometry *const g = geometry();
  if (g)
    list << g;
  return list;
}

///////////////////////////////////////////////////////////
//  ODE related methods for WbColladaShapes in boundingObjects  //
///////////////////////////////////////////////////////////

void WbColladaShape::connectGeometryField() const {
  connect(mGeometry, &WbSFNode::changed, this, &WbColladaShape::createOdeGeom, Qt::UniqueConnection);
}

void WbColladaShape::disconnectGeometryField() const {
  disconnect(mGeometry, &WbSFNode::changed, this, &WbColladaShape::createOdeGeom);
}

void WbColladaShape::createOdeGeom() {
  const WbGeometry *const g = geometry();

  if (g == NULL) {
    parsingInfo(tr("Please specify 'geometry' field (of Shape placed in 'boundingObject')."));
    return;
  }

  emit geometryInShapeInserted();
}

bool WbColladaShape::isSuitableForInsertionInBoundingObject(bool warning) const {
  const WbGeometry *const g = geometry();
  return g ? g->isSuitableForInsertionInBoundingObject(warning) : true;
}

bool WbColladaShape::isAValidBoundingObject(bool checkOde, bool warning) const {
  const WbGeometry *const g = geometry();
  return g && g->isAValidBoundingObject(checkOde, warning);
}

////////////
// Export //
////////////

bool WbColladaShape::exportNodeHeader(WbVrmlWriter &writer) const {
  if (writer.isX3d()) {
    writer << "<" << x3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
    if (isInvisibleNode())
      writer << " render=\'false\'";

    if (isUseNode() && defNode()) {
      writer << " USE=\'" + QString::number(defNode()->uniqueId()) + "\'></" + x3dName() + ">";
      return true;
    }

    if (!mIsPickable->value())
      writer << " isPickable='false'";
    if (mCastShadows->value())
      writer << " castShadows='true'";

    return false;
  } else
    return WbBaseNode::exportNodeHeader(writer);
}

void WbColladaShape::exportBoundingObjectToX3D(WbVrmlWriter &writer) const {
  assert(writer.isX3d());

  writer << "<Shape>";

  geometry()->exportBoundingObjectToX3D(writer);

  writer << "</Shape>";
}
