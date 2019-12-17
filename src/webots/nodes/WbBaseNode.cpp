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

#include "WbBaseNode.hpp"
#include "WbBoundingSphere.hpp"
#include "WbDictionary.hpp"
#include "WbNodeOperations.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbTemplateManager.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"

#include <wren/scene.h>

void WbBaseNode::init() {
  mPreFinalizeCalled = false;
  mPostFinalizeCalled = false;
  mWrenObjectsCreatedCalled = false;
  mOdeObjectsCreatedCalled = false;
  mWrenNode = NULL;
  mIsInBoundingObject = false;
  mUpperTransform = NULL;
  mUpperSolid = NULL;
  mTopSolid = NULL;
  mBoundingObjectFirstTimeSearch = true;
  mUpperTransformFirstTimeSearch = true;
  mUpperSolidFirstTimeSearch = true;
  mTopSolidFirstTimeSearch = true;
  mFinalizationCanceled = false;
  mNodeUse = WbNode::UNKNOWN_USE;
  mNodeUseDirty = true;
}

WbBaseNode::WbBaseNode(const QString &modelName, WbTokenizer *tokenizer) :
  WbNode(modelName, WbWorld::instance() ? WbWorld::instance()->fileName() : "", tokenizer) {
  init();
}

WbBaseNode::WbBaseNode(const WbBaseNode &other) : WbNode(other) {
  init();
}

WbBaseNode::WbBaseNode(const WbNode &other) : WbNode(other) {
  init();
}

WbBaseNode::~WbBaseNode() {
  emit isBeingDestroyed(this);
  if (mPostFinalizeCalled && !defName().isEmpty() && !WbWorld::instance()->isCleaning() && !WbTemplateManager::isRegenerating())
    WbDictionary::instance()->removeNodeFromDictionary(this);
}

void WbBaseNode::finalize() {
  if (isProtoParameterNode()) {
    // finalize PROTO parameter node instances of the current node
    QVector<WbNode *> nodeInstances = protoParameterNodeInstances();
    WbBaseNode *baseNodeInstance = NULL;
    foreach (WbNode *nodeInstance, nodeInstances) {
      baseNodeInstance = dynamic_cast<WbBaseNode *>(nodeInstance);
      // recursive call to finalize nested parameter instances
      baseNodeInstance->finalize();
    }
    setFieldsParentNode();
    return;
  }

  WbWrenOpenGlContext::makeWrenCurrent();

  if (!isPreFinalizedCalled())
    preFinalize();

  if (!areOdeObjectsCreated() && (WbWorld::instance()->isLoading() || !WbNodeUtilities::isTrackAnimatedGeometry(this)))
    // in case of nodes descending from Track.animatedGeometries field we don't want to create ODE objects
    // these nodes are automatically skipped if a Track or ancestor node is finalized, so we only have to check in case of node
    // insertion
    createOdeObjects();

  if (!areWrenObjectsInitialized())
    createWrenObjects();

  if (mFinalizationCanceled) {
    WbWrenOpenGlContext::doneWren();
    return;
  }

  setFieldsParentNode();

  if (!isPostFinalizedCalled())
    postFinalize();

  validateProtoNodes();

  WbWrenOpenGlContext::doneWren();

  emit finalizationCompleted(this);
}

void WbBaseNode::postFinalize() {
  mPostFinalizeCalled = true;
  connect(this, &WbNode::defUseNameChanged, WbNodeOperations::instance(), &WbNodeOperations::requestUpdateSceneDictionary);
}

void WbBaseNode::validateProtoNodes() {
  QList<WbNode *> nodes = subNodes(true, false, false);
  nodes.prepend(this);

  foreach (WbNode *node, nodes) {
    if (node->isProtoInstance())
      dynamic_cast<WbBaseNode *>(node)->validateProtoNode();
  }
}

void WbBaseNode::reset() {
  WbNode::reset();
  if (boundingSphere())
    boundingSphere()->resetGlobalCoordinatesUpdateTime();
}

//////////////////////////
// WREN and ODE objects //
//////////////////////////

void WbBaseNode::createWrenObjects() {
  mWrenObjectsCreatedCalled = true;

  if (parent()) {
    const WbBaseNode *const p = static_cast<WbBaseNode *>(parent());
    mWrenNode = p->wrenNode();
  } else
    mWrenNode = wr_scene_get_root(wr_scene_get_instance());
}

void WbBaseNode::updateContextDependentObjects() {
  if (isProtoParameterNode()) {
    // update context of PROTO parameter node instances
    // this has no WREN objects to be updated
    QVector<WbNode *> nodeInstances = protoParameterNodeInstances();
    foreach (WbNode *nodeInstance, nodeInstances) { static_cast<WbBaseNode *>(nodeInstance)->updateContextDependentObjects(); }

  } else {
    QList<WbNode *> sbn = subNodes(false);
    foreach (WbNode *node, sbn)
      static_cast<WbBaseNode *>(node)->updateContextDependentObjects();
  }
}

// Utility functions
bool WbBaseNode::isInBoundingObject() const {
  if (mBoundingObjectFirstTimeSearch) {
    mIsInBoundingObject = WbNodeUtilities::isInBoundingObject(this);
    if (areWrenObjectsInitialized())
      mBoundingObjectFirstTimeSearch = false;
  }

  return mIsInBoundingObject;
}

WbNode::NodeUse WbBaseNode::nodeUse() const {
  if (mNodeUseDirty) {
    mNodeUse = WbNodeUtilities::checkNodeUse(this);
    if (areWrenObjectsInitialized())
      mNodeUseDirty = false;
  }

  return mNodeUse;
}

WbTransform *WbBaseNode::upperTransform() const {
  if (mUpperTransformFirstTimeSearch) {
    mUpperTransform = WbNodeUtilities::findUpperTransform(this);
    if (areWrenObjectsInitialized())
      mUpperTransformFirstTimeSearch = false;
  }

  return mUpperTransform;
}

WbSolid *WbBaseNode::upperSolid() const {
  if (mUpperSolidFirstTimeSearch) {
    mUpperSolid = WbNodeUtilities::findUpperSolid(this);
    if (areWrenObjectsInitialized())
      mUpperSolidFirstTimeSearch = false;
  }

  return mUpperSolid;
}

WbSolid *WbBaseNode::topSolid() const {
  if (mTopSolidFirstTimeSearch) {
    mTopSolid = WbNodeUtilities::findTopSolid(this);
    if (areWrenObjectsInitialized())
      mTopSolidFirstTimeSearch = false;
  }

  return mTopSolid;
}

WbBaseNode *WbBaseNode::getSingleFinalizedProtoInstance() {
  if (!isProtoParameterNode())
    return NULL;

  WbBaseNode *finalizedInstance = NULL;
  QVector<WbNode *> nodeInstances = protoParameterNodeInstances();
  foreach (WbNode *node, nodeInstances) {
    WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(node);
    if (baseNode && baseNode->isPostFinalizedCalled()) {
      if (finalizedInstance)
        // multiple finilized instances found
        return NULL;

      finalizedInstance = baseNode;
    }
  }

  return finalizedInstance;
}

bool WbBaseNode::isInvisibleNode() const {
  return WbWorld::instance()->viewpoint()->getInvisibleNodes().contains(const_cast<WbBaseNode *>(this));
}

bool WbBaseNode::exportNodeHeader(WbVrmlWriter &writer) const {
  if (!writer.isX3d())
    return WbNode::exportNodeHeader(writer);

  writer << "<" << x3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
  if (isInvisibleNode())
    writer << " render=\'false\'";
  QStringList bookAndPage = documentationBookAndPage(WbNodeUtilities::isRobotTypeName(nodeModelName()));
  if (!bookAndPage.isEmpty())
    writer
      << QString(" docUrl=\'%1/doc/%2/%3\'").arg(WbStandardPaths::cyberboticsUrl()).arg(bookAndPage[0]).arg(bookAndPage[1]);

  if (isUseNode() && defNode()) {  // export referred DEF node id
    writer << " USE=\'n" + QString::number(defNode()->uniqueId()) + "\'></" + x3dName() + ">";
    return true;
  }
  return false;
}
