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

#include "WbVisualBoundingSphere.hpp"

#include "WbBaseNode.hpp"
#include "WbBoundingSphere.hpp"
#include "WbPerspective.hpp"
#include "WbSphere.hpp"
#include "WbSysInfo.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

static bool gEnabled = false;

WbVisualBoundingSphere *WbVisualBoundingSphere::cInstance = NULL;

WbVisualBoundingSphere *WbVisualBoundingSphere::instance() {
  if (!cInstance)
    cInstance = new WbVisualBoundingSphere();
  return cInstance;
}

void WbVisualBoundingSphere::enable(bool enabled, const WbBaseNode *node) {
  gEnabled = enabled;
  instance()->show(node);
}

void WbVisualBoundingSphere::createSphere(
  /* const Ogre::String &meshName, const float r, const int nRings, const int nSegments */) {
  // TODO_WREN
  // Ogre::SceneManager *const sceneManager = WbWrenRenderingContext::instance()->sceneManager();

  // const int subdivision = 2;
  // const int indexSize = 60 * (4 << (2 * (subdivision - 1))); // from WbSphere::computeIndexSize()
  // Ogre::ManualObject *o = sceneManager->createManualObject(meshName);
  // o->setDynamic(true); // this is needed to avoid a bug for large vertex buffer on some intel graphics cards
  // o->estimateVertexCount(indexSize);
  // o->estimateIndexCount(indexSize);
  // o->begin(WbWrenMaterialManager::name(WbWrenMaterialManager::MAT_ACTIVE_HANDLE_Y), Ogre::RenderOperation::OT_LINE_LIST);
  // int index = 0;
  // for (int i = 0; i < 20; i++)
  //   WbSphere::subdivideToOgreManualObject(
  //     o,
  //     r,
  //     WbSphere::defaultVertex(i, 0),
  //     WbSphere::defaultVertex(i, 1),
  //     WbSphere::defaultVertex(i, 2),
  //     subdivision,
  //     index
  //   );
  // o->end();
  // Ogre::MeshPtr mesh = o->convertToMesh(meshName);
  // sceneManager->destroyManualObject(o);
  // mesh->_setBounds( Ogre::AxisAlignedBox( Ogre::Vector3(-r, -r, -r), Ogre::Vector3(r, r, r) ), false );
  // mesh->_setBoundingSphereRadius(r);
}

WbVisualBoundingSphere::WbVisualBoundingSphere() : QObject() {
  gEnabled = !WbSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty() &&
             WbWorld::instance()->perspective()->isGlobalOptionalRenderingEnabled("BoundingSphere");
}

WbVisualBoundingSphere::~WbVisualBoundingSphere() {
  clear();
}

void WbVisualBoundingSphere::show(const WbBaseNode *node) {
  // TODO_WREN
  // WbBoundingSphere *boundingSphere = node ? node->boundingSphere() : NULL;
  // if (!boundingSphere || !gEnabled) {
  //   if (mSceneNode)
  //     clear();
  //   return;
  // }

  // Ogre::SceneManager *sceneManager = WbWrenRenderingContext::instance()->sceneManager();

  // // only one bounding sphere can be visible at a time
  // if (!mSceneNode) {
  //   const Ogre::String &meshName   = "mesh_debug_boundingSphere";
  //   createSphere(meshName, 1.0, 32, 32);
  //   assert(!mEntity);
  //   mEntity = sceneManager->createEntity("entity_debug_boundingSphere", meshName);
  //   mEntity->setVisibilityFlags(WbWrenRenderingContext::VF_SELECTED_OUTLINE);
  //   mEntity->setQueryFlags(WbWrenRenderingContext::QM_NOT_QUERIABLE);
  //   mEntity->setCastShadows(false);
  //   mEntity->setVisible(true);

  //   mSceneNode = sceneManager->getRootSceneNode()->createChildSceneNode();
  //   mSceneNode->setInheritOrientation(false);
  //   mSceneNode->setInheritScale(false);
  //   mSceneNode->attachObject(mEntity);
  // }

  // assert(mSceneNode && mEntity);
  // double radius;
  // WbVector3 pos;
  // boundingSphere->recomputeIfNeeded();
  // const_cast<WbBoundingSphere *>(boundingSphere)->computeSphereInGlobalCoordinates(pos, radius);
  // mSceneNode->setScale(radius, radius, radius);
  // mSceneNode->setPosition(pos.x(), pos.y(), pos.z());
}

void WbVisualBoundingSphere::clear() {
  // TODO_WREN
  // if (!mSceneNode)
  //   return;

  // assert(mEntity);
  // Ogre::SceneManager *const sceneManager = WbWrenRenderingContext::instance()->sceneManager();
  // if (mEntity->isAttached())
  //   mSceneNode->detachObject(mEntity);
  // sceneManager->destroyEntity(mEntity);
  // sceneManager->destroySceneNode(mSceneNode);
  // mEntity = NULL;
  // mSceneNode = NULL;
}
