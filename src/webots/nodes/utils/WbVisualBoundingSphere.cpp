// Copyright 1996-2020 Cyberbotics Ltd.
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
  // TODO_WREN: port the functionality to WREN.
  // The visual bounding sphere can be enabled from the optional rendering if WEBOTS_DEBUG is set.
}

WbVisualBoundingSphere::WbVisualBoundingSphere() : QObject() {
  gEnabled = !WbSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty() &&
             WbWorld::instance()->perspective()->isGlobalOptionalRenderingEnabled("BoundingSphere");
}

WbVisualBoundingSphere::~WbVisualBoundingSphere() {
  clear();
}

void WbVisualBoundingSphere::show(const WbBaseNode *node) {
  // TODO_WREN: port the functionality from Ogre to WREN.
  // The visual bounding sphere can be enabled from the optional rendering if WEBOTS_DEBUG is set.

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
  // TODO_WREN: port the functionality from Ogre to WREN.
  // The visual bounding sphere can be enabled from the optional rendering if WEBOTS_DEBUG is set.

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
