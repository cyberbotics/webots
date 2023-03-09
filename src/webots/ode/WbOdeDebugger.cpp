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

#include "WbOdeDebugger.hpp"
#include "WbOdeContext.hpp"
#include "WbSimulationWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <ode/ode_MT.h>
#include <cassert>

WbOdeDebugger *WbOdeDebugger::cOdeDebugger = NULL;

WbOdeDebugger::WbOdeDebugger() {
  cOdeDebugger = this;
  // odeDebugNode = NULL;
}

WbOdeDebugger::~WbOdeDebugger() {
  cOdeDebugger = NULL;
}

// Ogre::ManualObject* WbOdeDebugger::makeClusterCell(const Ogre::String name, WbSFVector3 *_position, WbSFVector3 *_size,
// WbSFVector3 *_color)
// {
// TODO_WREN
// Ogre::Material *mOgreMaterial = WbWrenMaterialManager::material(WbWrenMaterialManager::MAT_BASE_WHITE_NO_LIGHTING);

// double x  = _position->value().x();
// double y  = _position->value().y();
// double z  = _position->value().z();

// double sx = _size->value().x();
// double sy = _size->value().y();
// double sz = _size->value().z();

// x -= sx/2.0f;
// y -= sy/2.0f;
// z -= sz/2.0f;

// Ogre::ManualObject *o;
// Ogre::ManualObject *retVal = NULL;
// if (WbWrenRenderingContext::instance()->sceneManager()->hasManualObject(name)) {
//   o = WbWrenRenderingContext::instance()->sceneManager()->getManualObject(name);
//   o->beginUpdate(0);
// } else {
//   o = WbWrenRenderingContext::instance()->sceneManager()->createManualObject(name);
//   o->setVisibilityFlags(WbWrenRenderingContext::VF_ODE_DEBUG_INFO);
//   o->estimateVertexCount(8);
//   o->estimateIndexCount(24);
//   o->setDynamic(true);
//   retVal = o;
//   o->begin(mOgreMaterial->getName(), Ogre::RenderOperation::OT_LINE_LIST);
// }

// o->colour(_color->value().x(), _color->value().y(), _color->value().z(), 1.0f);

// o->position(x+sx, y+sy, z+sz);

// o->position(x+sx, y+sy, z);
// o->position(x+sx, y, z);
// o->position(x+sx, y, z+sz);
// o->position(x, y+sy, z+sz);
// o->position(x, y+sy, z);
// o->position(x, y, z);
// o->position(x, y, z+sz);

// o->index(0); o->index(1);
// o->index(1); o->index(2);
// o->index(2); o->index(3);
// o->index(3); o->index(0);

// o->index(0); o->index(4);
// o->index(1); o->index(5);
// o->index(2); o->index(6);
// o->index(3); o->index(7);

// o->index(4); o->index(5);
// o->index(5); o->index(6);
// o->index(6); o->index(7);
// o->index(7); o->index(4);

// o->end();

// if (retVal != NULL)
//   odeDebugNode->attachObject(retVal);

// return retVal;
//   return NULL;
// }

// cppcheck-suppress functionStatic
void WbOdeDebugger::toggleDebugging(bool val) {
  // TODO_WREN
  // if (val == false)
  // {
  //     if (odeDebugNode != NULL)
  //     {
  //         cleanupExtraManualObjects(0);
  //         WbWrenRenderingContext::instance()->sceneManager()->getRootSceneNode()->removeAndDestroyChild("ODEDebugging");
  //     }

  //     odeDebugNode = NULL;
  // } else if (odeDebugNode == NULL)
  //     odeDebugNode =
  //     WbWrenRenderingContext::instance()->sceneManager()->getRootSceneNode()->createChildSceneNode("ODEDebugging");
}

double colorPalette[][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// cppcheck-suppress functionStatic
void WbOdeDebugger::step() {
  // TODO_WREN
  // if (odeDebugNode == NULL)
  //   return;

  // odeWorld = WbSimulationWorld::instance()->odeContext()->world();
  // odeSpace = WbSimulationWorld::instance()->odeContext()->space();

  // WbSFVector3 *size = new WbSFVector3();
  // size->setValue(0.1, 0.1, 0.1);
  // WbSFVector3 *position = new WbSFVector3();
  // position->setValue(0.0, 0.0, 0.0);
  // WbSFVector3 *color = new WbSFVector3();
  // color->setValue(1.0, 1.0, 1.0);

  // int clusterCount = dClusterGetCount(odeWorld, odeSpace);
  // float hsAxis[3];
  // dClusterGetCenter(odeWorld, odeSpace, hsAxis[0], hsAxis[1], hsAxis[2]);
  // float gridstep = dClusterGetGridStep(odeWorld, odeSpace);
  // dxClusterNode** clusterAABBs = dClusterGetClusterAABBs(odeWorld, odeSpace);

  // int k = 0;
  // for (int i=0; i<clusterCount; i++)
  // {
  //   float phi = (i+1)/((float)clusterCount+1);
  //   float Y = phi;
  //   float U = i*cos(phi);
  //   float V = i*sin(phi);
  //   float R=Y + V/0.88f;
  //   float G=Y - 0.38f*U - 0.58f*V;
  //   float B=Y + U/0.49f;
  //   color->setValue(R, G, B);
  //   // int j = 0;
  //   for (dxClusterNode* curNode = clusterAABBs[i]; curNode; curNode = curNode->next)
  //   {
  //       for (int xi = curNode->aabb->dbounds[0]; xi <= curNode->aabb->dbounds[1]; xi += 1)
  //       for (int yi = curNode->aabb->dbounds[2]; yi <= curNode->aabb->dbounds[3]; yi += 1)
  //       for (int zi = curNode->aabb->dbounds[4]; zi <= curNode->aabb->dbounds[5]; zi += 1)
  //       {
  //           Ogre::String clusterCellName = "ClusterCell" + Ogre::StringConverter::toString(k++);

  //           position->setValue((xi + 0.5f) * gridstep + hsAxis[0],
  //                               (yi + 0.5f) * gridstep + hsAxis[1],
  //                                (zi + 0.5f) * gridstep + hsAxis[2]);
  //           size->setValue(gridstep, gridstep, gridstep);
  //           makeClusterCell(clusterCellName, position, size, color);
  //       }

  //   }
  // }

  //   delete size;
  //   delete position;
  //   delete color;
  //   cleanupExtraManualObjects(k);
}

/*
void WbOdeDebugger::cleanupExtraManualObjects(int j)
{
  // TODO_WREN
  // int k = j;
  // Ogre::String clusterCellName = "ClusterCell" + Ogre::StringConverter::toString(k++);
  // while (WbWrenRenderingContext::instance()->sceneManager()->hasManualObject(clusterCellName))
  // {
  //   Ogre::ManualObject* o = WbWrenRenderingContext::instance()->sceneManager()->getManualObject(clusterCellName);
  //   WbWrenRenderingContext::instance()->sceneManager()->destroyManualObject(o);
  //   clusterCellName = "ClusterCell" + Ogre::StringConverter::toString(k++);
  // }
}
*/
