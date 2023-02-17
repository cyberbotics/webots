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

#include "WbPhysicsPlugin.hpp"

#include "WbBasicJoint.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbProject.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbReceiver.hpp"
#include "WbSimulationState.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbStandardPaths.hpp"
#include "WbWorld.hpp"

#include <QtCore/QDir>

WbPhysicsPlugin *gInstance = NULL;

#ifdef __linux__
#include "../../../include/plugins/physics.h"
#else
extern "C" {
dGeomID dWebotsGetGeomFromDEF(const char *);
dBodyID dWebotsGetBodyFromDEF(const char *);
dJointGroupID dWebotsGetContactJointGroup();
void dWebotsSend(int, const void *, int);
const void *dWebotsReceive(int *);
void dWebotsConsolePrintf(const char *, ...);
double dWebotsGetTime();
}
typedef void (*PhysicsSpecialInitFunc)(dGeomID (*dWebotsGetGeomFromDEFProcA)(const char *),
                                       dBodyID (*dWebotsGetBodyFromDEFProcA)(const char *),
                                       dJointGroupID (*dWebotsGetContactJointGroupProcA)(),
                                       void (*dWebotsSendProcA)(int, const void *, int),
                                       const void *(*dWebotsReceiveProcA)(int *),
                                       void (*dWebotsConsolePrintfProcA)(const char *, ...),
                                       double (*dWebotsGetTimeProcA)(void));
#endif

extern "C" {
dGeomID dWebotsGetGeomFromDEF(const char *defName) {
  if (!defName || !defName[0]) {
    WbLog::warning(QObject::tr("dWebotsGetGeomFromDEF(): invalid NULL or empty DEF argument."));
    return NULL;
  }

  const WbSolid *const solid = gInstance->findSolidByDef(QString(defName));
  if (solid == NULL)
    return NULL;

  return solid->odeGeom();
}

dBodyID dWebotsGetBodyFromDEF(const char *defName) {
  if (!defName || !defName[0]) {
    WbLog::warning(QObject::tr("dWebotsGetBodyFromDEF(): invalid NULL or empty DEF argument."));
    return NULL;
  }

  const WbSolid *const solid = gInstance->findSolidByDef(QString(defName));
  if (solid == NULL)
    return NULL;

  return solid->bodyMerger();
}

dJointGroupID dWebotsGetContactJointGroup() {
  return gInstance->currentContactJointGroup();
}

void dWebotsSend(int channel, const void *buffer, int size) {
  if (!buffer || size < 1) {
    WbLog::warning(QObject::tr("dWebotsSend(): invalid argument(s): buffer == NULL or size < 1."));
    return;
  }

  WbReceiver::transmitData(channel, buffer, size);
}

const void *dWebotsReceive(int *size) {
  if (gInstance->resetReceiverBufferFlag())
    gInstance->resetReceiverBufferSize();

  if (size)
    *size = gInstance->receiverBufferSize();

  if (gInstance->receiverBufferSize() == 0)
    return NULL;

  gInstance->raiseResetReceiverBufferFlag();

  return gInstance->receiverBuffer();
}

void dWebotsConsolePrintf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  WbLog::appendStdout(QString::vasprintf(format, args), WbLog::PHYSICS_PLUGINS);
  va_end(args);
}

double dWebotsGetTime() {
  return WbSimulationState::instance()->time();  // milliseconds
}
}

WbPhysicsPlugin *WbPhysicsPlugin::instance() {
  return gInstance;
}

WbPhysicsPlugin::WbPhysicsPlugin(const QString &name) : WbPlugin(name), mResetReceiverBufferFlag(false), mCurrentGroupID(0) {
  gInstance = this;
}

WbPhysicsPlugin::~WbPhysicsPlugin() {
  if (isLoaded())
    cleanup();  // calls webots_physics_cleanup()
  gInstance = NULL;
}

void WbPhysicsPlugin::connectReceiver(const WbReceiver *receiver) {
  connect(receiver, &WbReceiver::dataReceived, this, &WbPhysicsPlugin::receiveData, Qt::DirectConnection);
}

void WbPhysicsPlugin::receiveData(const void *data, int size) {
  if (mResetReceiverBufferFlag)
    resetReceiverBufferSize();

  mReceiverBuffer.append(static_cast<const char *>(data), size);
}

void WbPhysicsPlugin::resetReceiverBufferSize() {
  mReceiverBuffer.resize(0);
  mResetReceiverBufferFlag = false;
}

// exclamation mark (!) indicates mandatory mFunctions
static const char *FUNCTION_NAMES[] = {" physics_special_init", "!webots_physics_init",     " webots_physics_collide",
                                       "!webots_physics_step",  " webots_physics_step_end", "!webots_physics_cleanup",
                                       " webots_physics_draw"};

int WbPhysicsPlugin::functionCount() const {
  return sizeof(FUNCTION_NAMES) / sizeof(char *);
}

const char *WbPhysicsPlugin::functionName(int index) const {
  return FUNCTION_NAMES[index];
}

bool WbPhysicsPlugin::load() {
  if (!WbPlugin::load())
    return false;

#ifndef __linux__
  if (!mFunctions[SPECIAL_INIT]) {
    WbLog::error(tr("Could not find physics_special_init() in '%1' plugin.").arg(name()));
    return false;
  } else {
    // cast function
    PhysicsSpecialInitFunc physics_special_init = reinterpret_cast<PhysicsSpecialInitFunc>(mFunctions[SPECIAL_INIT]);

    // invoke function
    (*physics_special_init)(&dWebotsGetGeomFromDEF, &dWebotsGetBodyFromDEF, &dWebotsGetContactJointGroup, &dWebotsSend,
                            &dWebotsReceive, &dWebotsConsolePrintf, &dWebotsGetTime);
  }
#endif

  if (mFunctions[DRAW])
    WbLog::warning(QObject::tr("In plugin '%1', webots_physics_draw(int, const char*) is deprecated.").arg(name()));

  return true;
}

// init() must be called from the plugin directory
void WbPhysicsPlugin::init() {
  if (!mFunctions[INIT])  // no init() function: do nothing
    return;

  // remember current directory
  const QString oldDir(QDir::currentPath());
  const QString newDir(dirPath());

  if (!QDir(newDir).exists())
    WbLog::error(tr("Can't find plugin directory: '%1'.").arg(newDir));

  if (!QDir::setCurrent(newDir))
    WbLog::error(tr("Can't change directory to: '%1'.").arg(newDir));

  // invoke plugin's webots_physics_init() function
  (*(reinterpret_cast<void (*)()>(mFunctions[INIT])))();

  // restore current directory
  if (!QDir::setCurrent(oldDir))
    WbLog::error(tr("Can't change directory to: '%1'.").arg(oldDir));

  mCurrentGroupID = 0;
}

int WbPhysicsPlugin::collide(dGeomID g1, dGeomID g2) {
  return mFunctions[COLLIDE] ? (*(reinterpret_cast<int (*)(dGeomID, dGeomID)>(mFunctions[COLLIDE])))(g1, g2) : 0;
};

void WbPhysicsPlugin::step() {
  if (mResetReceiverBufferFlag)
    resetReceiverBufferSize();

  (*(reinterpret_cast<void (*)()>(mFunctions[STEP])))();
}

const WbSolid *WbPhysicsPlugin::findSolidByDef(const QString &def) const {
  const QStringList list(def.split("."));
  QStringListIterator it(list);

  if (!it.hasNext())
    return NULL;

  // search sub elements
  const WbNode *node = findNodeByDef(WbWorld::instance()->root(), it.next());
  while (node && it.hasNext())
    node = findNodeByDef(node, it.next());

  // make sure the correct solid is returned
  const WbSolid *const solid = dynamic_cast<const WbSolid *>(node);
  return (solid && solid->defName() == list.last()) ? solid : NULL;
}

const WbNode *WbPhysicsPlugin::findNodeByDef(const WbNode *node, const QString &def) const {
  if (node->defName() == def)
    return node;

  const WbBasicJoint *const joint = dynamic_cast<const WbBasicJoint *>(node);
  if (joint) {
    const WbSolid *endPoint = joint->solidEndPoint();
    if (endPoint) {
      const WbNode *const result = findNodeByDef(endPoint, def);
      if (result)
        return result;
    }
  }

  const WbGroup *const group = dynamic_cast<const WbGroup *>(node);
  if (group) {
    WbMFNode::Iterator it(group->children());
    while (it.hasNext()) {
      const WbNode *const result = findNodeByDef(it.next(), def);
      if (result)
        return result;
    }
  }

  return NULL;
}

QString WbPhysicsPlugin::findSourceFileForPlugin(const QString &name) {
  static const char *EXTENSIONS[] = {".c", ".cc", ".cpp", NULL};

  for (int i = 0; EXTENSIONS[i]; ++i) {
    const QString fileName("plugins/physics/" + name + "/" + name + EXTENSIONS[i]);

    QString path(WbProject::current()->path() + fileName);
    if (QFile::exists(path))
      return path;

    path = WbStandardPaths::resourcesProjectsPath() + fileName;
    if (QFile::exists(path))
      return path;

    // search in projects folder of loaded PROTOs
    foreach (WbProtoModel *model, WbProtoManager::instance()->models()) {
      if (!model->path().isEmpty()) {
        path = model->path() + "../" + fileName;
        if (QFile::exists(path))
          return path;
      }
    }
  }

  return "";
}
