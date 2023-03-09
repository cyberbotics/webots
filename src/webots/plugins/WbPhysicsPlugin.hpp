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

#ifndef WB_PHYSICS_PLUGIN_HPP
#define WB_PHYSICS_PLUGIN_HPP

#include <ode/ode.h>
#include <QtCore/QByteArray>
#include "WbPlugin.hpp"

class WbReceiver;
class WbSolid;
class WbNode;

class WbPhysicsPlugin : public WbPlugin {
  Q_OBJECT

public:
  // singleton instance
  static WbPhysicsPlugin *instance();

  // constructor & destructor
  // name: name of the plugin, e.g. "salamander_physics"
  explicit WbPhysicsPlugin(const QString &name);
  virtual ~WbPhysicsPlugin();

  // reimplemented public functions
  bool load() override;

  // user implemented functions in shared library
  void init();
  int collide(dGeomID g1, dGeomID g2);
  void step();
  void stepEnd() {
    if (mFunctions[STEP_END])
      (*(reinterpret_cast<void (*)()>(mFunctions[STEP_END])))();
  }
  void cleanup() { (*(reinterpret_cast<void (*)()>(mFunctions[CLEANUP])))(); }

  // for communication between Emitter and dWebotsReceive()
  void connectReceiver(const WbReceiver *receiver);
  int receiverBufferSize() const { return mReceiverBuffer.size(); }
  void resetReceiverBufferSize();
  void raiseResetReceiverBufferFlag() { mResetReceiverBufferFlag = true; }
  bool resetReceiverBufferFlag() { return mResetReceiverBufferFlag; }
  const void *receiverBuffer() const { return mReceiverBuffer.constData(); }

  // find a solid by its DEF name
  const WbSolid *findSolidByDef(const QString &def) const;

  // to open file from the GUI
  // name: name of the plugin, e.g. "salamander_physics"
  static QString findSourceFileForPlugin(const QString &name);

  void setCurrentContactJointGroup(dJointGroupID id) { mCurrentGroupID = id; }
  dJointGroupID currentContactJointGroup() { return mCurrentGroupID; }

protected:
  // reimplemented protected functions
  const QString &type() const override {
    static const QString TYPE("physics");
    return TYPE;
  }
  int functionCount() const override;
  const char *functionName(int index) const override;
  bool shouldCopyBeforeLoad() const override { return true; }

private:
  // for dWebotsReceive()
  QByteArray mReceiverBuffer;
  bool mResetReceiverBufferFlag;

  dJointGroupID mCurrentGroupID;

  const WbNode *findNodeByDef(const WbNode *node, const QString &def) const;
  enum CallbackType { SPECIAL_INIT = 0, INIT = 1, COLLIDE = 2, STEP = 3, STEP_END = 4, CLEANUP = 5, DRAW = 6 };

private slots:
  void receiveData(const void *data, int size);
};

#endif
