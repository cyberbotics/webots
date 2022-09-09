// Copyright 1996-2022 Cyberbotics Ltd.
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

#ifndef WB_SIMULATION_WORLD_HPP
#define WB_SIMULATION_WORLD_HPP

//
// Description: world with physics/kinematic simulation
//

#include <QtCore/QElapsedTimer>
#include <QtCore/QList>
#include <QtCore/QMutex>

#include "WbWorld.hpp"

class WbSimulationCluster;
class WbOdeContext;
class WbPhysicsPlugin;
class QTimer;

class WbSimulationWorld : public WbWorld {
  Q_OBJECT

public:
  // singleton instance
  static WbSimulationWorld *instance();

  // constructors and destructor
  explicit WbSimulationWorld(WbTokenizer *tokenizer = NULL);
  virtual ~WbSimulationWorld();

  // returns the physics plugin used in this world or NULL if none
  WbPhysicsPlugin *physicsPlugin() const { return mPhysicsPlugin; }
  WbOdeContext *odeContext() const { return mOdeContext; }

  bool simulationHasRunAfterSave();

  bool saveAs(const QString &fileName) override;
  void reset(bool restartControllers) override;

  void pauseStepTimer();
  void restartStepTimer();

  virtual void step();

public slots:
  void modeChanged();
  void rayTracingEnabled();
  void updateNumberOfThreads();
  void checkNeedForBoundingMaterialUpdate();
  virtual void triggerStepFromTimer(){};

signals:
  void physicsStepStarted();
  void physicsStepEnded();
  void cameraRenderingStarted();
  void simulationStartedAfterSave(bool started);

protected slots:
  void storeAddedNodeIfNeeded(WbNode *node) override;

private:
  WbSimulationCluster *mCluster;
  WbOdeContext *mOdeContext;
  WbPhysicsPlugin *mPhysicsPlugin;
  QTimer *mTimer;
  QElapsedTimer mRealTimeTimer;
  double mSleepRealTime;
  QList<int> mElapsedTimeHistory;
  QVector<WbNode *> mAddedNode;  // list of nodes added since the simulation started

  void storeLastSaveTime() override;
  bool mSimulationHasRunAfterSave;
  void propagateBoundingObjectMaterialUpdate(bool onMenuAction);

private slots:
  void removeNodeFromAddedNodeList(QObject *node);
  void propagateBoundingObjectUpdate() { propagateBoundingObjectMaterialUpdate(false); }
  void updateRandomSeed();
};

#endif
