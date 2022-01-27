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

#ifndef WB_CONTROLLED_WORLD_HPP
#define WB_CONTROLLED_WORLD_HPP

#include <QtCore/QList>
#include "WbSimulationWorld.hpp"  // TODO: should we rename WbSimulationWorld to WbSimulatedWorld ?

class QLocalServer;
class QLocalSocket;
class WbController;

class WbControlledWorld : public WbSimulationWorld {
  Q_OBJECT

public:
  // singleton instance
  static WbControlledWorld *instance();

  // constructors and destructor
  WbControlledWorld(WbProtoList *protos = NULL, WbTokenizer *tokenizer = NULL);
  virtual ~WbControlledWorld();

  void startControllers();
  void startController(WbRobot *robot);

  QStringList activeControllersNames() const;
  bool needToWait(bool *waitForExternControllerStart = NULL);
  void writePendingImmediateAnswer();
  bool isExecutingStep() const { return mIsExecutingStep; }
  void checkIfReadRequestCompleted();

  void reset(bool restartControllers) override;

  void step() override;

public slots:
  void deleteController(WbController *controller);
  void triggerStepFromTimer() override;

signals:
  void stepBlocked(bool blocked);

protected:
  void setUpControllerForNewRobot(WbRobot *robot) override;

private:
  void startControllerFromSocket(WbRobot *robot, QLocalSocket *socket);
  void updateRobotController(WbRobot *robot);
  void handleRobotRemoval(WbBaseNode *node);

  QLocalServer *mServer;
  QList<WbController *> mControllers;
  QList<WbController *> mWaitingControllers;  // controllers inserted in previous step and waiting to be started in current step
  QList<WbController *> mNewControllers;      // controllers inserted in current step mode and waiting next step to start
  QList<WbController *> mTerminatingControllers;    // controllers waiting to be deleted
  QList<WbRobot *> mRobotsWaitingExternController;  // robots with extern controller not started
  QList<double> mRequests;
  bool mNeedToYield;
  bool mFirstStep;

  // wait for controller synchronization in step mode
  bool mRetryEnabled;
  void retryStepLater();
  void processWaitingStep();

  // avoid executing a new step before the current one is completed
  bool mIsExecutingStep;  // flag indicating if a step is currently being processed
  bool mHasWaitingStep;   // flag indicating if a new step execution has been requested

private slots:
  void addControllerConnection();
  void updateCurrentRobotController();
  void waitForRobotWindowIfNeededAndCompleteStep();
};

#endif  // WB_CONTROLLED_WORLD_HPP
