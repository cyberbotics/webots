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

#ifndef WB_CONTROLLED_WORLD_HPP
#define WB_CONTROLLED_WORLD_HPP

#include <QtCore/QList>
#include "WbSimulationWorld.hpp"  // TODO: should we rename WbSimulationWorld to WbSimulatedWorld ?

class WbController;

class WbControlledWorld : public WbSimulationWorld {
  Q_OBJECT

public:
  // singleton instance
  static WbControlledWorld *instance();

  // constructors and destructor
  explicit WbControlledWorld(WbTokenizer *tokenizer = NULL);
  virtual ~WbControlledWorld();

  void startController(WbRobot *robot);
  void externConnection(WbController *controller, bool connect);
  QStringList activeControllersNames() const;
  bool needToWait(bool *waitForExternControllerStart = NULL);
  bool isExecutingStep() const { return mIsExecutingStep; }
  void checkIfReadRequestCompleted();
  void reset(bool restartControllers) override;

  void step() override;

  QList<WbController *> controllers() const { return mControllers; }
  QList<WbController *> disconnectedExternControllers() const { return mDisconnectedExternControllers; }

public slots:
  void deleteController(WbController *controller);
  void triggerStepFromTimer() override;

signals:
  void stepBlocked(bool blocked);

protected:
  void setUpControllerForNewRobot(WbRobot *robot) override;

private:
  void updateRobotController(WbRobot *robot);

#ifndef NDEBUG  // debug methods
  bool controllerInOnlyOneList(WbController *controller) const;
  bool controllerInNoList(WbController *controller) const;
  bool showControllersLists(const QString &message) const;
#endif

  QList<WbController *> mControllers;         // currently running controllers (both intern and extern)
  QList<WbController *> mWaitingControllers;  // controllers inserted in previous step and waiting to be started in current step
  QList<WbController *> mNewControllers;      // controllers inserted in current step and waiting next step to start
  QList<WbController *> mTerminatingControllers;         // controllers waiting to be deleted
  QList<WbController *> mDisconnectedExternControllers;  // extern controllers started but unconnected
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
  void updateCurrentRobotController();
  void waitForRobotWindowIfNeededAndCompleteStep();
};

#endif  // WB_CONTROLLED_WORLD_HPP
