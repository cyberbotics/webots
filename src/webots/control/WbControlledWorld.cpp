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

#include "WbControlledWorld.hpp"

#include "WbController.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbProtoList.hpp"
#include "WbRandom.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtCore/QThread>

#include <cassert>

WbControlledWorld *WbControlledWorld::instance() {
  return static_cast<WbControlledWorld *>(WbSimulationWorld::instance());
}

WbControlledWorld::WbControlledWorld(WbProtoList *protos, WbTokenizer *tokenizer) :
  WbSimulationWorld(protos, tokenizer),
  mFirstStep(true),
  mRetryEnabled(false),
  mIsExecutingStep(false),
  mHasWaitingStep(false) {
  if (mWorldLoadingCanceled)
    return;

  mNeedToYield = false;
  foreach (WbRobot *const robot, robots()) {
    connect(robot, &WbRobot::startControllerRequest, this, &WbControlledWorld::startController);
    connect(robot, &WbRobot::isBeingDestroyed, this, &WbControlledWorld::handleRobotRemoval);
  }
}

WbControlledWorld::~WbControlledWorld() {
  while (!mControllers.isEmpty())
    delete mControllers.takeFirst();
  while (!mNewControllers.isEmpty())
    delete mNewControllers.takeFirst();
  while (!mWaitingControllers.isEmpty())
    delete mWaitingControllers.takeFirst();
  while (!mTerminatingControllers.isEmpty())
    delete mTerminatingControllers.takeFirst();
  while (!mWaitingExternControllers.isEmpty())
    delete mWaitingExternControllers.takeFirst();
}

void WbControlledWorld::setUpControllerForNewRobot(WbRobot *robot) {
  assert(robot);

  connect(robot, &WbRobot::startControllerRequest, this, &WbControlledWorld::startController);
  connect(robot, &WbRobot::isBeingDestroyed, this, &WbControlledWorld::handleRobotRemoval);

  if (mFirstStep && !mRetryEnabled)
    // simulation not started yet, controller will be created at first step() call
    return;

  updateRobotController(robot);
  connect(robot, &WbRobot::controllerChanged, this, &WbControlledWorld::updateCurrentRobotController, Qt::UniqueConnection);
}

void WbControlledWorld::startController(WbRobot *robot) {
  if (robot->controllerName() == "<none>") {
    connect(robot, &WbRobot::controllerChanged, this, &WbControlledWorld::updateCurrentRobotController, Qt::UniqueConnection);
    return;
  }
  assert(!robot->isControllerStarted());

  WbController *controller = NULL;
  // check if a controller instance already exists for the current robot
  for (int i = 0; i < mWaitingControllers.size(); ++i) {
    if (mWaitingControllers.at(i)->robot() == robot) {
      controller = mWaitingControllers.at(i);
      mWaitingControllers.removeAt(i);
      break;
    }
  }
  if (!controller) {
    for (int i = 0; i < mNewControllers.size(); ++i) {
      if (mNewControllers.at(i)->robot() == robot) {
        controller = mNewControllers.at(i);
        mNewControllers.removeAt(i);
        break;
      }
    }
  }
  if (!controller) {
    // if not first step and not an extern controller, the controller should be created when inserting the Robot node
    assert(mFirstStep || robot->controllerName() == "<extern>");
    controller = new WbController(robot);
    connect(robot, &WbRobot::controllerChanged, this, &WbControlledWorld::updateCurrentRobotController, Qt::UniqueConnection);
    connect(controller, &WbController::hasTerminatedByItself, this, &WbControlledWorld::deleteController, Qt::UniqueConnection);
    if (robot->controllerName() == "<extern>")
      mWaitingExternControllers.append(controller);
  } else if (robot->controllerName() == "<extern>")
    if (!mWaitingExternControllers.removeOne(controller))
      assert(false);
  mControllers.append(controller);
  controller->start();
}

// delete the controller only if it has not been already deleted
void WbControlledWorld::deleteController(WbController *controller) {
  mControllers.removeOne(controller);
  mWaitingControllers.removeOne(controller);
  mWaitingExternControllers.removeOne(controller);
  mNewControllers.removeOne(controller);

  if (controller->isProcessingRequest())
    mTerminatingControllers.append(controller);
  else {
    mTerminatingControllers.removeOne(controller);
    delete controller;
  }

  if (mRetryEnabled)
    // avoid waiting for a terminated controller
    step();
}

void WbControlledWorld::retryStepLater() {
  if (!mRetryEnabled) {
    mRetryEnabled = true;
    emit stepBlocked(true);
  }
  // call the step() function again when a WbController received some data from the libController
  foreach (WbController *const controller, mControllers)
    connect(controller, &WbController::requestReceived, this, &WbControlledWorld::step, Qt::UniqueConnection);
}

void WbControlledWorld::triggerStepFromTimer() {
  mHasWaitingStep = true;
  if (!mIsExecutingStep)
    processWaitingStep();
}

void WbControlledWorld::processWaitingStep() {
  if (mHasWaitingStep) {
    mHasWaitingStep = false;
    step();
  }
}

void WbControlledWorld::reset(bool restartControllers) {
  WbSimulationWorld::reset(restartControllers);
  if (!restartControllers) {
    foreach (WbController *controller, mControllers)
      controller->resetRequestTime();
  }
}

void WbControlledWorld::checkIfReadRequestCompleted() {
  if (mControllers.isEmpty())
    return;
  if (!needToWait()) {
    WbSimulationState *state = WbSimulationState::instance();
    emit state->controllerReadRequestsCompleted();
    if (state->isPaused() || state->isStep()) {
      // in order to avoid mixing immediate messages sent by Webots and the libController
      // some Webots immediate messages could have been postponed
      // if the simulation is running these messages will be sent within the step message
      // otherwise we want to send them as soon as the libController request is over
      writePendingImmediateAnswer();
    }

    // print controller logs to Webots console(s)
    // wait until read request completed to guarantee the printout determinism
    // logs are ordered by controller and not by receiving time
    foreach (WbController *const controller, mControllers)
      controller->flushBuffers();
  }
}

void WbControlledWorld::step() {
  if (mFirstStep && !mRetryEnabled) {
    foreach (WbRobot *const robot, robots()) {
      if (!robot->isControllerStarted())
        startController(robot);
    }
  }

  WbSimulationState *const simulationState = WbSimulationState::instance();

  // starts controllers that were set by the user at the previous time step
  static QList<WbController *> justStartedControllers;
  if (!mWaitingControllers.isEmpty()) {
    foreach (WbController *const controller, mWaitingControllers) {
      if (controller->robot()->controllerName() != "<extern>")
        controller->start();
      mControllers.append(controller);
      if (!mFirstStep)
        justStartedControllers.append(controller);
    }

    mWaitingControllers.clear();
  }

  // we will have to handle the controllers requests here...
  bool waitForExternControllerStart = false;
  bool waitForController = needToWait(&waitForExternControllerStart);
  if (mNeedToYield) {
    QThread::yieldCurrentThread();
    mNeedToYield = false;
  }

  if (waitForController) {
    if (waitForExternControllerStart)
      // stop timer and restore it when extern controller is connected
      pauseStepTimer();
    else if (simulationState->isStep() || simulationState->isPaused())
      // wait for controllers configuration and try to call step function later
      // otherwise the simulation time is not updated when clicking on the step button the first time
      retryStepLater();
    return;
  }

  if (mFirstStep)
    justStartedControllers = mControllers;

  if (!justStartedControllers.isEmpty()) {
    if ((simulationState->isStep() || simulationState->isPaused())) {
      foreach (WbController *const controller, justStartedControllers) {
        if (!mControllers.contains(controller)) {  // controller already terminated
          if (!justStartedControllers.removeOne(controller))
            assert(false);
        }
        if (controller->deltaTimeRequested() == 0) {
          retryStepLater();  // execute first step just after init step
          return;
        }
      }
    }
    justStartedControllers.clear();
  }

  if (mFirstStep)
    mFirstStep = false;

  if (mRetryEnabled) {
    mRetryEnabled = false;
    foreach (WbController *const controller, mControllers)
      disconnect(controller, &WbController::requestReceived, this, &WbControlledWorld::step);
    emit stepBlocked(false);
  }

  // delete controllers that could not be deleted previously because still executing
  if (!mTerminatingControllers.isEmpty()) {
    foreach (WbController *controller, mTerminatingControllers) {
      if (!controller->isProcessingRequest()) {
        if (!mTerminatingControllers.removeOne(controller))
          assert(false);
        delete controller;
      }
    }
  }

  if (mNewControllers.isEmpty()) {
    mIsExecutingStep = true;
    WbSimulationWorld::step();
  }

  waitForRobotWindowIfNeededAndCompleteStep();
}

bool WbControlledWorld::needToWait(bool *waitForExternControllerStart) {
  if (waitForExternControllerStart)
    *waitForExternControllerStart = false;
  foreach (WbController *const controller, mWaitingExternControllers) {
    if (controller->robot()->synchronization()) {
      if (waitForExternControllerStart)
        *waitForExternControllerStart = true;
      return true;
    }
  }
  foreach (WbController *const controller, mControllers) {
    if (!controller->isRequestPending() || controller->isIncompleteRequest()) {
      mNeedToYield = true;
      if (controller->synchronization())
        return true;
    }
  }
  return false;
}

void WbControlledWorld::writePendingImmediateAnswer() {
  foreach (WbController *const controller, mControllers)
    controller->writePendingImmediateAnswer();
}

void WbControlledWorld::updateCurrentRobotController() {
  WbRobot *const robot = dynamic_cast<WbRobot *>(sender());
  if (robot)
    updateRobotController(robot);
}

void WbControlledWorld::updateRobotController(WbRobot *robot) {
  assert(robot);
  const int robotId = robot->uniqueId();
  const int size = mControllers.size();
  bool paused = WbSimulationState::instance()->isPaused();
  const QString &newControllerName = robot->controllerName();

  for (WbController *controller : mWaitingExternControllers) {
    if (controller->robotId() == robotId && !mControllers.contains(controller)) {
      if (mWaitingExternControllers.removeOne(controller)) {
        mNewControllers.removeOne(controller);  // needed in case the extern controller is connected, not needed otherwise
        WbLog::info(tr("\"%1\" extern controller: stopped.").arg(controller->robot()->name()));
        delete controller;
        restartStepTimer();
      } else
        assert(false);
    }
  }
  // There should not be any controller for `robot` in `mWaitingControllers`
  for (WbController *controller : mWaitingControllers)
    if (controller->robotId() == robotId && !mControllers.contains(controller)) {
      if (!mWaitingControllers.removeOne(controller))
        assert(false);
      delete controller;
    }

  // restart the controller if needed
  for (int i = 0; i < size; ++i) {
    WbController *controller = mControllers[i];
    if (controller->robotId() == robotId) {
      controller->flushBuffers();
      disconnect(controller, &WbController::hasTerminatedByItself, this,
                 &WbControlledWorld::deleteController);  // avoids double delete
      mNewControllers.removeOne(controller);
      mWaitingControllers.removeOne(controller);
      if (!mControllers.removeOne(controller))
        assert(false);
      if (newControllerName == "<none>" || newControllerName == "<extern>") {
        if (controller->name() == "<extern>") {
          WbLog::info(tr("Terminating extern controller for robot \"%1\".").arg(controller->robot()->name()));
          mWaitingExternControllers.append(controller);
        } else
          WbLog::info(tr("Terminating controller \"%1\".").arg(controller->name()));
      }
      delete controller;
      if (newControllerName == "<none>") {
        robot->setControllerStarted(false);
        return;
      }
      controller = new WbController(robot);
      if (newControllerName == "<extern>") {
        mWaitingExternControllers.append(controller);
        controller->start();
      }
      if (paused)  // step finished
        mWaitingControllers.append(controller);
      else  // step executing
        mNewControllers.append(controller);
      connect(controller, &WbController::hasTerminatedByItself, this, &WbControlledWorld::deleteController);
      return;
    }
  }

  if (newControllerName.isEmpty())
    return;

  // The controller has never been created. Creates a new one
  WbController *const controller = new WbController(robot);
  if (newControllerName == "<extern>")
    mWaitingExternControllers.append(controller);
  if (paused)  // step finished
    mWaitingControllers.append(controller);
  else  // step executing
    mNewControllers.append(controller);
  connect(controller, &WbController::hasTerminatedByItself, this, &WbControlledWorld::deleteController);
}

void WbControlledWorld::handleRobotRemoval(WbBaseNode *node) {
}

void WbControlledWorld::externConnection(WbController *controller, bool connect) {
  if (connect) {
    if (!mWaitingExternControllers.removeOne(controller))
      assert(false);
    controller->robot()->externControllerChanged();
    restartStepTimer();
  } else {
    if (!mControllers.removeOne(controller))
      assert(false);
    mWaitingExternControllers.append(controller);
    mNewControllers.append(controller);
    pauseStepTimer();
  }
}

QStringList WbControlledWorld::activeControllersNames() const {
  QStringList list;
  foreach (WbController *const controller, mControllers) {
    if (controller->isRunning())
      list.append(controller->name());
  }
  return list;
}

void WbControlledWorld::waitForRobotWindowIfNeededAndCompleteStep() {
  disconnect(this, SLOT(waitForRobotWindowIfNeededAndCompleteStep()));
  const int controllersCount = mControllers.size();
  for (int i = 0; i < controllersCount; ++i) {
    const WbController *controller = mControllers[i];
    if (controller->robot()->isWaitingForWindow()) {
      connect(controller->robot(), &WbRobot::windowReady, this, &WbControlledWorld::waitForRobotWindowIfNeededAndCompleteStep,
              Qt::UniqueConnection);
      return;
    }
  }

  WbSimulationState *const simulationState = WbSimulationState::instance();
  for (int i = 0; i < controllersCount; ++i) {
    WbController *controller = mControllers[i];
    if (!controller->isRequestPending())
      continue;
    double rt = controller->requestTime() + controller->deltaTimeRequested();
    if (rt <= simulationState->time()) {
      controller->writeAnswer();
      continue;
    } else
      // sensors have to be updated regularly independently from controller step
      controller->robot()->updateSensors();
  }
  if (!mNewControllers.isEmpty()) {
    mWaitingControllers.append(mNewControllers);
    mNewControllers.clear();
  }
  if (!needToWait())
    emit WbSimulationState::instance()->controllerReadRequestsCompleted();

  mIsExecutingStep = false;
  processWaitingStep();
}
