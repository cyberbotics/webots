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

#include "WbControlledWorld.hpp"

#include "WbController.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbProtoManager.hpp"
#include "WbRandom.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtCore/QThread>

#include <cassert>

#ifndef NDEBUG
#include <iostream>
#endif

WbControlledWorld *WbControlledWorld::instance() {
  return static_cast<WbControlledWorld *>(WbSimulationWorld::instance());
}

WbControlledWorld::WbControlledWorld(WbTokenizer *tokenizer) :
  WbSimulationWorld(tokenizer),
  mFirstStep(true),
  mRetryEnabled(false),
  mIsExecutingStep(false),
  mHasWaitingStep(false) {
  if (mWorldLoadingCanceled)
    return;

  mNeedToYield = false;
  foreach (WbRobot *const robot, robots())
    connect(robot, &WbRobot::startControllerRequest, this, &WbControlledWorld::startController);
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
  while (!mDisconnectedExternControllers.isEmpty())
    delete mDisconnectedExternControllers.takeFirst();
}

void WbControlledWorld::setUpControllerForNewRobot(WbRobot *robot) {
  assert(robot);

  connect(robot, &WbRobot::startControllerRequest, this, &WbControlledWorld::startController);

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
  }
  if (robot->controllerName() == "<extern>") {
    mDisconnectedExternControllers.append(controller);
    controller->start();
  } else {
    mControllers.append(controller);
    controller->start();
  }
  assert(showControllersLists("start " + controller->robot()->controllerName()));
  assert(controllerInOnlyOneList(controller));
}

// delete the controller only if it has not been already deleted
void WbControlledWorld::deleteController(WbController *controller) {
  assert(showControllersLists("deleteController " + controller->name()));  // controller->robot() may be NULL at this point
  assert(controllerInOnlyOneList(controller));

  mControllers.removeOne(controller);
  mWaitingControllers.removeOne(controller);
  mNewControllers.removeOne(controller);
  mDisconnectedExternControllers.removeOne(controller);
  mTerminatingControllers.removeOne(controller);

  if (controller->isProcessingRequest())
    mTerminatingControllers.append(controller);
  else
    delete controller;

  if (mRetryEnabled)  // avoid waiting for a terminated controller
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
      foreach (WbController *const controller, mControllers)
        controller->writePendingImmediateAnswer();
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
    assert(showControllersLists("moving from waiting to controllers list"));
    foreach (WbController *const controller, mWaitingControllers) {
      assert(controller->robot()->controllerName() != "<extern>");
      controller->start();
      mControllers.append(controller);
      if (!mFirstStep)
        justStartedControllers.append(controller);
    }
    mWaitingControllers.clear();
    assert(showControllersLists("moved from waiting to controllers list"));
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
          assert(justStartedControllers.count(controller) == 1);
          justStartedControllers.removeOne(controller);
        }
        if (controller->deltaTimeRequested() == 0) {
          retryStepLater();  // execute first step just after init step
          return;
        }
      }
    }
    justStartedControllers.clear();
  }

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
        assert(mTerminatingControllers.count(controller) == 1);
        mTerminatingControllers.removeOne(controller);
        delete controller;
        assert(controllerInNoList(controller));
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
  foreach (WbController *const controller, mDisconnectedExternControllers) {
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

void WbControlledWorld::updateCurrentRobotController() {
  WbRobot *const robot = dynamic_cast<WbRobot *>(sender());
  if (robot)
    updateRobotController(robot);
}

void WbControlledWorld::updateRobotController(WbRobot *robot) {
  assert(robot);
  const int robotId = robot->uniqueId();
  const QString &newControllerName = robot->controllerName();

  for (WbController *controller : mDisconnectedExternControllers) {
    if (controller->robotId() == robotId && !mControllers.contains(controller)) {
      assert(mDisconnectedExternControllers.count(controller) == 1);
      mDisconnectedExternControllers.removeOne(controller);
      WbLog::info(tr("\"%1\" extern controller: stopped.").arg(controller->robot()->name()));
      assert(controllerInNoList(controller));
      delete controller;
      restartStepTimer();
    }
  }
  // There should not be any controller for `robot` in `mWaitingControllers`
  for (WbController *controller : mWaitingControllers)
    if (controller->robotId() == robotId && !mControllers.contains(controller)) {
      assert(mWaitingControllers.count(controller) == 1);
      mWaitingControllers.removeOne(controller);
      assert(controllerInNoList(controller));
      delete controller;
    }

  // restart the controller if needed
  const bool paused = WbSimulationState::instance()->isPaused();
  const int size = mControllers.size();
  for (int i = 0; i < size; ++i) {
    WbController *controller = mControllers[i];
    if (controller->robotId() == robotId) {
      controller->flushBuffers();
      disconnect(controller, &WbController::hasTerminatedByItself, this,
                 &WbControlledWorld::deleteController);  // avoids double delete
      assert(controllerInOnlyOneList(controller));
      mNewControllers.removeOne(controller);
      mWaitingControllers.removeOne(controller);
      mControllers.removeOne(controller);
      if (controller->name() == "<extern>")
        WbLog::info(tr("Terminating extern controller for robot \"%2\".").arg(controller->robot()->name()));
      delete controller;
      assert(controllerInNoList(controller));
      if (newControllerName == "<none>") {
        robot->setControllerStarted(false);
        return;
      }
      controller = new WbController(robot);
      if (newControllerName == "<extern>") {
        mDisconnectedExternControllers.append(controller);
        controller->start();
      } else if (paused)  // step finished
        mWaitingControllers.append(controller);
      else {  // step executing
        mNewControllers.append(controller);
        restartStepTimer();
      }
      connect(controller, &WbController::hasTerminatedByItself, this, &WbControlledWorld::deleteController);
      assert(showControllersLists("started " + newControllerName));
      return;
    }
  }

  if (newControllerName == "<none>") {
    robot->setControllerStarted(false);
    return;
  }

  // The controller has never been created. Creates a new one
  WbController *const controller = new WbController(robot);
  if (newControllerName == "<extern>") {
    mDisconnectedExternControllers.append(controller);
    controller->start();
  } else if (paused)  // step finished
    mWaitingControllers.append(controller);
  else {  // step executing
    mNewControllers.append(controller);
    restartStepTimer();
  }
  connect(controller, &WbController::hasTerminatedByItself, this, &WbControlledWorld::deleteController);
  assert(controllerInOnlyOneList(controller));
}

void WbControlledWorld::externConnection(WbController *controller, bool connect) {
  if (connect) {
    assert(showControllersLists("extern connect " + controller->name() + " " + controller->robot()->name()));
    controller->robot()->notifyExternControllerChanged();

    for (int i = 0; i < mDisconnectedExternControllers.size(); ++i) {
      if (mDisconnectedExternControllers.at(i)->robot() == controller->robot()) {
        mControllers.append(controller);
        mDisconnectedExternControllers.removeAt(i);
        break;
      }
    }

    restartStepTimer();
  } else {
    assert(showControllersLists("extern disconnect " + controller->name() + " " + controller->robot()->name()));
    assert(controller->isProcessingRequest() ? controllerInOnlyOneList(controller) : true);

    for (int i = 0; i < mControllers.size(); ++i) {
      if (mControllers.at(i)->robot() == controller->robot()) {
        mDisconnectedExternControllers.append(controller);
        mControllers.removeAt(i);
        break;
      }
    }

    if (controller->robot()->synchronization())
      pauseStepTimer();
  }
}

QStringList WbControlledWorld::activeControllersNames() const {
  QStringList list;
  foreach (WbController *const controller, mControllers) {
    if (controller && controller->isRunning())
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
    assert(showControllersLists("moving new to waiting"));
    mWaitingControllers.append(mNewControllers);
    mNewControllers.clear();
  }
  if (!needToWait())
    emit WbSimulationState::instance()->controllerReadRequestsCompleted();

  mIsExecutingStep = false;
  processWaitingStep();
}

#ifndef NDEBUG
bool WbControlledWorld::controllerInOnlyOneList(WbController *controller) const {
  return mControllers.count(controller) + mNewControllers.count(controller) + mWaitingControllers.count(controller) +
           mTerminatingControllers.count(controller) + mDisconnectedExternControllers.count(controller) ==
         1;
}
bool WbControlledWorld::controllerInNoList(WbController *controller) const {
  if (mControllers.contains(controller))
    qDebug() << "in mControllers";
  if (mNewControllers.contains(controller))
    qDebug() << "in mNewControllers";
  if (mWaitingControllers.contains(controller))
    qDebug() << "in mWaitingControllers";
  if (mTerminatingControllers.contains(controller))
    qDebug() << "in mTerminatingControllers";
  if (mDisconnectedExternControllers.contains(controller))
    qDebug() << "in mDisconnectedExternControllers";
  return mControllers.count(controller) + mNewControllers.count(controller) + mWaitingControllers.count(controller) +
           mTerminatingControllers.count(controller) + mDisconnectedExternControllers.count(controller) ==
         0;
}
bool WbControlledWorld::showControllersLists(const QString &message) const {
  /*
  QString output;
  if (mControllers.count())
    output += "mControllers:\n";
  foreach (WbController *controller, mControllers)
    output += "  " + controller->robot()->name() + " " + controller->name() + "\n";
  if (mNewControllers.count())
    output += "mNewControllers:\n";
  foreach (WbController *controller, mNewControllers)
    output += "  " + controller->robot()->name() + " " + controller->name() + "\n";
  if (mWaitingControllers.count())
    output += "mWaitingControllers:\n";
  foreach (WbController *controller, mWaitingControllers)
    output += "  " + controller->robot()->name() + " " + controller->name() + "\n";
  if (mTerminatingControllers.count())
    output += "mTerminatingControllers:\n";
  foreach (WbController *controller, mTerminatingControllers)
    output += "  " + controller->robot()->name() + " " + controller->name() + "\n";
  if (mDisconnectedExternControllers.count())
    output += "mDisconnectedExternControllers:\n";
  foreach (WbController *controller, mDisconnectedExternControllers)
    output += "  " + controller->robot()->name() + " " + controller->name() + "\n";
  std::cerr << "-------------------\n" << message.toUtf8().constData() << "\n";
  std::cerr << output.toUtf8().constData();
  */
  return true;
}
#endif
