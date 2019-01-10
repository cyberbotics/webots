// Copyright 1996-2018 Cyberbotics Ltd.
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

#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtCore/QThread>
#include <QtNetwork/QLocalServer>
#include <QtNetwork/QLocalSocket>

#include <cassert>

WbControlledWorld *WbControlledWorld::instance() {
  return static_cast<WbControlledWorld *>(WbSimulationWorld::instance());
}

WbControlledWorld::WbControlledWorld(WbProtoList *protos, WbTokenizer *tokenizer) :
  WbSimulationWorld(protos, tokenizer),
  mServer(NULL),
  mFirstStep(true),
  mRetryEnabled(false),
  mIsExecutingStep(false),
  mHasWaitingStep(false) {
  if (mWorldLoadingCanceled)
    return;
  // create a unique name for this Webots instance and store it in the environment for the controller child process
  // Note: the random factor was added after having detected an occasional error (~1/20 times) in jenkins at
  // during the mServer->listen() call ("Address in use")
  const unsigned int seed = WbRandom::getSeed();
  WbRandom::setSeed(QDateTime::currentMSecsSinceEpoch());
  static QString serverName = QString("webots_%1_%2").arg(QCoreApplication::applicationPid()).arg(WbRandom::nextUniform());
  WbRandom::setSeed(seed);

  // recover from a crash, when the previous server instance has not been cleaned up
  bool success = QLocalServer::removeServer(serverName);
  if (!success) {
    WbLog::error(tr("Cannot cleanup the local server (server name = \"%1\").").arg(serverName));
    return;
  }

  mServer = new QLocalServer();
  connect(mServer, &QLocalServer::newConnection, this, &WbControlledWorld::addControllerConnection);

  success = mServer->listen(serverName);
  if (!success) {
    WbLog::error(tr("Cannot listen the local server (server name = \"%1\"): %2").arg(serverName).arg(mServer->errorString()));
    return;
  }
  mNeedToYield = false;
  qputenv("WEBOTS_SERVER", mServer->fullServerName().toUtf8());

  foreach (WbRobot *const robot, robots())
    connect(robot, &WbRobot::startControllerRequest, this, &WbControlledWorld::startController);
}

WbControlledWorld::~WbControlledWorld() {
  WbController *controller = NULL;
  while (!mNewControllers.isEmpty()) {
    controller = mNewControllers.takeFirst();
    delete controller;
  }
  while (!mWaitingControllers.isEmpty()) {
    controller = mWaitingControllers.takeFirst();
    delete controller;
  }
  while (!mControllers.isEmpty()) {
    controller = mControllers.takeFirst();
    delete controller;
  }
  while (!mTerminatingControllers.isEmpty()) {
    controller = mTerminatingControllers.takeFirst();
    delete controller;
  }
  delete mServer;
}

void WbControlledWorld::setUpControllerForNewRobot(WbRobot *robot) {
  if (!robot)
    return;

  connect(robot, &WbRobot::startControllerRequest, this, &WbControlledWorld::startController);

  if (mFirstStep && !mRetryEnabled)
    // simulation not started yet, controller will be created at first step() call
    return;

  updateRobotController(robot);
  connect(robot, &WbRobot::controllerChanged, this, &WbControlledWorld::updateCurrentRobotController, Qt::UniqueConnection);
}

void WbControlledWorld::startControllers() {
  foreach (WbRobot *const robot, robots()) {
    if (!robot->isControllerStarted())
      startController(robot);
  }
}

void WbControlledWorld::startController(WbRobot *robot) {
  if (robot->controllerName().isEmpty()) {
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
    assert(mFirstStep);  // if not first step the controller should be created when inserting the Robot node
    controller = new WbController(robot);
    connect(robot, &WbRobot::controllerChanged, this, &WbControlledWorld::updateCurrentRobotController, Qt::UniqueConnection);
    connect(controller, &WbController::hasTerminatedByItself, this, &WbControlledWorld::deleteController, Qt::UniqueConnection);
  }
  mControllers.append(controller);
  controller->start();
}

// delete the controller only if it has not been already deleted
void WbControlledWorld::deleteController(WbController *controller) {
  mControllers.removeOne(controller);
  mWaitingControllers.removeOne(controller);
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

void WbControlledWorld::addControllerConnection() {
  QLocalSocket *socket = mServer->nextPendingConnection();
  socket->waitForReadyRead();

  QByteArray bytes = socket->read(sizeof(int));
  QDataStream stream(bytes);
  stream.setByteOrder(QDataStream::LittleEndian);
  int robotId = 0;
  stream >> robotId;

  // qDebug() << "WbControlledWorld: received" << bytes.size() << "bytes for robot_id =" << robotId << ":";
  foreach (WbController *const controller, mControllers) {
    if (controller->robotId() == robotId) {
      controller->setSocket(socket);
      return;
    }
  }
  // if the robot is not found, it could be that it was deleted meanwhile
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

void WbControlledWorld::step() {
  if (mFirstStep && !mRetryEnabled) {
    startControllers();
  }

  WbSimulationState *const simulationState = WbSimulationState::instance();

  // starts controllers that were set by the user at the previous time step
  static QList<WbController *> justStartedControllers;
  if (!mWaitingControllers.isEmpty()) {
    foreach (WbController *const controller, mWaitingControllers) {
      controller->start();
      mControllers << controller;
      if (!mFirstStep)
        justStartedControllers << controller;
    }

    mWaitingControllers.clear();
  }

  // we will have to handle the controllers requests here...
  bool waitForController = needToWait();
  if (mNeedToYield) {
    QThread::yieldCurrentThread();
    mNeedToYield = false;
  }

  if (waitForController) {
    // wait for controllers configuration and try to call step function later
    // otherwise the simulation time is not updated when clicking on the step button the first time
    if ((simulationState->isStep() || simulationState->isPaused()))
      retryStepLater();
    return;
  }

  if (mFirstStep)
    justStartedControllers = mControllers;

  if (!justStartedControllers.isEmpty()) {
    if ((simulationState->isStep() || simulationState->isPaused())) {
      foreach (WbController *const controller, justStartedControllers) {
        if (!mControllers.contains(controller))
          justStartedControllers.removeOne(controller);  // controller already terminated
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
        mTerminatingControllers.removeOne(controller);
        delete controller;
      }
    }
  }

  mIsExecutingStep = true;
  WbSimulationWorld::step();

  waitForRobotWindowIfNeededAndCompleteStep();
}

bool WbControlledWorld::needToWait() {
  foreach (WbController *const controller, mControllers)
    if (!controller->isRequestPending() || controller->isIncompleteRequest()) {
      mNeedToYield = true;
      if (controller->synchronization())
        return true;
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

  const int robotID = robot->uniqueId();
  const int size = mControllers.size();
  bool paused = WbSimulationState::instance()->isPaused();
  const QString &newControllerName = robot->controllerName();

  // restart the controller if needed
  for (int i = 0; i < size; ++i) {
    WbController *controller = mControllers[i];
    if (controller->robotId() == robotID) {
      disconnect(controller, &WbController::hasTerminatedByItself, this,
                 &WbControlledWorld::deleteController);  // avoids double delete
      mNewControllers.removeOne(controller);
      mWaitingControllers.removeOne(controller);
      mControllers.removeOne(controller);
      if (newControllerName.isEmpty())
        WbLog::info(tr("%1: Terminating.").arg(controller->name()));
      delete controller;
      if (newControllerName.isEmpty())
        return;
      controller = new WbController(robot);
      if (paused)  // step finished
        mWaitingControllers << controller;
      else  // step executing
        mNewControllers << controller;
      connect(controller, &WbController::hasTerminatedByItself, this, &WbControlledWorld::deleteController);
      return;
    }
  }

  if (newControllerName.isEmpty())
    return;

  // The controller has never been created. Creates a new one
  WbController *const controller = new WbController(robot);
  if (paused)  // step finished
    mWaitingControllers << controller;
  else  // step executing
    mNewControllers << controller;
  connect(controller, &WbController::hasTerminatedByItself, this, &WbControlledWorld::deleteController);
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
    controller->flushBuffers();
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
    mWaitingControllers << mNewControllers;
    mNewControllers.clear();
  }
  if (!needToWait())
    emit WbSimulationState::instance()->controllerReadRequestsCompleted();

  mIsExecutingStep = false;
  processWaitingStep();
}
