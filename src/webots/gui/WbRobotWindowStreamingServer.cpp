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

#include "WbX3dStreamingServer.hpp"

#include "WbAnimationRecorder.hpp"
#include "WbHttpReply.hpp"
#include "WbNodeOperations.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbRobot.hpp"
#include "WbSupervisorUtilities.hpp"
#include "WbTemplateManager.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtWebSockets/QWebSocket>

WbRobotWindowStreamingServer::WbRobotWindowStreamingServer() :
  WbStreamingServer(monitorActivity, disableTextStreams, ssl, controllerEdit),
  mX3dWorldGenerationTime(-1.0) {
}

WbRobotWindowStreamingServer::~WbRobotWindowStreamingServer() {
}

void WbRobotWindowStreamingServer::start(int port) {
  WbStreamingServer::start(port);
}

void WbRobotWindowStreamingServer::stop() {
  WbStreamingServer::stop();
}

void WbRobotWindowStreamingServer::create(int port) {
  WbStreamingServer::create(port);
}

void WbX3dStreamingServer::processTextMessage(QString message) {
  if (message.startsWith("start")) {
    sendToClients();  // send possible bufferized messages
  }
  WbStreamingServer::processTextMessage(message);
}

void WbX3dStreamingServer::startX3dStreaming(QWebSocket *client) {
  try {
    if (WbWorld::instance()->isModified() || mX3dWorldGenerationTime != WbSimulationState::instance()->time())
      generateX3dWorld();
    sendWorldToClient(client);
    // send the current simulation state to the newly connected client
    const QString &stateMessage = simulationStateString();
    if (!stateMessage.isEmpty())
      client->sendTextMessage(stateMessage);
  } catch (const QString &e) {
    WbLog::error(tr("Streaming server: Cannot send world date to client [%1] because: %2.").arg(clientToId(client)).arg(e));
  }
}

bool WbX3dStreamingServer::prepareWorld() {
  try {
    generateX3dWorld();
    foreach (QWebSocket *client, mWebSocketClients)
      sendWorldToClient(client);
    WbAnimationRecorder::instance()->initFromStreamingServer();
  } catch (const QString &e) {
    WbLog::error(tr("Error when reloading world: %1.").arg(e));
    destroy();
    return false;
  }

  return true;
}
