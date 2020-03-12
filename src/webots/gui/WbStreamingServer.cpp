// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "WbStreamingServer.hpp"

#include "WbApplication.hpp"
#include "WbField.hpp"
#include "WbLanguage.hpp"
#include "WbMainWindow.hpp"
#include "WbNodeOperations.hpp"
#include "WbProject.hpp"
#include "WbRobot.hpp"
#include "WbSimulationState.hpp"
#include "WbSimulationWorld.hpp"
#include "WbStandardPaths.hpp"
#include "WbStreamingTcpServer.hpp"
#include "WbSupervisorUtilities.hpp"
#include "WbTemplateManager.hpp"
#include "WbWorld.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QRegularExpression>
#include <QtNetwork/QSslKey>
#include <QtWebSockets/QWebSocket>
#include <QtWebSockets/QWebSocketServer>

WbMainWindow *WbStreamingServer::cMainWindow = NULL;

WbStreamingServer::WbStreamingServer() :
  QObject(),
  mPauseTimeout(-1),
  mWebSocketServer(NULL),
  mMonitorActivity(false),
  mDisableTextStreams(false),
  mSsl(false),
  mControllerEdit(false) {
  connect(WbApplication::instance(), &WbApplication::postWorldLoaded, this, &WbStreamingServer::newWorld);
  connect(WbApplication::instance(), &WbApplication::preWorldLoaded, this, &WbStreamingServer::deleteWorld);
  connect(WbApplication::instance(), &WbApplication::worldLoadingHasProgressed, this,
          &WbStreamingServer::setWorldLoadingProgress);
  connect(WbApplication::instance(), &WbApplication::worldLoadingStatusHasChanged, this,
          &WbStreamingServer::setWorldLoadingStatus);
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
          &WbStreamingServer::propagateSimulationStateChange);
  connect(WbNodeOperations::instance(), &WbNodeOperations::nodeAdded, this, &WbStreamingServer::propagateNodeAddition);
  connect(WbTemplateManager::instance(), &WbTemplateManager::postNodeRegeneration, this,
          &WbStreamingServer::propagateNodeAddition);
}

WbStreamingServer::~WbStreamingServer() {
  if (isActive())
    destroy();
  WbLog::info(tr("Streaming server closed"));
};

QString WbStreamingServer::clientToId(QWebSocket *client) {
  return QString::number((quintptr)client);
}

void WbStreamingServer::setMainWindow(WbMainWindow *mainWindow) {
  cMainWindow = mainWindow;
}

void WbStreamingServer::startFromCommandLine(const QString &argument) {
  // default values
  int port = 1234;
  // parse argument
  const QStringList &options = argument.split(';', QString::SkipEmptyParts);
  foreach (QString option, options) {
    option = option.trimmed();
    const QRegExp rx("(\\w+)\\s*=\\s*([A-Za-z0-9:/.\\-]+)?");
    rx.indexIn(option);
    const QStringList &capture = rx.capturedTexts();
    // "key" without value case
    if (option == "monitorActivity")
      mMonitorActivity = true;
    else if (option == "disableTextStreams")
      mDisableTextStreams = true;
    else if (option == "ssl")
      mSsl = true;
    else if (option == "controllerEdit")
      mControllerEdit = true;
    else if (capture.size() == 3) {
      const QString &key = capture[1];
      const QString &value = capture[2];
      if (key == "port") {
        bool ok;
        const int tmpPort = value.toInt(&ok);
        if (ok)
          port = tmpPort;
        else
          WbLog::error(tr("Streaming server: invalid option: port '%1'").arg(value));
      } else if (key != "mode")
        WbLog::error(tr("Streaming server: unknown option '%1'").arg(option));
    } else
      WbLog::error(tr("Streaming server: unknown option '%1'").arg(option));
  }
  start(port);
}

void WbStreamingServer::start(int port) {
  try {
    create(port);
  } catch (const QString &e) {
    WbLog::error(tr("Error when creating the TCP streaming server on port %1: %2").arg(port).arg(e));
    return;
  }
  WbLog::info(tr("Streaming server listening on port %1.").arg(port));
}

void WbStreamingServer::sendToJavascript(const QByteArray &string) {
  WbRobot *robot = dynamic_cast<WbRobot *>(sender());
  if (robot) {
    QString text = "robot:" + robot->name() + ":" + QString::fromUtf8(string);
    sendToClients(text);
  } else
    WbLog::info("WbStreamingServer::sendToJavaScript: Can't send message: " + QString::fromUtf8(string));
}

void WbStreamingServer::stop() {
  destroy();
}

void WbStreamingServer::create(int port) {
  // Create a simple HTTP server, serving:
  // - a websocket on "/"
  // - texture images on the other urls. e.g. "/textures/dir/image.[jpg|png|hdr]"

  // Reference to let live QTcpSocket and QWebSocketServer on the same port using `QWebSocketServer::handleConnection()`:
  // - https://bugreports.qt.io/browse/QTBUG-54276
  QWebSocketServer::SslMode sslMode = mSsl ? QWebSocketServer::SecureMode : QWebSocketServer::NonSecureMode;
  mWebSocketServer = new QWebSocketServer("Webots Streaming Server", sslMode, this);
  mTcpServer = new WbStreamingTcpServer();
  if (mSsl) {
    QSslConfiguration sslConfiguration;
    QFile privateKeyFile(WbStandardPaths::resourcesWebPath() + "server/ssl/privkey.pem");
    privateKeyFile.open(QIODevice::ReadOnly);
    QSslKey privateKey(&privateKeyFile, QSsl::Rsa);
    privateKeyFile.close();
    sslConfiguration.setPrivateKey(privateKey);
    QList<QSslCertificate> localCertificateChain =
      QSslCertificate::fromPath(WbStandardPaths::resourcesWebPath() + "server/ssl/cert.pem");
    sslConfiguration.setLocalCertificateChain(localCertificateChain);
    sslConfiguration.setPeerVerifyMode(QSslSocket::VerifyNone);
    mWebSocketServer->setSslConfiguration(sslConfiguration);
    mTcpServer->setSslConfiguration(sslConfiguration);
  }
  if (!mTcpServer->listen(QHostAddress::Any, port))
    throw tr("Cannot set the server in listen mode: %1").arg(mTcpServer->errorString());
  connect(mWebSocketServer, &QWebSocketServer::newConnection, this, &WbStreamingServer::onNewWebSocketConnection);
  connect(mTcpServer, &WbStreamingTcpServer::newConnection, this, &WbStreamingServer::onNewTcpConnection);
  connect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this,
          &WbStreamingServer::sendUpdatePackageToClients, Qt::UniqueConnection);
  if (!mDisableTextStreams) {
    connect(WbLog::instance(), &WbLog::controllerLogEmitted, this, &WbStreamingServer::propagateControllerLogToClients);
    connect(WbLog::instance(), &WbLog::logEmitted, this, &WbStreamingServer::propagateWebotsLogToClients);
  }
}

void WbStreamingServer::destroy() {
  // test that the animation recorder is instanciated.
  // Otherwise, the instance() call can wrongly recreate an instance of the
  // animation recorder in the cleanup routines.
  // if (WbAnimationRecorder::isInstantiated()) { // TODO fix test
  disconnect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this,
             &WbStreamingServer::sendUpdatePackageToClients);
  disconnect(WbLog::instance(), &WbLog::controllerLogEmitted, this, &WbStreamingServer::propagateControllerLogToClients);
  disconnect(WbLog::instance(), &WbLog::logEmitted, this, &WbStreamingServer::propagateWebotsLogToClients);
  //}

  if (mWebSocketServer)
    mWebSocketServer->close();

  foreach (QWebSocket *client, mWebSocketClients) {
    disconnect(client, &QWebSocket::textMessageReceived, this, &WbStreamingServer::processTextMessage);
    disconnect(client, &QWebSocket::disconnected, this, &WbStreamingServer::socketDisconnected);
  };
  qDeleteAll(mWebSocketClients);
  mWebSocketClients.clear();

  delete mWebSocketServer;
  mWebSocketServer = NULL;

  delete mTcpServer;
  mTcpServer = NULL;
}

void WbStreamingServer::onNewTcpConnection() {
  QTcpSocket *socket = mTcpServer->nextPendingConnection();
  if (socket) {
    mWebSocketServer->handleConnection(socket);
    connect(socket, &QTcpSocket::readyRead, this, &WbStreamingServer::onNewTcpData);
  }
}

void WbStreamingServer::onNewTcpData() {
  QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());

  const QString &line(socket->peek(1024));  // Peek the request header to determine the requested url.
  QStringList tokens = QString(line).split(QRegExp("[ \r\n][ \r\n]*"));
  if (tokens[0] == "GET") {
    const QString &requestedUrl(tokens[1].replace(QRegExp("^/"), ""));
    if (!requestedUrl.isEmpty())  // "/" is reserved for the websocket.
      sendTcpRequestReply(requestedUrl, socket);
  }
}

void WbStreamingServer::onNewWebSocketConnection() {
  QWebSocket *client = mWebSocketServer->nextPendingConnection();
  if (client) {
    connect(client, &QWebSocket::textMessageReceived, this, &WbStreamingServer::processTextMessage);
    connect(client, &QWebSocket::disconnected, this, &WbStreamingServer::socketDisconnected);
    mWebSocketClients << client;
    WbLog::info(
      tr("Streaming server: New client [%1] (%2 connected client(s)).").arg(clientToId(client)).arg(mWebSocketClients.size()));
    sendToClients();  // send possible bufferized messages
  }
}

void WbStreamingServer::sendFileToClient(QWebSocket *client, const QString &type, const QString &folder, const QString &path,
                                         const QString &filename) {
  QFile file(path + "/" + filename);
  if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QString content;
    int numberOfLines = 0;
    while (!file.atEnd()) {
      content += file.readLine();
      numberOfLines++;
    }
    file.close();
    WbLog::info("Sending " + folder + "/" + filename + " " + type + " to web interface (" + QString::number(numberOfLines) +
                " lines).");
    const QString answer =
      "set " + type + ":" + folder + "/" + filename + ":" + QString::number(numberOfLines) + "\n" + content;
    client->sendTextMessage(answer);
  }
}

void WbStreamingServer::processTextMessage(QString message) {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());

  if (message.startsWith("robot:")) {
    const QString &name = message.mid(6, message.indexOf(":", 6) - 6);
    const QString &data = message.mid(7 + name.size());
    const QByteArray &byteData = data.toUtf8();
    WbLog::info(tr("Streaming server: received robot message for %1: \"%2\".").arg(name).arg(data));
    const QList<WbRobot *> &robots = WbWorld::instance()->robots();
    foreach (WbRobot *const robot, robots)
      if (robot->name() == name) {
        robot->receiveFromJavascript(byteData);
        break;
      }
  } else if (message == "pause") {
    disconnect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
               &WbStreamingServer::propagateSimulationStateChange);
    WbSimulationState::instance()->setMode(WbSimulationState::PAUSE);
    connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
            &WbStreamingServer::propagateSimulationStateChange);
    printf("pause\n");
    fflush(stdout);
    client->sendTextMessage("paused by client");
  } else if (message.startsWith("real-time:") or message.startsWith("fast:")) {
    const bool realTime = message.startsWith("real-time:");
    const double timeout = realTime ? message.mid(10).toDouble() : message.mid(5).toDouble();
    if (timeout >= 0)
      mPauseTimeout = WbSimulationState::instance()->time() + timeout;
    else
      mPauseTimeout = -1.0;
    disconnect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
               &WbStreamingServer::propagateSimulationStateChange);
    if (realTime) {
      printf("real-time\n");
      WbSimulationState::instance()->setMode(WbSimulationState::REALTIME);
    } else {
      printf("fast\n");
      WbSimulationState::instance()->setMode(WbSimulationState::FAST);
    }
    connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
            &WbStreamingServer::propagateSimulationStateChange);
    fflush(stdout);
  } else if (message == "step") {
    disconnect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
               &WbStreamingServer::propagateSimulationStateChange);
    WbSimulationState::instance()->setMode(WbSimulationState::STEP);
    printf("step\n");
    fflush(stdout);
    WbSimulationWorld::instance()->step();
    WbSimulationState::instance()->setMode(WbSimulationState::PAUSE);
    connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
            &WbStreamingServer::propagateSimulationStateChange);
    printf("pause\n");
    fflush(stdout);
    client->sendTextMessage("paused by client");
  } else if (message.startsWith("timeout:")) {
    const double timeout = message.mid(8).toDouble();
    if (timeout >= 0)
      mPauseTimeout = WbSimulationState::instance()->time() + timeout;
    else
      mPauseTimeout = -1.0;
  } else if (message == "reset") {
    resetSimulation();
    sendToClients("reset finished");
  } else if (message == "revert")
    WbApplication::instance()->worldReload();
  else if (message.startsWith("load:")) {
    const QString &worldsPath = WbProject::current()->worldsPath();
    const QString &fullPath = worldsPath + '/' + message.mid(5);
    if (!QFile::exists(fullPath))
      WbLog::error(tr("Streaming server: world %1 doesn't exist.").arg(fullPath));
    else if (QDir(worldsPath) != QFileInfo(fullPath).absoluteDir())
      WbLog::error(tr("Streaming server: you are not allowed to open a world in another project directory."));
    else if (cMainWindow)
      cMainWindow->loadDifferentWorld(fullPath);
  } else if (message.startsWith("get controller:")) {
    const QString &controller = message.mid(15);
    if (!isControllerEditAllowed(controller))
      return;
    // Searches into the current projects controllers directories only
    // We look first for a perfect match of the file name (deprived of extension) with the controller directory name
    const QString &controllerDirPath = WbProject::current()->path() + "controllers/" + controller;
    const QDir &controllerDir(controllerDirPath);
    if (controllerDir.exists()) {
      // retrieve main controller filename first and other source files afterwards
      QStringList filterNames = WbLanguage::sourceFileExtensions();
      filterNames.replaceInStrings(QRegExp("^"), controller);  // prepend controller name to each item
      QStringList matchingSourceFiles = controllerDir.entryList(filterNames, QDir::Files);
      QString mainControllerFilename;
      if (!matchingSourceFiles.isEmpty()) {
        mainControllerFilename = matchingSourceFiles[0];
        sendFileToClient(client, "controller", controller, controllerDirPath, mainControllerFilename);
      }
      // send other controller files
      filterNames = WbLanguage::sourceFileExtensions() + WbLanguage::headerFileExtensions() + WbLanguage::dataFileExtensions();
      filterNames.replaceInStrings(QRegExp("^"), "*");
      matchingSourceFiles = controllerDir.entryList(filterNames, QDir::Files);
      matchingSourceFiles.removeOne(mainControllerFilename);
      foreach (QString matchingSourceFile, matchingSourceFiles) {
        if (matchingSourceFile == "runtime.ini")
          // skip runtime.ini that cannot be modified by the user
          continue;
        sendFileToClient(client, "controller", controller, controllerDirPath, matchingSourceFile);
      }
    }
  } else if (message.startsWith("sync controller:")) {
    const QString &controllerFile = message.mid(16);  // e.g. square_path/square_path.py
    const QString &controllerName = controllerFile.split("/")[0];
    if (!isControllerEditAllowed(controllerName))
      return;
    const QFileInfo &fi(WbProject::current()->path() + "controllers/" + controllerFile);
    const QString &filename = fi.fileName();
    if (fi.isFile() && fi.exists() && filename != "runtime.ini")
      sendFileToClient(client, "controller", controllerName, fi.absolutePath(), filename);
  } else if (message.startsWith("set controller:")) {
    const int s = message.indexOf('/', 15);
    const QString &controller = message.mid(15, s - 15);
    if (!isControllerEditAllowed(controller))
      return;
    const QString &filename = message.mid(s + 1, message.indexOf(':', s) - s - 1);
    if (filename == "runtime.ini")
      // it is forbidden to modify the runtime.ini file from the web interface
      return;
    if (controller.contains('.') || filename.contains("..")) {
      WbLog::error(tr("Streaming server: bad controller file name: %1/%2.").arg(controller, filename));
      return;
    }
    const QString &projectDirPath = WbProject::current()->path() + "controllers/" + controller;
    const QDir &projectDir(projectDirPath);
    if (!projectDir.exists()) {
      WbLog::error(tr("Streaming server: non-existing controller folder: %1.").arg(controller));
      return;
    }
    const QString fullFilename = projectDirPath + "/" + filename;
    QFile file(fullFilename);
    if (!file.exists()) {
      WbLog::error(tr("Streaming server: non-existing controller file: %1.").arg(fullFilename));
      return;
    }
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      WbLog::error(tr("Streaming server: cannot write controller file: %1.").arg(fullFilename));
      return;
    }
    const QStringRef content = message.midRef(message.indexOf('\n') + 1);
    file.write(content.toUtf8());
    file.close();
  } else
    WbLog::error(tr("Streaming server: Unsupported message: %1.").arg(message));
}

bool WbStreamingServer::isControllerEditAllowed(const QString &controller) {
  if (mEditableControllers.contains(controller))
    return true;

  WbLog::error(tr("Streaming server: edit of '%1' controller not allowed.").arg(controller));
  return false;
}

void WbStreamingServer::socketDisconnected() {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());
  if (client) {
    mWebSocketClients.removeAll(client);
    client->deleteLater();
    WbLog::info(tr("Streaming server: Client disconnected [%1] (remains %2 client(s)).")
                  .arg(clientToId(client))
                  .arg(mWebSocketClients.size()));
  }
}

void WbStreamingServer::sendUpdatePackageToClients() {
  sendActivityPulse();

  if (mWebSocketClients.size() > 0) {
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    if (mLastUpdateTime < 0.0 || currentTime - mLastUpdateTime >= 1000.0 / WbWorld::instance()->worldInfo()->fps()) {
      foreach (QWebSocket *client, mWebSocketClients)
        pauseClientIfNeeded(client);
      mLastUpdateTime = currentTime;
    }
  }
}

void WbStreamingServer::sendActivityPulse() const {
  if (!mMonitorActivity)
    return;

  static qint64 lastTime = 0;
  const qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
  if (currentTime - lastTime >= 5000.0) {  // send a pulse every 5 second on stdout
    lastTime = currentTime;                // to mean the simulation is up and running
    printf(".\n");
    fflush(stdout);
  }
}

void WbStreamingServer::propagateControllerLogToClients(WbLog::Level level, const QString &message, const QString &prefix,
                                                        bool popup) {
  propagateLogToClients(level, message);
}

bool WbStreamingServer::isControllerMessageIgnored(const QString &pattern, const QString &message) const {
  if (!QRegularExpression(pattern.arg(".+")).match(message).hasMatch())
    return false;

  for (int i = 0; i < mEditableControllers.size(); ++i) {
    if (QRegularExpression(pattern.arg(mEditableControllers.at(i))).match(message).hasMatch())
      return false;
  }

  return true;
}

void WbStreamingServer::propagateWebotsLogToClients(WbLog::Level level, const QString &message, bool popup) {
  if (message.startsWith("INFO: Streaming server") || level == WbLog::STATUS || level == WbLog::DEBUG)
    // do not propagate streaming server logs, status or debug messages
    return;

  // do not propagate start and exit messages coming from controllers that are not editable
  if (isControllerMessageIgnored("^INFO: %1: Starting:", message) ||
      isControllerMessageIgnored("^INFO: '%1' controller", message))
    return;

  propagateLogToClients(level == WbLog::INFO ? WbLog::STDOUT : level, message);
}

void WbStreamingServer::propagateLogToClients(WbLog::Level level, const QString &message) {
  QString result;

  if (level == WbLog::STDOUT)
    result = "stdout:";
  else
    result = "stderr:";

  result += message;
  sendToClients(result);
}

void WbStreamingServer::sendToClients(const QString &message) {
  if (mMessageToClients.isEmpty())
    mMessageToClients = message;
  else if (!message.isEmpty())
    mMessageToClients += "\n" + message;
  if (mWebSocketClients.isEmpty())
    return;
  foreach (QWebSocket *client, mWebSocketClients)
    client->sendTextMessage(mMessageToClients);
  mMessageToClients = "";
}

void WbStreamingServer::computeEditableControllers() {
  mEditableControllers.clear();
  const QList<WbRobot *> &robots = WbWorld::instance()->robots();
  foreach (WbRobot *const robot, robots)
    connectNewRobot(robot);
}

void WbStreamingServer::connectNewRobot(const WbRobot *robot) {
  connect(robot, &WbRobot::sendToJavascript, this, &WbStreamingServer::sendToJavascript);
  if (mControllerEdit) {
    const WbField *controllerField = robot->findField("controller");
    if (controllerField) {
      const QString &name = dynamic_cast<WbSFString *>(controllerField->value())->value();
      if (name != "void")
        mEditableControllers.append(name);
    }
  }
}

bool WbStreamingServer::prepareWorld() {
  try {
    foreach (QWebSocket *client, mWebSocketClients)
      sendWorldToClient(client);
  } catch (const QString &e) {
    WbLog::error(tr("Error when reloading world: %1.").arg(e));
    destroy();
    return false;
  }

  return true;
}

void WbStreamingServer::newWorld() {
  if (mWebSocketServer == NULL)
    return;

  if (mMonitorActivity) {
    printf("open\n");
    fflush(stdout);
  }

  if (!prepareWorld())
    return;
  computeEditableControllers();
}

void WbStreamingServer::deleteWorld() {
  if (mWebSocketServer == NULL)
    return;
  foreach (QWebSocket *client, mWebSocketClients)
    client->sendTextMessage("delete world");
  mEditableControllers.clear();
}

void WbStreamingServer::resetSimulation() {
  WbApplication::instance()->simulationReset(true);
  QCoreApplication::processEvents();  // this is required to make sure the simulation reset has been performed before sending
                                      // the update
  mLastUpdateTime = -1.0;
  mPauseTimeout = -1.0;
}

void WbStreamingServer::setWorldLoadingProgress(const int progress) {
  foreach (QWebSocket *client, mWebSocketClients) {
    client->sendTextMessage("loading:" + mCurrentWorldLoadingStatus + ":" + QString::number(progress));
    client->flush();
  }
}

void WbStreamingServer::propagateNodeAddition(WbNode *node) {
  if (mWebSocketServer == NULL || WbWorld::instance() == NULL)
    return;

  WbRobot *robot = dynamic_cast<WbRobot *>(node);
  if (robot)
    connectNewRobot(robot);
}

QString WbStreamingServer::simulationStateString() {
  switch (WbSimulationState::instance()->mode()) {
    case WbSimulationState::PAUSE:
      return "pause";
    case WbSimulationState::STEP:
      return "step";
    case WbSimulationState::REALTIME:
      return "real-time";
    case WbSimulationState::RUN:
      return "run";
    case WbSimulationState::FAST:
      return "fast";
    default:
      return QString();
  }
}

void WbStreamingServer::propagateSimulationStateChange() const {
  if (mWebSocketServer == NULL || WbWorld::instance() == NULL || mWebSocketClients.isEmpty())
    return;

  QString message = simulationStateString();
  if (message.isEmpty())
    return;
  if (message == "pause")
    message = QString("pause: %1").arg(WbSimulationState::instance()->time());
  foreach (QWebSocket *client, mWebSocketClients)
    client->sendTextMessage(message);
}

void WbStreamingServer::pauseClientIfNeeded(QWebSocket *client) {
  if (mPauseTimeout < 0 || WbSimulationState::instance()->time() < mPauseTimeout)
    return;

  disconnect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
             &WbStreamingServer::propagateSimulationStateChange);
  WbSimulationState::instance()->setMode(WbSimulationState::PAUSE);
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
          &WbStreamingServer::propagateSimulationStateChange);
  client->sendTextMessage(QString("pause: %1").arg(WbSimulationState::instance()->time()));
  printf("pause\n");
  fflush(stdout);
}

void WbStreamingServer::sendWorldToClient(QWebSocket *client) {
  WbWorld *world = WbWorld::instance();
  const QDir dir = QFileInfo(world->fileName()).dir();
  const QStringList worldList = dir.entryList(QStringList() << "*.wbt", QDir::Files);
  QString worlds;
  for (int i = 0; i < worldList.size(); ++i)
    worlds += (i == 0 ? "" : ";") + QFileInfo(worldList.at(i)).fileName();
  client->sendTextMessage("world:" + QFileInfo(world->fileName()).fileName() + ':' + worlds);

  QList<WbRobot *> robots = WbWorld::instance()->robots();
  foreach (const WbRobot *robot, robots) {
    if (!robot->window().isEmpty()) {
      const QString &robotName = robot->name();
      client->sendTextMessage(QString("robot window: %1:%2:%3").arg(robotName.size()).arg(robotName).arg(robot->window()));
    }
  }

  const WbWorldInfo *currentWorldInfo = WbWorld::instance()->worldInfo();
  const QString &infoWindow = currentWorldInfo->window();
  client->sendTextMessage(
    QString("world info: %1:%2:%3").arg(infoWindow.size()).arg(infoWindow).arg(currentWorldInfo->title()));

  client->sendTextMessage("scene load completed");
}
