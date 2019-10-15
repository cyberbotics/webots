// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbAnimationRecorder.hpp"
#include "WbApplication.hpp"
#include "WbField.hpp"
#include "WbHttpReply.hpp"
#include "WbLanguage.hpp"
#include "WbMainWindow.hpp"
#include "WbMultimediaStreamer.hpp"
#include "WbNodeOperations.hpp"
#include "WbProject.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbRobot.hpp"
#include "WbSimulationState.hpp"
#include "WbSimulationWorld.hpp"
#include "WbStandardPaths.hpp"
#include "WbStreamingTcpServer.hpp"
#include "WbSupervisorUtilities.hpp"
#include "WbTemplateManager.hpp"
#include "WbView3D.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QTemporaryFile>
#include <QtGui/QMouseEvent>
#include <QtNetwork/QSslCertificate>
#include <QtNetwork/QSslConfiguration>
#include <QtNetwork/QSslKey>
#include <QtWebSockets/QWebSocket>
#include <QtWebSockets/QWebSocketServer>

#include <cassert>

static bool gCleanedFromPostRoutine = false;
static WbStreamingServer *gInstance = NULL;
static WbView3D *gView3D = NULL;
static WbMainWindow *gMainWindow = NULL;

WbStreamingServer *WbStreamingServer::instance() {
  if (gInstance == NULL) {
    gInstance = new WbStreamingServer;
    qAddPostRoutine(WbStreamingServer::cleanup);
  }
  return gInstance;
}

bool WbStreamingServer::instanceExists() {
  return gInstance != NULL;
}

void WbStreamingServer::cleanup() {
  gCleanedFromPostRoutine = true;
  delete gInstance;
  gInstance = NULL;
}

WbStreamingServer::WbStreamingServer() :
  QObject(),
  mX3dWorldGenerationTime(-1.0),
  mWebSocketServer(NULL),
  mMonitorActivity(false),
  mDisableTextStreams(false),
  mSsl(false),
  mControllerEdit(false),
  mPauseTimeout(-1) {
  connect(WbApplication::instance(), &WbApplication::postWorldLoaded, this, &WbStreamingServer::newWorld);
  connect(WbApplication::instance(), &WbApplication::preWorldLoaded, this, &WbStreamingServer::deleteWorld);
  connect(WbApplication::instance(), &WbApplication::worldLoadingHasProgressed, this,
          &WbStreamingServer::setWorldLoadingProgress);
  connect(WbApplication::instance(), &WbApplication::worldLoadingStatusHasChanged, this,
          &WbStreamingServer::setWorldLoadingStatus);
  connect(WbNodeOperations::instance(), &WbNodeOperations::nodeAdded, this, &WbStreamingServer::propagateNodeAddition);
  connect(WbNodeOperations::instance(), &WbNodeOperations::nodeDeleted, this, &WbStreamingServer::propagateNodeDeletion);
  connect(WbTemplateManager::instance(), &WbTemplateManager::postNodeRegeneration, this,
          &WbStreamingServer::propagateNodeAddition);
  connect(WbTemplateManager::instance(), &WbTemplateManager::preNodeRegeneration, this,
          &WbStreamingServer::propagateNodeDeletion);
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
          &WbStreamingServer::propagateSimulationStateChange);
}

WbStreamingServer::~WbStreamingServer() {
  if (mWebSocketServer)
    stop();
}

static QString clientToId(QWebSocket *client) {
  return QString::number((quintptr)client);
}

void WbStreamingServer::setView3D(WbView3D *view3D) {
  gView3D = view3D;
}

void WbStreamingServer::setMainWindow(WbMainWindow *mainWindow) {
  gMainWindow = mainWindow;
}

void WbStreamingServer::startFromCommandLine(const QString &argument) {
  // default values
  int port = 1234;
  // parse argument
  QStringList options = argument.split(';', QString::SkipEmptyParts);
  foreach (QString option, options) {
    option = option.trimmed();
    QRegExp rx("(\\w+)\\s*=\\s*([A-Za-z0-9:/.\\-]+)?");
    rx.indexIn(option);
    QStringList capture = rx.capturedTexts();
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
        int tmpPort = value.toInt(&ok);
        if (ok)
          port = tmpPort;
        else
          WbLog::error(tr("Streaming server: invalid option: port '%1'").arg(value));
      } else if (key == "multimediaServer")
        mMultimediaServer = value;
      else if (key == "multimediaStream")
        mMultimediaStream = value;
      else
        WbLog::error(tr("Streaming server: unknown option '%1'").arg(option));
    } else
      WbLog::error(tr("Streaming server: unknown option '%1'").arg(option));
  }
  start(port);
}

void WbStreamingServer::start(int port) {
  if (WbWorld::instance()) {
    try {
      WbAnimationRecorder::instance()->initFromStreamingServer();
    } catch (const QString &e) {
      WbLog::error(tr("Error when initializing the animation recorder: %1").arg(e));
      return;
    }
  }
  try {
    create(port);
  } catch (const QString &e) {
    WbLog::error(tr("Error when creating the TCP streaming server: %1").arg(e));
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
  // test that the animation recorder is instanciated.
  // Otherwise, the instance() call can wrongly recreate an instance of the
  // animation recorder in the cleanup routines.
  if (WbAnimationRecorder::isInstantiated())
    WbAnimationRecorder::instance()->cleanupFromStreamingServer();
  destroy();
}

void WbStreamingServer::create(int port) {
  // Create a simple HTTP server, serving:
  // - a websocket on "/"
  // - texture images on the other urls. e.g. "/textures/dir/image.[jpg|png|hdr]"

  // Reference to let live QTcpSocket and QWebSocketServer on the same port using `QWebSocketServer::handleConnection()`:
  // - https://bugreports.qt.io/browse/QTBUG-54276

  generateX3dWorld();
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
  if (WbAnimationRecorder::isInstantiated()) {
    disconnect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this,
               &WbStreamingServer::sendUpdatePackageToClients);
    disconnect(WbLog::instance(), &WbLog::controllerLogEmitted, this, &WbStreamingServer::propagateControllerLogToClients);
    disconnect(WbLog::instance(), &WbLog::logEmitted, this, &WbStreamingServer::propagateWebotsLogToClients);
  }

  if (mWebSocketServer)
    mWebSocketServer->close();

  foreach (QWebSocket *client, mClients) {
    disconnect(client, &QWebSocket::textMessageReceived, this, &WbStreamingServer::processTextMessage);
    disconnect(client, &QWebSocket::disconnected, this, &WbStreamingServer::socketDisconnected);
  };
  qDeleteAll(mClients);
  mClients.clear();

  delete mWebSocketServer;
  mWebSocketServer = NULL;

  delete mTcpServer;
  mTcpServer = NULL;

  if (gCleanedFromPostRoutine == false)
    // calling WbLog from the post routine can crash Webots if WbLog is deleted before
    WbLog::info(tr("Streaming server closed"));
}

void WbStreamingServer::onNewTcpConnection() {
  QTcpSocket *socket = mTcpServer->nextPendingConnection();
  mWebSocketServer->handleConnection(socket);
  connect(socket, &QTcpSocket::readyRead, this, &WbStreamingServer::onNewTcpData);
}

void WbStreamingServer::onNewTcpData() {
  QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());

  QString line(socket->peek(1024));  // Peek the request header to determine the requested url.
  QStringList tokens = QString(line).split(QRegExp("[ \r\n][ \r\n]*"));
  if (tokens[0] == "GET") {
    QString requestedUrl(tokens[1].replace(QRegExp("^/"), ""));
    if (!requestedUrl.isEmpty()) {  // "/" is reserved for the websocket.
      QByteArray reply;
      if (mX3dWorldTextures.contains(requestedUrl))
        reply = WbHttpReply::forgeImageReply(mX3dWorldTextures[requestedUrl]);
      else
        reply = WbHttpReply::forge404Reply();
      socket->write(reply);
      socket->disconnectFromHost();
    }
  }
}

void WbStreamingServer::onNewWebSocketConnection() {
  QWebSocket *client = mWebSocketServer->nextPendingConnection();
  if (client) {
    connect(client, &QWebSocket::textMessageReceived, this, &WbStreamingServer::processTextMessage);
    connect(client, &QWebSocket::disconnected, this, &WbStreamingServer::socketDisconnected);
    mClients << client;
    WbLog::info(tr("Streaming server: New client [%1] (%2 connected client(s))").arg(clientToId(client)).arg(mClients.size()));
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

bool WbStreamingServer::isControllerEditAllowed(const QString &controller) {
  if (mEditableControllers.contains(controller))
    return true;

  WbLog::error(tr("Streaming server: edit of '%1' controller not allowed.").arg(controller));
  return false;
}

void WbStreamingServer::processTextMessage(QString message) {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());

  if (message.startsWith("mouse")) {
    int action, button, buttons, x, y, modifiers, wheel;
    QString skip;  // will receive "mouse"
    QTextStream(&message) >> skip >> action >> button >> buttons >> x >> y >> modifiers >> wheel;
    const QPointF point(x, y);
    Qt::MouseButtons buttonsPressed = ((buttons & 1) ? Qt::LeftButton : Qt::NoButton) |
                                      ((buttons & 2) ? Qt::RightButton : Qt::NoButton) |
                                      ((buttons & 4) ? Qt::MiddleButton : Qt::NoButton);
    Qt::KeyboardModifiers keyboardModifiers = ((modifiers & 1) ? Qt::ShiftModifier : Qt::NoModifier) |
                                              ((modifiers & 2) ? Qt::ControlModifier : Qt::NoModifier) |
                                              ((modifiers & 4) ? Qt::AltModifier : Qt::NoModifier);
    if (action <= 1) {
      QInputEvent::Type type;
      Qt::MouseButton buttonPressed;
      if (action == 0) {
        type = QEvent::MouseMove;
        buttonPressed = Qt::NoButton;
      } else {
        switch (button) {
          case 1:
            buttonPressed = Qt::LeftButton;
            break;
          case 2:
            buttonPressed = Qt::RightButton;
            break;
          case 3:
            buttonPressed = Qt::MiddleButton;
            break;
          default:
            buttonPressed = Qt::NoButton;
            break;
        }
        if (action == -1)
          type = QEvent::MouseButtonPress;
        else if (action == 1)
          type = QEvent::MouseButtonRelease;
        else
          type = QEvent::MouseMove;
      }
      QMouseEvent event(type, point, buttonPressed, buttonsPressed, keyboardModifiers);
      if (gView3D)
        gView3D->remoteMouseEvent(&event);
    } else if (action == 2) {
      wheel = -wheel;  // Wheel delta is inverted in JS and Webots
      QWheelEvent wheelEvent(point, point, QPoint(), QPoint(), wheel, Qt::Vertical, buttonsPressed, keyboardModifiers);
      if (gView3D)
        gView3D->remoteWheelEvent(&wheelEvent);
    }
  } else if (message.startsWith("robot:")) {
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
    // reset nodes visibility
    foreach (WbBaseNode *node, WbWorld::instance()->viewpoint()->getInvisibleNodes())
      client->sendTextMessage(QString("visibility:%1:1").arg(node->uniqueId()));
    // reset the simulation
    WbApplication::instance()->simulationReset();
    QCoreApplication::processEvents();  // this is required to make sure the simulation reset has been performed before sending
                                        // the update
    mLastUpdateTime = -1.0;
    mPauseTimeout = -1.0;
    // send update
    QString state = WbAnimationRecorder::instance()->computeUpdateData(true);
    if (!state.isEmpty()) {
      foreach (QWebSocket *client, mClients)
        sendWorldStateToClient(client, state);
    }
    sendToClients("reset finished");
  } else if (message == "revert")
    WbApplication::instance()->worldReload();
  else if (message.startsWith("load:")) {
    const QString worldsPath = WbProject::current()->worldsPath();
    const QString fullPath = worldsPath + '/' + message.mid(5);
    if (!QFile::exists(fullPath))
      WbLog::error(tr("Streaming server: world %1 doesn't exist.").arg(fullPath));
    else if (QDir(worldsPath) != QFileInfo(fullPath).absoluteDir())
      WbLog::error(tr("Streaming server: you are not allowed to open a world in another project directory."));
    else if (gMainWindow)
      gMainWindow->loadDifferentWorld(fullPath);
  } else if (message.startsWith("get controller:")) {
    const QString controller = message.mid(15);
    if (!isControllerEditAllowed(controller))
      return;
    // Searches into the current projects controllers directories only
    // We look first for a perfect match of the file name (deprived of extension) with the controller directory name
    const QString controllerDirPath = WbProject::current()->path() + "controllers/" + controller;
    const QDir controllerDir(controllerDirPath);
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
    const QString controllerFile = message.mid(16);  // e.g. square_path/square_path.py
    const QString controllerName = controllerFile.split("/")[0];
    if (!isControllerEditAllowed(controllerName))
      return;
    QFileInfo fi(WbProject::current()->path() + "controllers/" + controllerFile);
    const QString &filename = fi.fileName();
    if (fi.isFile() && fi.exists() && filename != "runtime.ini")
      sendFileToClient(client, "controller", controllerName, fi.absolutePath(), filename);
  } else if (message.startsWith("set controller:")) {
    const int s = message.indexOf('/', 15);
    const QString controller = message.mid(15, s - 15);
    if (!isControllerEditAllowed(controller))
      return;
    const QString filename = message.mid(s + 1, message.indexOf(':', s) - s - 1);
    if (filename == "runtime.ini")
      // it is forbidden to modify the runtime.ini file from the web interface
      return;
    if (controller.contains('.') || filename.contains("..")) {
      WbLog::error(tr("Streaming server: bad controller file name: %1/%2.").arg(controller, filename));
      return;
    }
    const QString &projectDirPath = WbProject::current()->path() + "controllers/" + controller;
    const QDir projectDir(projectDirPath);
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
  } else if (message.startsWith("x3d")) {
    WbLog::info(tr("Streaming server: Client set mode to: X3D."));
    mPauseTimeout = message.endsWith(";broadcast") ? -1 : 0;

    if (!WbWorld::instance()->isLoading())
      startX3dStreaming(client);
    // else streaming is started once the world loading is completed
  } else if (message.startsWith("video: ")) {
    QStringList resolution = message.mid(7).split("x");
    int width = resolution[0].toInt();
    int height = resolution[1].toInt();
    WbLog::info(tr("Streaming server: Client set mode to: VIDEO %1x%2").arg(width).arg(height));
    gMainWindow->setView3DSize(QSize(width, height));
    WbMultimediaStreamer::instance()->initialize(width, height, mMultimediaStream);
    WbMultimediaStreamer::instance()->start();
    client->sendTextMessage("video: " + mMultimediaServer + " 1");  // 1 is the stream id of the main Webots view
    // this should be fixed when we want to support multiples instance of Webots running on the same multimedia streaming server
  } else if (message.startsWith("resize: ")) {
    QStringList resolution = message.mid(8).split("x");
    int width = resolution[0].toInt();
    int height = resolution[1].toInt();
    WbLog::info(tr("Streaming server: Client resize: VIDEO %1x%2").arg(width).arg(height));
    gMainWindow->setView3DSize(QSize(width, height));
    WbMultimediaStreamer::reset();
    WbMultimediaStreamer::instance()->initialize(width, height, mMultimediaStream);
    WbMultimediaStreamer::instance()->start();
  } else
    WbLog::error(tr("Streaming server: Unsupported message: %1.").arg(message));
}

void WbStreamingServer::startX3dStreaming(QWebSocket *client) {
  try {
    if (WbWorld::instance()->isModified() || mX3dWorldGenerationTime != WbSimulationState::instance()->time())
      generateX3dWorld();
    sendWorldToClient(client);
    // send the current simulation state to the newly connected client
    const QString &stateMessage = simulationStateString();
    if (!stateMessage.isEmpty())
      client->sendTextMessage(stateMessage);
    WbLog::info(tr("Streaming server: New client [%1] (%2 connected client(s)).").arg(clientToId(client)).arg(mClients.size()));
  } catch (const QString &e) {
    WbLog::error(tr("Streaming server: Cannot send world date to client [%1] because: %2.").arg(clientToId(client)).arg(e));
  }
}

void WbStreamingServer::socketDisconnected() {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());
  if (client) {
    mClients.removeAll(client);
    client->deleteLater();
    WbLog::info(
      tr("Streaming server: Client disconnected [%1] (remains %2 client(s)).").arg(clientToId(client)).arg(mClients.size()));
  }
}

void WbStreamingServer::sendUpdatePackageToClients() {
  if (mMonitorActivity) {
    static qint64 lastTime = 0;
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    if (currentTime - lastTime >= 5000.0) {  // send a pulse every 5 second on stdout
      lastTime = currentTime;                // to mean the simulation is up and running
      printf(".\n");
      fflush(stdout);
    }
  }
  if (mClients.size() > 0) {
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    if (mLastUpdateTime < 0.0 || currentTime - mLastUpdateTime >= 1000.0 / WbWorld::instance()->worldInfo()->fps()) {
      QString state = WbAnimationRecorder::instance()->computeUpdateData(false);
      if (!state.isEmpty()) {
        foreach (QWebSocket *client, mClients) {
          sendWorldStateToClient(client, state);
          if (mPauseTimeout >= 0 && WbSimulationState::instance()->time() >= mPauseTimeout) {
            disconnect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
                       &WbStreamingServer::propagateSimulationStateChange);
            WbSimulationState::instance()->setMode(WbSimulationState::PAUSE);
            connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this,
                    &WbStreamingServer::propagateSimulationStateChange);
            client->sendTextMessage("pause");
            printf("pause\n");
            fflush(stdout);
          }
        }
      }
      mLastUpdateTime = currentTime;
    }
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
  if (mClients.isEmpty())
    return;
  foreach (QWebSocket *client, mClients)
    client->sendTextMessage(mMessageToClients);
  mMessageToClients = "";
}

void WbStreamingServer::newWorld() {
  if (mWebSocketServer == NULL)
    return;

  if (mMonitorActivity) {
    printf("open\n");
    fflush(stdout);
  }

  try {
    bool regenerationRequired = mX3dWorldReferenceFile != WbWorld::instance()->fileName() || mX3dWorldGenerationTime != 0.0;
    if (!regenerationRequired) {
      // if a non static procedural PROTO is used we need to regenerate the world anyway
      QList<WbProtoModel *> models = WbProtoList::current()->models();
      for (int i = 0; i < models.size(); ++i) {
        if (models.at(i)->isTemplate() && !models.at(i)->isStatic()) {
          regenerationRequired = true;
          break;
        }
      }
    }
    if (regenerationRequired)
      generateX3dWorld();
    foreach (QWebSocket *client, mClients)
      sendWorldToClient(client);
    WbAnimationRecorder::instance()->initFromStreamingServer();
  } catch (const QString &e) {
    WbLog::error(tr("Error when reloading world: %1.").arg(e));
    destroy();
    return;
  }
  mEditableControllers.clear();
  const QList<WbRobot *> &robots = WbWorld::instance()->robots();
  foreach (WbRobot *const robot, robots) {
    connect(robot, &WbRobot::sendToJavascript, this, &WbStreamingServer::sendToJavascript);
    if (mControllerEdit) {
      const WbField *controllerField = robot->findField("controller");
      if (controllerField) {
        const QString &name = dynamic_cast<WbSFString *>(controllerField->value())->value();
        if (name != "void")
          mEditableControllers.append(name);
      }
    }
    if (robot->supervisor())
      connect(robot->supervisorUtilities(), &WbSupervisorUtilities::labelChanged, this, &WbStreamingServer::sendLabelUpdate);
  }
}

void WbStreamingServer::deleteWorld() {
  if (mWebSocketServer == NULL)
    return;
  WbAnimationRecorder::instance()->cleanupFromStreamingServer();
  foreach (QWebSocket *client, mClients)
    client->sendTextMessage("model:");  // send an empty model to destroy the player world
  mEditableControllers.clear();
}

void WbStreamingServer::setWorldLoadingProgress(const int progress) {
  foreach (QWebSocket *client, mClients) {
    client->sendTextMessage("loading:" + mCurrentWorldLoadingStatus + ":" + QString::number(progress));
    client->flush();
  }
}

void WbStreamingServer::propagateNodeAddition(WbNode *node) {
  if (mWebSocketServer == NULL || WbWorld::instance() == NULL)
    return;

  WbBaseNode *baseNode = static_cast<WbBaseNode *>(node);
  if (baseNode && baseNode->isInBoundingObject())
    return;

  WbRobot *robot = dynamic_cast<WbRobot *>(node);
  if (robot) {
    connect(robot, &WbRobot::sendToJavascript, this, &WbStreamingServer::sendToJavascript);
    if (mControllerEdit) {
      const WbField *controllerField = robot->findField("controller");
      if (controllerField) {
        const QString &name = dynamic_cast<WbSFString *>(controllerField->value())->value();
        if (name != "void")
          mEditableControllers.append(name);
      }
    }
    if (robot->supervisor())
      connect(robot->supervisorUtilities(), &WbSupervisorUtilities::labelChanged, this, &WbStreamingServer::sendLabelUpdate);
  }
  if (!mClients.isEmpty()) {
    QString nodeString;
    WbVrmlWriter writer(&nodeString, node->modelName() + ".x3d");
    node->write(writer);
    foreach (QWebSocket *client, mClients)
      // add root <nodes> element to handle correctly multiple root elements like in case of PBRAppearance node.
      client->sendTextMessage(QString("node:%1:<nodes>%2</nodes>").arg(node->parent()->uniqueId()).arg(nodeString));
  }
}

void WbStreamingServer::propagateNodeDeletion(WbNode *node) {
  if (mWebSocketServer == NULL || WbWorld::instance() == NULL)
    return;

  foreach (QWebSocket *client, mClients)
    client->sendTextMessage(QString("delete:%1").arg(node->uniqueId()));
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

void WbStreamingServer::propagateSimulationStateChange() {
  if (mWebSocketServer == NULL || WbWorld::instance() == NULL || mClients.isEmpty())
    return;

  const QString &message = simulationStateString();
  if (message.isEmpty())
    return;
  foreach (QWebSocket *client, mClients)
    client->sendTextMessage(message);
}

void WbStreamingServer::generateX3dWorld() {
  WbWorld *world = WbWorld::instance();
  if (!world)
    return;

  QString worldString;
  WbVrmlWriter writer(&worldString, QFileInfo(world->fileName()).baseName() + ".x3d");
  world->write(writer);
  mX3dWorld = worldString;
  mX3dWorldTextures = writer.texturesList();
  mX3dWorldGenerationTime = WbSimulationState::instance()->time();
  mX3dWorldReferenceFile = WbWorld::instance()->fileName();
  mLastUpdateTime = -1.0;
}

void WbStreamingServer::sendWorldToClient(QWebSocket *client) {
  WbWorld *world = WbWorld::instance();
  const QDir dir = QFileInfo(world->fileName()).dir();
  const QStringList worldList = dir.entryList(QStringList() << "*.wbt", QDir::Files);
  QString worlds;
  for (int i = 0; i < worldList.size(); ++i)
    worlds += (i == 0 ? "" : ";") + QFileInfo(worldList.at(i)).fileName();
  client->sendTextMessage("world:" + QFileInfo(world->fileName()).fileName() + ':' + worlds);

  qint64 ret = client->sendTextMessage(QString("model:") + mX3dWorld);
  if (ret < mX3dWorld.size())
    throw tr("Cannot sent the entire world");

  QString state = WbAnimationRecorder::instance()->computeUpdateData(true);
  if (!state.isEmpty())
    sendWorldStateToClient(client, state);

  QList<WbRobot *> robots = WbWorld::instance()->robots();
  foreach (const WbRobot *robot, robots) {
    if (robot->supervisor()) {
      foreach (const QString &label, robot->supervisorUtilities()->labelsState())
        client->sendTextMessage(label);
    }
  }

  client->sendTextMessage("scene load completed");
}

void WbStreamingServer::sendWorldStateToClient(QWebSocket *client, const QString &state) {
  client->sendTextMessage(QString("application/json:") + state);
}

void WbStreamingServer::sendLabelUpdate(const QString &labelDescription) {
  sendToClients(labelDescription);
}
