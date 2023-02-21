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

#ifndef WB_TCP_SERVER_HPP
#define WB_TCP_SERVER_HPP

#include <QtCore/QObject>

#include <QtCore/QList>

#include "WbLog.hpp"

class QTcpSocket;
class QTcpServer;
class QWebSocket;
class QWebSocketServer;

class WbMainWindow;
class WbNode;
class WbRobot;
class WbView3D;

class WbTcpServer : public QObject {
  Q_OBJECT

public:
  explicit WbTcpServer(bool stream);
  virtual ~WbTcpServer();

  void setView3D(WbView3D *);
  void setMainWindow(WbMainWindow *mainWindow);
  virtual void start(int port);
  void sendToClients(const QString &message = "");
  void closeClient(const QString &clientID);
  bool streamStatus() { return mStream; }
  int port() { return mPort; }

signals:
  void sendRobotWindowClientID(const QString &clientID, const QString &robotName, const QString &socketStatus);

protected slots:
  void newWorld();
  virtual void deleteWorld();
  virtual void processTextMessage(QString);
  virtual void propagateNodeAddition(WbNode *node);
  virtual void sendUpdatePackageToClients();

protected:
  virtual void create(int port);
  virtual void stop();
  // cppcheck-suppress virtualCallInConstructor
  virtual bool prepareWorld();
  virtual void connectNewRobot(const WbRobot *robot);
  virtual void sendWorldToClient(QWebSocket *client);
  virtual void sendTcpRequestReply(const QString &completeUrl, const QString &etag, const QString &host, QTcpSocket *socket);
  virtual void addNewTcpController(QTcpSocket *socket);
  virtual void propagateNodeDeletion(WbNode *node);

  bool isActive() const { return mWebSocketServer != NULL; }
  void destroy();
  void resetSimulation();
  void pauseClientIfNeeded(QWebSocket *client);
  void sendRobotWindowInformation(QWebSocket *client, const WbRobot *robot, bool remove = false);

  QList<QWebSocket *> mWebSocketClients;
  double mPauseTimeout;

  static QString clientToId(QWebSocket *client);
  static QString simulationStateString(bool pauseTime = true);
  static WbMainWindow *cMainWindow;

private slots:
  void setWorldLoadingProgress(const int progress);
  void setWorldLoadingStatus(const QString &status) { mCurrentWorldLoadingStatus = status; }
  void onNewWebSocketConnection();
  void onNewTcpConnection();
  void onNewTcpData();
  void socketDisconnected();
  void propagateWebotsLogToClients(WbLog::Level level, const QString &message, bool popup);
  void propagateControllerLogToClients(WbLog::Level level, const QString &message, bool popup);
  void propagateSimulationStateChange() const;
  void sendToJavascript(const QByteArray &string);

private:
  void toggleAction(bool serverIsCreated);
  void sendFileToClient(QWebSocket *client, const QString &type, const QString &folder, const QString &path,
                        const QString &filename);
  void sendWorldStateToClient(QWebSocket *client, const QString &state);
  void propagateLogToClients(WbLog::Level level, const QString &message);
  bool isControllerMessageIgnored(const QString &pattern, const QString &message) const;

  QWebSocketServer *mWebSocketServer;
  QTcpServer *mTcpServer;
  qint64 mLastUpdateTime;

  QString mCurrentWorldLoadingStatus;
  QString mMessageToClients;
  bool mClientsReadyToReceiveMessages;
  bool mDisableTextStreams;
  bool mStream;
  bool mWorldReady;
  int mPort;
};

#endif
