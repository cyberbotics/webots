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

#ifndef WB_STREAMING_SERVER_HPP
#define WB_STREAMING_SERVER_HPP

#include <QtCore/QObject>

#include <QtCore/QList>

#include "WbLog.hpp"

class QTcpSocket;
class QWebSocket;
class QWebSocketServer;

class WbMainWindow;
class WbNode;
class WbRobot;
class WbStreamingTcpServer;
class WbView3D;

class WbStreamingServer : public QObject {
  Q_OBJECT

public:
  WbStreamingServer();
  virtual ~WbStreamingServer();

  void startFromCommandLine(const QString &argument);
  void setView3D(WbView3D *);
  void setMainWindow(WbMainWindow *mainWindow);

protected slots:
  void newWorld();
  virtual void deleteWorld();
  virtual void processTextMessage(QString);
  virtual void propagateNodeAddition(WbNode *node);
  virtual void sendUpdatePackageToClients();

protected:
  virtual void start(int port);
  virtual void create(int port);
  virtual void stop();
  virtual bool prepareWorld();
  virtual void connectNewRobot(const WbRobot *robot);
  virtual void sendWorldToClient(QWebSocket *client);
  virtual void sendTcpRequestReply(const QString &requestedUrl, QTcpSocket *socket) = 0;

  bool isActive() const { return mWebSocketServer != NULL; }
  void destroy();
  void resetSimulation();
  void computeEditableControllers();
  void sendToClients(const QString &message = "");
  void sendActivityPulse() const;
  void pauseClientIfNeeded(QWebSocket *client);

  QList<QWebSocket *> mWebSocketClients;
  double mPauseTimeout;

  static QString clientToId(QWebSocket *client);
  static QString simulationStateString();
  static WbMainWindow *cMainWindow;

private slots:
  void setWorldLoadingProgress(const int progress);
  void setWorldLoadingStatus(const QString &status) { mCurrentWorldLoadingStatus = status; }
  void onNewWebSocketConnection();
  void onNewTcpConnection();
  void onNewTcpData();
  void socketDisconnected();
  void propagateWebotsLogToClients(WbLog::Level level, const QString &message, bool popup);
  void propagateControllerLogToClients(WbLog::Level level, const QString &message, const QString &prefix, bool popup);
  void propagateSimulationStateChange() const;
  void sendToJavascript(const QByteArray &string);

private:
  void toggleAction(bool serverIsCreated);
  bool isControllerEditAllowed(const QString &controller);
  void sendFileToClient(QWebSocket *client, const QString &type, const QString &folder, const QString &path,
                        const QString &filename);
  void sendWorldStateToClient(QWebSocket *client, const QString &state);
  void propagateLogToClients(WbLog::Level level, const QString &message);
  bool isControllerMessageIgnored(const QString &pattern, const QString &message) const;

  QWebSocketServer *mWebSocketServer;
  WbStreamingTcpServer *mTcpServer;
  QStringList mEditableControllers;
  qint64 mLastUpdateTime;

  QString mCurrentWorldLoadingStatus;
  QString mMessageToClients;
  bool mMonitorActivity;
  bool mDisableTextStreams;
  bool mSsl;
  bool mControllerEdit;
};

#endif
