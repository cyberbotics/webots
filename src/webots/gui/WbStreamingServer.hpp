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

#ifndef WB_STREAMING_SERVER_HPP
#define WB_STREAMING_SERVER_HPP

#include <QtCore/QObject>

#include <QtCore/QHash>
#include <QtCore/QList>

#include "WbLog.hpp"

class QWebSocketServer;
class QWebSocket;

class WbMainWindow;
class WbNode;
class WbView3D;

class WbStreamingServer : public QObject {
  Q_OBJECT

public:
  static WbStreamingServer *instance();
  static bool instanceExists();
  static void cleanup();

  void startFromCommandLine(const QString &argument);
  void setView3D(WbView3D *);
  void setMainWindow(WbMainWindow *mainWindow);

public slots:

private slots:
  void propagateNodeAddition(WbNode *node);
  void propagateNodeDeletion(WbNode *node);
  void newWorld();
  void deleteWorld();
  void start(int port);
  void stop();
  void onNewConnection();
  void socketDisconnected();
  void processTextMessage(QString);
  void sendUpdatePackageToClients();
  void propagateWebotsLogToClients(WbLog::Level level, const QString &message, bool popup);
  void propagateControllerLogToClients(WbLog::Level level, const QString &message, const QString &prefix, bool popup);
  void propagateSimulationStateChange();
  void sendLabelUpdate(const QString &labelDescription);
  void sendToJavascript(const QByteArray &string);

private:
  WbStreamingServer();
  virtual ~WbStreamingServer();

  void create(int port);
  void destroy();

  void toggleAction(bool serverIsCreated);

  void sendToClients(const QString &message = "");
  void sendWorldToClient(QWebSocket *client);
  bool isControllerEditAllowed(const QString &controller);
  void sendFileToClient(QWebSocket *client, const QString &type, const QString &folder, const QString &path,
                        const QString &filename);
  void sendWorldStateToClient(QWebSocket *client, const QString &state);
  void sendTexturesToClient(QWebSocket *client, const QHash<QString, QString> &textures);
  void startX3domStreaming(QWebSocket *client);
  void generateX3dWorld();
  void propagateLogToClients(WbLog::Level level, const QString &message);
  bool isControllerMessageIgnored(const QString &pattern, const QString &message) const;

  QString mX3dWorld;
  QHash<QString, QString> mX3dWorldTextures;
  double mX3dWorldGenerationTime;
  QString mX3dWorldReferenceFile;
  QWebSocketServer *mServer;
  QList<QWebSocket *> mClients;
  QStringList mEditableControllers;

  qint64 mLastUpdateTime;

  QString mMessageToClients;
  bool mMonitorActivity;
  bool mDisableTextStreams;
  bool mSsl;
  bool mControllerEdit;
  QString mMultimediaServer;
  QString mMultimediaStream;
  double mPauseTimeout;
};

#endif
