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

#ifndef WB_X3D_STREAMING_SERVER_HPP
#define WB_X3D_STREAMING_SERVER_HPP

#include "WbTcpServer.hpp"

#include <QtCore/QHash>

class WbX3dStreamingServer : public WbTcpServer {
  Q_OBJECT

public:
  WbX3dStreamingServer();
  ~WbX3dStreamingServer();

private slots:
  void propagateNodeAddition(WbNode *node) override;
  void start(int port) override;
  void stop() override;
  void sendUpdatePackageToClients() override;
  void processTextMessage(QString) override;

  void propagateNodeDeletion(WbNode *node) override;

private:
  void create(int port) override;
  void sendTcpRequestReply(const QString &url, const QString &etag, const QString &host, QTcpSocket *socket) override;
  bool prepareWorld() override;
  void deleteWorld() override;
  void sendWorldToClient(QWebSocket *client) override;

  void startX3dStreaming(QWebSocket *client);
  void generateX3dWorld();
  void sendWorldStateToClient(QWebSocket *client, const QString &state) const;

  QString mX3dWorld;
  double mX3dWorldGenerationTime;

  qint64 mLastUpdateTime;
};

#endif
