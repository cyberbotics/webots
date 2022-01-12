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

#ifndef WB_ROBOT_WINDOW_STREAMING_SERVER_HPP
#define WB_ROBOT_WINDOW_STREAMING_SERVER_HPP

#include "WbStreamingServer.hpp"

class WbRobotWindowStreamingServer : public WbStreamingServer {
  Q_OBJECT

public:
  WbRobotWindowStreamingServer();
  ~WbRobotWindowStreamingServer();

private slots:
  void start(int port) override;
  void stop() override;
  void sendUpdatePackageToClients() override;
  void processTextMessage(QString) override;

private:
  void create(int port) override;
  bool prepareWorld() override;
  void sendWorldToClient(QWebSocket *client) override;
};

#endif
