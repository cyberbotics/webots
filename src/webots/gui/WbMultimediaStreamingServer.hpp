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

#ifndef WB_VIDEO_STREAMING_SERVER_HPP
#define WB_VIDEO_STREAMING_SERVER_HPP

#include "WbStreamingServer.hpp"

#include <QtCore/QElapsedTimer>
#include <QtGui/QImage>

class WbView3D;

class WbMultimediaStreamingServer : public WbStreamingServer {
  Q_OBJECT

public:
  WbMultimediaStreamingServer() : WbStreamingServer(), mPort(-1), mImageWidth(-1), mImageHeight(-1){};
  ~WbMultimediaStreamingServer() {}
  void sendImage(QImage image);

  void setView3D(WbView3D *view3D);
  bool isNewFrameNeeded() const;

private slots:
  void onNewTcpData() override;
  void processTextMessage(QString message);

private:
  void start(int port) override;

  int mPort;
  int mImageWidth;
  int mImageHeight;
  QImage mSceneImage;
  QList<QTcpSocket *> mTcpClients;
  QElapsedTimer mUpdateTimer;
};

#endif
