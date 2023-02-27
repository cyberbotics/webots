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

#ifndef WB_MULTIMEDIA_STREAMING_SERVER_HPP
#define WB_MULTIMEDIA_STREAMING_SERVER_HPP

#include "WbTcpServer.hpp"
#include "WbVector3.hpp"

#include <QtCore/QElapsedTimer>
#include <QtCore/QTimer>

class WbMatter;
class WbMultimediaStreamingLimiter;
class WbView3D;

class WbMultimediaStreamingServer : public WbTcpServer {
  Q_OBJECT

public:
  WbMultimediaStreamingServer();
  ~WbMultimediaStreamingServer();
  void sendImage(const QImage &image);

  void setView3D(WbView3D *view3D);
  bool isNewFrameNeeded() const;

signals:
  void imageRequested();

private slots:
  void removeTcpClient();
  void processTextMessage(QString message) override;
  void sendImageOnTimeout();
  void processLimiterTimeout();
  void sendWorldToClient(QWebSocket *client) override;

private:
  void start(int port) override;
  void sendTcpRequestReply(const QString &requestedUrl, const QString &etag, const QString &host, QTcpSocket *socket) override;
  int bytesToWrite();
  void sendContextMenuInfo(const WbMatter *node);
  void sendLastImage(QTcpSocket *client = NULL);
  void updateStreamingParameters(int skippedImagesCount);

  int mImageWidth;
  int mImageHeight;
  int mImageUpdateTimeStep;

  QByteArray mSceneImage;
  QList<QTcpSocket *> mTcpClients;
  QElapsedTimer mUpdateTimer;
  QTimer mWriteTimer;

  WbMultimediaStreamingLimiter *mLimiter;
  QTimer mLimiterTimer;
  int mAverageBytesToWrite;
  int mSentImagesCount;
  // flag keeping track of the current status
  //   0: none
  //   1: scene changed since full resolution image was sent
  //   2: full resolution image just sent
  int mFullResolutionOnPause;
  int mBlockedResolutionFactor;

  double mLastSpeedIndicatorTime;
  WbVector3 mTouchEventRotationCenter;
  bool mTouchEventObjectPicked;
  double mTouchEventZoomScale;
};

#endif
