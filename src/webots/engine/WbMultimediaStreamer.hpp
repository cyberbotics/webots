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

#ifndef WB_MULTIMEDIA_STREAMER_HPP
#define WB_MULTIMEDIA_STREAMER_HPP

#include <QtCore/QObject>
#include <QtCore/QProcess>
#include <QtCore/QString>
#include <QtGui/QImage>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>

// The WbMultimediaStreamer class allows to send a stream of raw images to a
// multimedia server, such as Janus for WebRTC broadcasting.

class WbMultimediaStreamer : public QObject {
  Q_OBJECT
public:
  static WbMultimediaStreamer *instance();
  bool initialize(int width, int height);
  bool start(int port);
  bool isInitialized() const { return mIsInitialized; }
  bool isReady() const { return mIsInitialized && mClients.size() > 0; };
  void *buffer() const { return mSharedMemoryData; }
  int imageWidth() const { return mImageWidth; }
  int imageHeight() const { return mImageHeight; }
  bool sendImage(QImage image);

  QString url() const { return QString("http://localhost:%1").arg(mPort); }

signals:
  void imageReady(const QByteArray &image);

private slots:
  void treatNewConnection();
  void onNewTcpData();

private:
  WbMultimediaStreamer();
  virtual ~WbMultimediaStreamer();
  bool isStreamerRunning() const;
  bool checkCommunication();
  void sendMessageToStreamer(const char *data, qint64 size);
  static WbMultimediaStreamer *sInstance;

  bool mIsInitialized;  // Remember if 'initialize()' was successfully called.
  int mPort;
  int mImageWidth;
  int mImageHeight;
  int mImageSize;
  QString mHostname;
  int mSharedMemoryKey;
  void *mSharedMemoryData;
  QTcpServer *mTcpServer;
  QList<QTcpSocket *> mClients;
  enum StreamerState { WAIT, READY, PROCESSING };
  StreamerState mStreamerState;
  QImage mSceneImage;
};

#endif  // WB_MULTIMEDIA_STREAMER_HPP
