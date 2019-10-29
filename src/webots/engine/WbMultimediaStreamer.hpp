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
#include <QtNetwork/QLocalServer>
#include <QtNetwork/QLocalSocket>

// The WbMultimediaStreamer class allows to send a stream of raw images to a
// multimedia server, such as Janus for WebRTC broadcasting.

class WbMultimediaStreamer : public QObject {
  Q_OBJECT
public:
  static WbMultimediaStreamer *instance();
  static void reset();
  bool initialize(int width, int height, const QString &stream);
  bool start();
  bool isReady();
  void *buffer() const { return mSharedMemoryData; }
  int imageWidth() const { return mImageWidth; }
  int imageHeight() const { return mImageHeight; }
  bool sendImage();

private slots:
  void treatNewConnection();

private:
  WbMultimediaStreamer();
  virtual ~WbMultimediaStreamer();
  bool isStreamerRunning() const;
  bool checkCommunication();
  void sendMessageToStreamer(const char *data, qint64 size);
  static WbMultimediaStreamer *sInstance;

  bool mIsInitialized;  // Remember if 'initialize()' was successfully called.
  int mImageWidth;
  int mImageHeight;
  QString mHostname;
  int mPort;
  QString mStreamerExecutablePath;
  QProcess mStreamer;
  int mSharedMemoryKey;
  void *mSharedMemoryData;
  QLocalServer mLocalServer;
  QLocalSocket *mLocalSocket;
  enum StreamerState { WAIT, READY, PROCESSING };
  StreamerState mStreamerState;
};

#endif  // WB_MULTIMEDIA_STREAMER_HPP
