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

#include "WbLog.hpp"
#include "WbMultimediaStreamer.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QBuffer>
#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtGui/QImage>
#include <cassert>
#include <cstdlib>
#include <iostream>

// This functionality is currently only available on Linux.
// Because all video streaming components are also available for macOS and Windows,
// it should possible to extend this functionality to other platforms whenever needed.

#ifdef __linux__
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

using namespace std;

WbMultimediaStreamer *WbMultimediaStreamer::sInstance = NULL;

#ifdef __linux__
// Init shared memory
// sharedMemoryKey: Key to identify the shared memory segment. It is common to all processes.
// sharedMemorySize: Size (in bytes) of the shared memory segment.
// sharedMemoryData: [Out] Will contain start address of shared memory segment. Undefined value if error.
static bool initSharedMemory(int &sharedMemoryKey, int sharedMemorySize, void *&sharedMemoryData) {
  int sharedMemoryId = shmget(sharedMemoryKey, sharedMemorySize, IPC_CREAT | SHM_R | SHM_W);
  if (sharedMemoryId == -1) {
    WbLog::error(QObject::tr("Streaming server failed to allocate shared memory segment."));
    return false;
  }
  sharedMemoryData = shmat(sharedMemoryId, NULL, 0);
  if (sharedMemoryData == (void *)-1) {
    WbLog::error(QObject::tr("Streaming server failed to attach shared memory segment."));
    return false;
  }
  return true;
}
#endif

WbMultimediaStreamer *WbMultimediaStreamer::instance() {
  if (!sInstance)
    sInstance = new WbMultimediaStreamer();
  return sInstance;
}

void WbMultimediaStreamer::reset() {
  delete sInstance;
  sInstance = NULL;
}

WbMultimediaStreamer::WbMultimediaStreamer() :
  mIsInitialized(false),
  mImageWidth(0),
  mImageHeight(0),
  mPort(0),
  mSharedMemoryKey(0),
  mSharedMemoryData(NULL) {
}

WbMultimediaStreamer::~WbMultimediaStreamer() {
#ifdef __linux__
  mTcpServer->close();
  if (mSharedMemoryData)
    shmdt(mSharedMemoryData);
#endif
}

bool WbMultimediaStreamer::initialize(int width, int height, const QString &stream) {
  mImageWidth = width;
  mImageHeight = height;
  mImageSize = mImageWidth * mImageHeight * 3;
  // this alternate trick is a hack to leave time for the previous shared memory segment to be deleted while we allocate a new
  // one when changing the size (resolution) of the video streaming view.
  static int alternate = 0;
  mSharedMemoryKey = 0x16aa81 + alternate + static_cast<int>(QCoreApplication::applicationPid());  // magic number
  if (alternate++ == 2)
    alternate = 0;
  if (!initSharedMemory(mSharedMemoryKey, mImageSize, mSharedMemoryData)) {
    WbLog::error(tr("Error when creating shared memory."));
    return false;
  }
  mIsInitialized = true;
  return true;
}

bool WbMultimediaStreamer::start() {
  if (!mIsInitialized)
    return false;
  mTcpServer = new QTcpServer();
  if (!mTcpServer->listen(QHostAddress::Any, 8089)) {  // TODO
    throw tr("Cannot set the server in listen mode: %1").arg(mTcpServer->errorString());
    return false;
  }
  connect(mTcpServer, &QTcpServer::newConnection, this, &WbMultimediaStreamer::treatNewConnection);
  QStringList arguments;
  arguments << QString::number(mSharedMemoryKey);
  arguments << QString::number(mImageWidth);
  arguments << QString::number(mImageHeight);
  arguments << mHostname;
  arguments << QString::number(mPort);

  WbLog::info("Webots streamer started: ");
  foreach (QString argument, arguments)
    WbLog::info(argument);
  return true;
}

bool WbMultimediaStreamer::isReady() {
  return mIsInitialized && mClients.size() > 0;
}

#include <QtCore/QDebug>
bool WbMultimediaStreamer::sendImage(QImage image) {
  mSceneImage = image;
  // emit imageReady(QByteArray((const char *)mSharedMemoryData, mImageSize));

  QByteArray im;
  QBuffer bufferJpeg(&im);
  bufferJpeg.open(QIODevice::WriteOnly);
  mSceneImage.save(&bufferJpeg, "JPG");
  QByteArray boundaryString =
    QString("--WebotsStreamingFrame\r\nContent-type: image/jpg\r\nContent-Length: %1\r\n\r\n").arg(im.length()).toUtf8();

  foreach (QTcpSocket *client, mClients) {
    if (client->state() != QAbstractSocket::ConnectedState) {
      mClients.removeAll(client);
      continue;
    }
    if (!client->isValid())
      continue;
    client->write(boundaryString);
    client->write(im);
    client->write(QByteArray("\r\n\r\n"));
    client->flush();
  }

  return true;
}

void WbMultimediaStreamer::treatNewConnection() {
  QTcpSocket *socket = mTcpServer->nextPendingConnection();
  connect(socket, &QTcpSocket::readyRead, this, &WbMultimediaStreamer::onNewTcpData);
}

void WbMultimediaStreamer::onNewTcpData() {
  qDebug() << "WbMultimediaStreamer::onNewTcpData";
  QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
  socket->readAll();  // Discard "Get Request String"

  QByteArray ContentType = ("HTTP/1.0 200 OK\r\nServer: Webots\r\nConnection: close\r\nMax-Age: 0\r\n"
                            "Expires: 0\r\nCache-Control: no-cache, private\r\nPragma: no-cache\r\n"
                            "Content-Type: multipart/x-mixed-replace; boundary=--WebotsStreamingFrame\r\n\r\n");
  socket->write(ContentType);
  mClients.append(socket);
}
