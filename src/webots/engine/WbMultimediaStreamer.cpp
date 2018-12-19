// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbMultimediaStreamer.hpp"
#include "WbLog.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
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
  mSharedMemoryData(NULL),
  mLocalSocket(0),
  mStreamerState(WAIT) {
}

WbMultimediaStreamer::~WbMultimediaStreamer() {
#ifdef __linux__
  WbLog::debug("WbMultimediaStreamer: Closing...");
  sendMessageToStreamer("e", 1);  // exit
  mLocalServer.close();
  mStreamer.waitForFinished(1000);
  mStreamer.close();
  if (mSharedMemoryData)
    shmdt(mSharedMemoryData);
#endif
}

bool WbMultimediaStreamer::initialize(int width, int height, const QString &stream) {
#ifdef __linux__
  const QString streamerExecutablePath = WbStandardPaths::webotsHomePath() + "bin/streamer";
  if (!QFile::exists(streamerExecutablePath)) {
    WbLog::error(tr("No such file: '%1'.").arg(streamerExecutablePath));
    return false;
  }
  if (!(QFile::permissions(streamerExecutablePath) & (QFile::ReadUser + QFile::ExeUser))) {
    WbLog::error(tr("'%1' is not readable or executable.").arg(streamerExecutablePath));
    return false;
  }
  const QStringList streamAddress = stream.split(":");
  if (streamAddress.size() != 2) {
    WbLog::error(tr("Wrong multimedia stream: '%1'").arg(stream));
    return false;
  }
  mStreamerExecutablePath = streamerExecutablePath;
  mImageWidth = width;
  mImageHeight = height;
  mHostname = streamAddress[0];
  mPort = streamAddress[1].toInt();
  WbLog::debug(tr("Setting port to %1").arg(mPort));
  // this alternate trick is a hack to leave time for the previous shared memory segment to be deleted while we allocate a new
  // one when changing the size (resolution) of the video streaming view.
  static int alternate = 0;
  mSharedMemoryKey = 0x16aa81 + alternate + static_cast<int>(QCoreApplication::applicationPid());  // magic number
  if (alternate++ == 2)
    alternate = 0;
  if (!initSharedMemory(mSharedMemoryKey, mImageWidth * mImageHeight * 3, mSharedMemoryData)) {
    WbLog::error(tr("Error when creating shared memory."));
    return false;
  }
  mIsInitialized = true;
#endif
  return true;
}

bool WbMultimediaStreamer::start() {
  if (!mIsInitialized)
    return false;
#ifdef __linux__
  QObject::connect(&mLocalServer, &QLocalServer::newConnection, this, &WbMultimediaStreamer::treatNewConnection);
  if (!mLocalServer.listen(QString("%1").arg(time(NULL)))) {
    WbLog::error(tr("Error when opening local server."));
    return false;
  }
#endif
  QStringList arguments;
  arguments << QString::number(mSharedMemoryKey);
  arguments << QString::number(mImageWidth);
  arguments << QString::number(mImageHeight);
  arguments << mLocalServer.fullServerName();
  arguments << mHostname;
  arguments << QString::number(mPort);

  WbLog::info("Webots streamer started: ");
  foreach (QString argument, arguments)
    WbLog::info(argument);
  mStreamer.setProcessChannelMode(QProcess::ForwardedChannels);
  mStreamer.start(mStreamerExecutablePath, arguments, QIODevice::ReadWrite | QIODevice::Unbuffered);
  if (!mStreamer.waitForStarted(2000)) {
    WbLog::error(tr("The streamer process was not started successfully."));
    return false;
  }
  return true;
}

bool WbMultimediaStreamer::isReady() {
  if (!isStreamerRunning() || !checkCommunication())
    return false;
  if (mStreamerState == READY)
    return true;
  while (mLocalSocket->bytesAvailable()) {
    char ack;
    if (mLocalSocket->getChar(&ack)) {
      if (ack == 'r')
        mStreamerState = READY;
      else if (ack == 'w')
        mStreamerState = WAIT;
      else
        WbLog::error(tr("Unknown ack '%1'").arg(ack));
    }
  }
  return mStreamerState == READY;
}

bool WbMultimediaStreamer::sendImage() {
  if (!isReady())
    return false;
  mStreamerState = PROCESSING;
  sendMessageToStreamer("g", 1);  // go
  return true;
}

void WbMultimediaStreamer::treatNewConnection() {
  if (mLocalSocket && mLocalSocket->state() != QLocalSocket::ConnectedState) {
    mLocalSocket->close();
    delete mLocalSocket;
    mLocalSocket = NULL;
  }
  if (!mLocalSocket) {
    assert(mLocalServer.hasPendingConnections());
    mLocalSocket = mLocalServer.nextPendingConnection();
    WbLog::debug("Multimedia Streamer connected.");
    if (!mLocalSocket->waitForConnected(500)) {
      WbLog::error(tr("Local socket connection timed out."));
      mLocalSocket->close();
      delete mLocalSocket;
      mLocalSocket = NULL;
    }
  }
}

bool WbMultimediaStreamer::isStreamerRunning() const {
  return mStreamer.state() == QProcess::Running;
}

bool WbMultimediaStreamer::checkCommunication() {
  return (mLocalSocket && mLocalSocket->state() == QLocalSocket::ConnectedState);
}

void WbMultimediaStreamer::sendMessageToStreamer(const char *data, qint64 size) {
  if (checkCommunication()) {
    mLocalSocket->write(data, size);
    mLocalSocket->flush();
  }
}
