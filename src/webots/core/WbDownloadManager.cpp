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

#include "WbDownloadManager.hpp"

#include "WbDownloader.hpp"
#include "WbSimulationState.hpp"

#include <QtCore/QTimer>
#include <QtCore/QVariant>

WbDownloadManager *WbDownloadManager::cInstance = NULL;

WbDownloadManager *WbDownloadManager::instance() {
  if (!cInstance)
    cInstance = new WbDownloadManager();
  return cInstance;
}

WbDownloadManager::WbDownloadManager() : mCount(0), mComplete(0), mDownloading(false), mTimer(NULL), mDisplayPopUp(false) {
}

int WbDownloadManager::progress() const {
  return mCount == 0 ? 100 : 100 * mComplete / mCount;
}

void WbDownloadManager::reset() {
  mCount = 0;
  mComplete = 0;
  mDownloading = false;
  mDisplayPopUp = false;
}

void WbDownloadManager::abort() {
  QMapIterator<QUrl, WbDownloader *> it(mUrlCache);
  while (it.hasNext()) {
    it.next();
    it.value()->abort();
  }
}

WbDownloader *WbDownloadManager::createDownloader(const QUrl &url, QObject *parent) {
  WbSimulationState::instance()->pauseSimulation();
  WbDownloader *existingDownload = mUrlCache.value(url, NULL);
  WbDownloader *downloader = new WbDownloader(url, existingDownload, parent);
  connect(downloader, &WbDownloader::destroyed, this, &WbDownloadManager::removeDownloader);
  connect(downloader, &WbDownloader::complete, this, &WbDownloadManager::downloadCompleted);
  mCount++;

  if (!existingDownload) {
    if (!mDownloading) {
      mDownloading = true;
      mTimer = new QTimer(0);
      connect(mTimer, &QTimer::timeout, this, &WbDownloadManager::displayPopUp);
      mTimer->setInterval(1000);
      mTimer->setSingleShot(true);
      mTimer->start();
    }

    mUrlCache.insert(url, downloader);
  }

  return downloader;
}

void WbDownloadManager::removeDownloader(QObject *obj) {
  const QVariant urlProperty = obj->property("url");
  if (urlProperty.isValid() && mUrlCache.contains(urlProperty.toString()))
    mUrlCache.remove(urlProperty.toString());

  if (!obj->property("finished").toBool()) {
    mCount--;
    updateProgress();
  }
}

void WbDownloadManager::downloadCompleted() {
  mComplete++;
  updateProgress();
}

void WbDownloadManager::updateProgress() {
  if (mComplete == mCount) {
    mDownloading = false;
    mDisplayPopUp = false;
    mUrlCache.clear();
    WbSimulationState::instance()->resumeSimulation();
    mProgressUpdateCallback(progress());
  } else if (mDisplayPopUp)
    mProgressUpdateCallback(progress());
}

void WbDownloadManager::displayPopUp() {
  if (mDownloading) {
    mProgressUpdateCallback(progress());
    mDisplayPopUp = true;
  }

  if (mTimer) {
    delete mTimer;
    mTimer = NULL;
  }
}
