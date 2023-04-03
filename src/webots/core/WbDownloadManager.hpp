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

#ifndef WB_DOWNLOADER_MANAGER_HPP
#define WB_DOWNLOADER_MANAGER_HPP

#include <QtCore/QMap>
#include <QtCore/QObject>
#include <QtCore/QUrl>

class WbDownloader;
class QTimer;
class QUrl;

class WbDownloadManager : public QObject {
  Q_OBJECT
public:
  static WbDownloadManager *instance();

  WbDownloader *createDownloader(const QUrl &url, QObject *parent = NULL);
  int progress() const;
  void reset();
  void abort();
  void displayPopUp();

  bool isCompleted() { return mCount == mComplete; }

  void setProgressUpdateCallback(void (*callback)(int)) { mProgressUpdateCallback = callback; };

private:
  static WbDownloadManager *cInstance;
  explicit WbDownloadManager();
  // cppcheck-suppress unknownMacro
  Q_DISABLE_COPY(WbDownloadManager)

  int mCount;
  int mComplete;
  bool mDownloading;
  QTimer *mTimer;
  bool mDisplayPopUp;
  QMap<QUrl, WbDownloader *> mUrlCache;

  void updateProgress();
  std::function<void(int)> mProgressUpdateCallback;

private slots:
  void downloadCompleted();
  void removeDownloader(QObject *obj);
};

#endif
