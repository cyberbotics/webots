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

#ifndef WB_DOWNLOADER_HPP
#define WB_DOWNLOADER_HPP

#include <QtCore/QObject>
#include <QtCore/QUrl>

class QIODevice;
class QNetworkReply;

class WbDownloader : public QObject {
  Q_OBJECT
public:
  explicit WbDownloader(const QUrl &url, const WbDownloader *existingDownload, QObject *parent = NULL);
  ~WbDownloader();

  void download();
  void abort();

  const QUrl &url() const { return mUrl; }
  QIODevice *device() const;
  const QString &error() const { return mError; }
  bool hasFinished() const { return mFinished; }

signals:
  void complete();

protected:
  QNetworkReply *networkReply() const { return mNetworkReply; }

private:
  QUrl mUrl;
  QNetworkReply *mNetworkReply;
  const WbDownloader *mExistingDownload;
  bool mFinished;
  QString mError;
  bool mOffline;

private slots:
  void finished();
};

#endif
