// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_DOWNLOADER_HPP
#define WB_DOWNLOADER_HPP

#include <QtCore/QObject>
#include <QtCore/QUrl>
#include <QtNetwork/QNetworkReply>

class WbDownloader : public QObject {
  Q_OBJECT
public:
  explicit WbDownloader(QObject *parent = NULL);
  ~WbDownloader();
  void download(const QUrl &url);
  const QUrl &url() { return mUrl; }
  QIODevice *device() { return dynamic_cast<QIODevice *>(mNetworkReply); }
  bool hasFinished() { return mFinished; }
  const QString &error() { return mError; }
  static int progress();
  static void reset();
signals:
  void complete();
  void progress(float progress);

private:
  QUrl mUrl;
  QNetworkReply *mNetworkReply;
  bool mFinished;
  QString mError;

private slots:
  void finished();
};

#endif
