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

#include "WbDownloader.hpp"

#include "WbNetwork.hpp"

#include <QtCore/QDebug>
#include <QtCore/QDir>
#include <QtCore/QStandardPaths>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkRequest>

static QString gCacheLocation;

void WbDownloader::clearCache() {
  QDir(gCacheLocation).removeRecursively();
}

const QString WbDownloader::cache(const QUrl &url) {
  return gCacheLocation + "/" + url.scheme() + "/" + url.host() + url.path();
}

static void downloadComplete() {
  qDebug() << "download complete";
}

void WbDownloader::test() {
  WbDownloader *d = new WbDownloader(QUrl("https://raw.githubusercontent.com/cyberbotics/webots/R2021a/projects/appearances/"
                                          "protos/textures/bakelite_plastic/bakelite_plastic_base_color_braun.jpg"));
  connect(d, &WbDownloader::complete, &downloadComplete);
  d->start();
}

WbDownloader::WbDownloader(const QUrl &url) : mUrl(url) {
  if (gCacheLocation.isEmpty()) {  // initialize cache folder if needed
    gCacheLocation = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    QDir cache(gCacheLocation);
    if (!cache.exists())
      cache.mkpath(".");
    qDebug() << gCacheLocation;
  }
}

WbDownloader::~WbDownloader() {
}

void WbDownloader::start() {
  const QString cacheFile = cache(mUrl);
  if (QFile::exists(cacheFile)) {
    emit complete();
    return;
  }
  QNetworkRequest request;
  request.setUrl(mUrl);
  QNetworkReply *reply = WbNetwork::instance()->networkAccessManager()->get(request);
  connect(reply, &QNetworkReply::finished, this, &WbDownloader::finished, Qt::UniqueConnection);
}

void WbDownloader::finished() {
  QNetworkReply *reply = dynamic_cast<QNetworkReply *>(sender());
  assert(reply);
  if (!reply)
    return;
  if (reply->error()) {
    qDebug() << tr("Cannot download file from %1: %2").arg(mUrl.toString()).arg(reply->errorString());
    return;
  }
  disconnect(reply, &QNetworkReply::finished, this, &WbDownloader::finished);
  QFile file(cache(mUrl));
  QFileInfo(file).dir().mkpath(".");
  if (!file.open(QIODevice::WriteOnly)) {
    qDebug() << "Cannot write into cache file" << cache(mUrl);
    return;
  }
  file.write(reply->readAll());
  file.close();
  reply->deleteLater();
  emit complete();
}
