// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbAssetCache.hpp"
#include "WbPreferences.hpp"

#include <QtCore/QDir>
#include <QtCore/QStandardPaths>
#include <QtNetwork/QNetworkReply>

WbAssetCache::WbAssetCache(QObject *parent) : QAbstractNetworkCache(parent) {
  printf("> WbAssetCache()\n");
}

WbAssetCache::~WbAssetCache() {
}

void WbAssetCache::clear() {
  printf("> clear()\n");
  QDir dir(mCacheDirectory);
  if (dir.exists()) {
    dir.removeRecursively();
    // recreate directory
    dir.mkpath(".");
  }
}

qint64 WbAssetCache::cacheSize() const {
  printf("> cacheSize()\n");
  // int value = 1024 * 1024 * WbPreferences::instance()->value("Network/cacheSize", 1024).toInt();
  return 0;
}

QIODevice *WbAssetCache::data(const QUrl &url) {
  printf("> data()\nn");
}

void WbAssetCache::insert(QIODevice *device) {
  printf("> insert()\n");

  assert(device);
  QNetworkReply *d = dynamic_cast<QNetworkReply *>(device);
  if (d)
    printf("URL IS %s\n", d->url().fileName().toUtf8().constData());

  /*
  device->open(QIODevice::ReadOnly);
  assert(device->isOpen());

  QFileInfo fi(mDestination);
  file.write(device->readAll());
  device->close();
  */
}

QNetworkCacheMetaData WbAssetCache::metaData(const QUrl &url) {
  printf("> metadata()\n");
}

QIODevice *WbAssetCache::prepare(const QNetworkCacheMetaData &metaData) {
  printf("> prepare()\n");
}

bool WbAssetCache::remove(const QUrl &url) {
  printf("> remove()\n");
}

void WbAssetCache::updateMetaData(const QNetworkCacheMetaData &metaData) {
  printf("> updateMetadata()\n");
}

void WbAssetCache::setCacheDirectory(const QString &cacheDirectory) {
  printf("> setCacheDirectory()\n");

  mCacheDirectory = cacheDirectory;

  QDir dir(cacheDirectory);
  if (!dir.exists())
    dir.mkpath(".");
}

void WbAssetCache::setMaximumCacheSize(qint64 size) {
  printf("> setMaximumCacheSize()\n");

  assert(size > 0);
  mMaximumCacheSize = size;
}