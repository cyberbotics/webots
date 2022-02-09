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
#include "WbLog.hpp"
#include "WbPreferences.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QStandardPaths>

static WbAssetCache *gInstance = NULL;

void WbAssetCache::cleanup() {
  delete gInstance;
}

WbAssetCache *WbAssetCache::instance() {
  if (gInstance == NULL)
    gInstance = new WbAssetCache();
  return gInstance;
}

WbAssetCache::WbAssetCache() {
  printf("> WbAssetCache()\n");
  mCacheDirectory = QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + "/assets/";
  QDir dir(mCacheDirectory);
  if (!dir.exists())
    dir.mkpath(".");

  // calculate cache size at startup
  recomputeCacheSize();
  printf("> cache size is: %lld MB\n", mCacheSize / (1024 * 1024));

  qAddPostRoutine(WbAssetCache::cleanup);
}

WbAssetCache::~WbAssetCache() {
  gInstance = NULL;
}

void WbAssetCache::save(const QString url, const QByteArray &content) {
  printf("> save()\n");

  if (!isCached(url)) {
    // create all the necessary directories
    QFileInfo fi(mCacheDirectory + urlToPath(url));
    bool success = fi.absoluteDir().mkpath(".");
    if (success) {
      // save to file
      QFile file(fi.absoluteFilePath());
      if (file.open(QIODevice::WriteOnly)) {
        file.write(content);
        mCacheSize += file.size();
        file.close();
      }
      printf("  cache size is: %lld MB\n", mCacheSize / (1024 * 1024));
    } else
      WbLog::warning(tr("\nInvalid generated cache path for remote file: %1").arg(url), true);
  } else {
    printf("  already cached\n");
  }
}

QString WbAssetCache::get(const QString url) {
  printf("> get()\n");
  QString loc = mCacheDirectory + urlToPath(url);
  printf("  file is at: %s\n", loc.toUtf8().constData());
  return mCacheDirectory + urlToPath(url);
}
/*
QIODevice *WbAssetCache::get(const QString url) {
  assert(isCached(url));

  QFile *file = new QFile(mCacheDirectory + encodeUrl(url));
  file->open(QIODevice::ReadOnly);
  assert(file->isOpen());

  return file;
}*/

bool WbAssetCache::isCached(QString url) {
  QFileInfo fi(mCacheDirectory + urlToPath(url));
  return fi.exists();
}

const QString WbAssetCache::urlToPath(QString url) {
  return url.replace("://", "/");
}

/*
const QString WbAssetCache::pathToUrl(QString url) {
  QString decoded = url;

  int n = decoded.indexOf("/");
  QString root = decoded.left(n);
  QStringList parts = root.split(".");

  decoded.remove(0, n);

  QStringListIterator it(parts);
  while (it.hasNext())
    decoded.insert(0, "." + it.next());

  decoded.remove(0, 1);
  decoded.insert(0, "https://");

  return decoded;
}
*/

void WbAssetCache::clearCache() {
  printf("> clearCache()\n");
  QDir dir(mCacheDirectory);
  if (dir.exists()) {
    dir.removeRecursively();
    // recreate directory
    dir.mkpath(".");
  }
}

void WbAssetCache::recomputeCacheSize() {
  mCacheSize = 0;

  QDirIterator it(mCacheDirectory, QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    it.next();
    mCacheSize += it.fileInfo().size();
  }
}