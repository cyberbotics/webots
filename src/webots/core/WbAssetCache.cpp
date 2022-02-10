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
#include <QtCore/QDateTime>
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
  printf("> cache size is: %lld MB\n", mCacheSizeInBytes / (1024 * 1024));

  // reduceCacheUsage(10 * 1024 * 1024);

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
        mCacheSizeInBytes += file.size();
        file.close();
      }
      printf("  cache size is: %lld MB\n", mCacheSizeInBytes / (1024 * 1024));
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

bool WbAssetCache::isCached(QString url) {
  QFileInfo fi(mCacheDirectory + urlToPath(url));
  return fi.exists();
}

const QString WbAssetCache::urlToPath(QString url) {
  return url.replace("://", "/");
}

void WbAssetCache::reduceCacheUsage(qint64 maxCacheSizeInBytes) {
  if (maxCacheSizeInBytes > mCacheSizeInBytes)
    return;  // unnecessary to purge cache items

  QFileInfoList files;

  QDirIterator it(mCacheDirectory, QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    it.next();
    files << it.fileInfo();
  }

  /*
  printf("BEFORE\n");
  QListIterator<QFileInfo> itb(list);
  while (itb.hasNext()) {
    QFileInfo fi = itb.next();
    printf(" %s: %s %lld\n", fi.lastRead().toString().toUtf8().constData(), fi.fileName().toUtf8().constData(), fi.size());
  }
  */

  std::sort(files.begin(), files.end(), lastReadLessThan);

  QListIterator<QFileInfo> i(files);
  while (i.hasNext() && mCacheSizeInBytes > maxCacheSizeInBytes) {
    const QFileInfo fi = i.next();

    QDir().remove(fi.absoluteFilePath());  // remove the file
    QDir().rmpath(fi.absolutePath());      // remove any empty parent directories

    mCacheSizeInBytes -= fi.size();
    printf(" removed %s [%lld]\n", fi.fileName().toUtf8().constData(), mCacheSizeInBytes);
  }

  /*
  printf("AFTER\n");
  QListIterator<QFileInfo> ita(list);
  while (ita.hasNext()) {
    QFileInfo fi = ita.next();
    printf(" %s: %s %lld\n", fi.lastRead().toString().toUtf8().constData(), fi.fileName().toUtf8().constData(), fi.size());
  }
  */
}

bool WbAssetCache::lastReadLessThan(QFileInfo &f1, QFileInfo &f2) {
  return f1.lastRead() < f2.lastRead();
}

void WbAssetCache::clearCache() {
  printf("> clearCache()\n");
  QDir dir(mCacheDirectory);
  if (dir.exists()) {
    dir.removeRecursively();
    // recreate directory
    dir.mkpath(".");
  }

  mCacheSizeInBytes = 0;
}

void WbAssetCache::recomputeCacheSize() {
  mCacheSizeInBytes = 0;

  QDirIterator it(mCacheDirectory, QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    it.next();
    mCacheSizeInBytes += it.fileInfo().size();
  }
}