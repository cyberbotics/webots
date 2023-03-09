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

#include "WbNetwork.hpp"

#include "WbLog.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QCryptographicHash>
#include <QtCore/QDateTime>
#include <QtCore/QDirIterator>
#include <QtCore/QStandardPaths>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkProxy>

static WbNetwork *gInstance = NULL;

void WbNetwork::cleanup() {
  delete gInstance;
}

WbNetwork *WbNetwork::instance() {
  if (gInstance == NULL)
    gInstance = new WbNetwork();
  return gInstance;
}

WbNetwork::WbNetwork() {
  mNetworkAccessManager = NULL;

  // delete previous caching system folder (< R2022b)
  QDir oldCache(QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + "/network/");
  if (oldCache.exists())
    oldCache.removeRecursively();

  QDir dir(WbStandardPaths::cachedAssetsPath());
  if (!dir.exists())
    dir.mkpath(".");

  // calculate cache size
  recomputeCacheSize();
  // if necessary purge part of it to enforce maximal cache size defined in the preferences
  reduceCacheUsage();

  qAddPostRoutine(WbNetwork::cleanup);
}

WbNetwork::~WbNetwork() {
  delete mNetworkAccessManager;
  gInstance = NULL;
}

QNetworkAccessManager *WbNetwork::networkAccessManager() {
  if (mNetworkAccessManager == NULL) {
    mNetworkAccessManager = new QNetworkAccessManager();
    setProxy();
  }
  return mNetworkAccessManager;
}

void WbNetwork::setProxy() {
  QNetworkProxy proxy;
  WbPreferences *prefs = WbPreferences::instance();
  const QNetworkProxy::ProxyType type =
    (QNetworkProxy::ProxyType)prefs->value("Network/httpProxyType", QNetworkProxy::DefaultProxy).toInt();

  if (type == QNetworkProxy::DefaultProxy) {
    // QNetworkProxyFactory::systemProxyForQuery() takes between 2.5 and 15 seconds on Windows.
    // So we call it only on the first run of Webots and also if networking timeout occurs
    // during login as it may possibly be caused by a change in the system proxy configuration.
    QList<QNetworkProxy> list = QNetworkProxyFactory::systemProxyForQuery();
    if (list.size() > 0)
      proxy = list[0];
    else
      proxy = QNetworkProxy::DefaultProxy;
    prefs->setValue("Network/httpProxyType", (int)proxy.type());
    prefs->setValue("Network/httpProxyHostName", proxy.hostName());
    prefs->setValue("Network/httpProxyPort", proxy.port());
    prefs->setValue("Network/httpProxyUsername", proxy.user());
    prefs->setValue("Network/httpProxyPassword", proxy.password());
  } else if (type != QNetworkProxy::NoProxy) {
    const QString &hostName = prefs->value("Network/httpProxyHostName").toString();
    const quint16 port = prefs->value("Network/httpProxyPort").toInt();
    const QString &user = prefs->value("Network/httpProxyUsername").toString();
    const QString &password = prefs->value("Network/httpProxyPassword").toString();
    proxy = QNetworkProxy(type, hostName, port, user, password);
  }
  QNetworkProxy::setApplicationProxy(proxy);
  if (mNetworkAccessManager)
    mNetworkAccessManager->setProxy(proxy);
}

void WbNetwork::save(const QString &url, const QByteArray &content) {
  if (!isCachedWithMapUpdate(url)) {
    // save to file
    const QString path(WbStandardPaths::cachedAssetsPath() + urlToHash(url));
    QFile file(path);
    if (file.open(QIODevice::WriteOnly)) {
      file.write(content);
      mCacheSizeInBytes += file.size();
      file.close();
      // save reference in internal representation
      mCacheMap.insert(url, path);
    }
  }
}

const QString &WbNetwork::get(const QString &url) {
  if (!mCacheMap.contains(url)) {
    const QString filePath = WbStandardPaths::cachedAssetsPath() + urlToHash(url);
    mCacheMap.insert(url, filePath);
    assert(QFileInfo(filePath).exists());  // the 'get' function should not be called unless we know that the file is cached
  }
  return mCacheMap[url];
}

bool WbNetwork::isCachedWithMapUpdate(const QString &url) {
  if (mCacheMap.contains(url))  // avoid checking for file existence (and computing hash again) if asset is known to be cached
    return true;

  // if URL is not in the internal representation, check for file existence on disk
  const QString filePath = WbStandardPaths::cachedAssetsPath() + urlToHash(url);
  if (QFileInfo(filePath).exists()) {
    mCacheMap.insert(url, filePath);  // knowing it exists, keep track of it in case it gets asked again
    return true;
  }

  return false;
}

bool WbNetwork::isCachedNoMapUpdate(const QString &url) const {
  if (mCacheMap.contains(url))  // avoid checking for file existence (and computing hash again) if asset is known to be cached
    return true;

  // if URL is not in the internal representation, check for file existence on disk
  const QString filePath = WbStandardPaths::cachedAssetsPath() + urlToHash(url);
  if (QFileInfo(filePath).exists())
    return true;

  return false;
}

const QString WbNetwork::urlToHash(const QString &url) {
  return QString(QCryptographicHash::hash(url.toUtf8(), QCryptographicHash::Sha1).toHex());
}

void WbNetwork::reduceCacheUsage() {
  const qint64 maxCacheSizeInBytes = WbPreferences::instance()->value("Network/cacheSize", 1024).toInt() * 1024 * 1024;
  if (maxCacheSizeInBytes > mCacheSizeInBytes)
    return;  // unnecessary to purge any cached assets

  QFileInfoList assets;

  QDirIterator it(WbStandardPaths::cachedAssetsPath(), QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    it.next();
    assets << it.fileInfo();
  }

  // sort the files based on lastRead metadata, from oldest to newest
  std::sort(assets.begin(), assets.end(), lastReadLessThan);

  QListIterator<QFileInfo> i(assets);
  while (i.hasNext() && mCacheSizeInBytes > maxCacheSizeInBytes) {
    const QFileInfo fi = i.next();

    QDir().remove(fi.absoluteFilePath());  // remove the file from disk

    // find key (url) corresponding to path, and remove it from the internal representation
    const QString key = mCacheMap.key(fi.absoluteFilePath());
    mCacheMap.remove(key);

    mCacheSizeInBytes -= fi.size();
  }
}

bool WbNetwork::lastReadLessThan(QFileInfo &f1, QFileInfo &f2) {
  return f1.lastRead() < f2.lastRead();
}

void WbNetwork::clearCache() {
  QDir dir(WbStandardPaths::cachedAssetsPath());
  if (dir.exists()) {
    dir.removeRecursively();
    // recreate cache directory since it gets removed as well by removeRecursively
    dir.mkpath(".");
  }

  mCacheSizeInBytes = 0;
  mCacheMap.clear();
}

void WbNetwork::recomputeCacheSize() {
  mCacheSizeInBytes = 0;

  QDirIterator it(WbStandardPaths::cachedAssetsPath(), QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    it.next();
    mCacheSizeInBytes += it.fileInfo().size();
  }
}

const QString WbNetwork::getUrlFromEphemeralCache(const QString &cachePath) const {
  assert(mCacheMap.values().contains(cachePath));  // should not attempt to get the URL from the hash unless it's available
  return mCacheMap.key(cachePath);
}
