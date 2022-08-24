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

#ifndef WB_NETWORK_HPP
#define WB_NETWORK_HPP

#include <QtCore/QFileInfo>

class QNetworkAccessManager;

class WbNetwork {
public:
  static WbNetwork *instance();
  QNetworkAccessManager *networkAccessManager();
  void setProxy();

  // check the file in the cache and add it to the internal representation (don't use it in asserts, but everywhere else).
  bool isCachedWithMapUpdate(const QString &url);
  // check the file in the cache without adding it to the internal representation (for use in asserts, no side effect).
  bool isCachedNoMapUpdate(const QString &url) const;
  const QString &get(const QString &url);
  void clearCache();
  void save(const QString &url, const QByteArray &content);

  qint64 cacheSize() const { return mCacheSizeInBytes; };
  void reduceCacheUsage();

  const QString getUrlFromEphemeralCache(const QString &cachePath) const;

private:
  static void cleanup();
  WbNetwork();
  ~WbNetwork();

  void recomputeCacheSize();
  static bool lastReadLessThan(QFileInfo &f1, QFileInfo &f2);

  static const QString urlToHash(const QString &url);

  // mCacheMap is an ephemeral (internal) representation of what is known about the cache at every session, as such it isn't
  // persistent nor is it ever complete. Its purpose is to speed up checking and retrieving previously referenced assets.
  QMap<QString, QString> mCacheMap;

  qint64 mCacheSizeInBytes;

  QNetworkAccessManager *mNetworkAccessManager;
};

#endif  // WB_NETWORK_HPP
