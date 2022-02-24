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

  bool isCached(const QString &url) const;
  void clearCache();
  void save(const QString &url, const QByteArray &content);
  const QString get(const QString &url) const;

  qint64 cacheSize() const { return mCacheSizeInBytes; };
  void reduceCacheUsage();

private:
  static void cleanup();
  WbNetwork();
  ~WbNetwork();

  void recomputeCacheSize();
  static bool lastReadLessThan(QFileInfo &f1, QFileInfo &f2);

  // cppcheck-suppress functionStatic
  const QString urlToHash(const QString &url) const;

  QString mCacheDirectory;
  qint64 mCacheSizeInBytes;

  QNetworkAccessManager *mNetworkAccessManager;
};

#endif  // WB_NETWORK_HPP
