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

#ifndef WB_ASSET_CACHE_HPP
#define WB_ASSET_CACHE_HPP

#include <QtCore/QObject>
#include <QtNetwork/QAbstractNetworkCache>
#include <QtNetwork/QNetworkCacheMetaData>

class WbAssetCache : public QAbstractNetworkCache {
  Q_OBJECT

public:
  explicit WbAssetCache(QObject *parent = nullptr);
  ~WbAssetCache();

  qint64 cacheSize() const;
  QIODevice *data(const QUrl &url);
  void insert(QIODevice *device);
  QNetworkCacheMetaData metaData(const QUrl &url);
  QIODevice *prepare(const QNetworkCacheMetaData &metaData);
  bool remove(const QUrl &url);
  void updateMetaData(const QNetworkCacheMetaData &metaData);

  QString cacheDirectory() const { return mCacheDirectory; };
  void setCacheDirectory(const QString &cacheDirectory);

  void setMaximumCacheSize(qint64 size);

public slots:
  void clear();

private:
  qint64 mMaximumCacheSize;
  QString mCacheDirectory;
};

#endif  // WB_ASSET_CACHE_HPP
