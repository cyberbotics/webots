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

class WbDownloader;
class WbProtoTreeItem;

#include <QtCore/QMap>
#include <QtCore/QObject>

class WbProtoTreeItem : public QObject {
  Q_OBJECT
public:
  WbProtoTreeItem(const QString &url, WbProtoTreeItem *parent);
  ~WbProtoTreeItem();

  const QString &name() const { return mName; }
  const QString &url() const { return mUrl; }
  const QStringList &error() const { return mError; }
  const WbProtoTreeItem *parent() const { return mParent; }
  const QList<WbProtoTreeItem *> children() const { return mChildren; }

  void setRawUrl(const QString url) { mRawUrl = url; }
  const QString &rawUrl() const { return mRawUrl; }

  void download();
  void insert(const QString &url);  // inserts in the sub-proto list of the node its being called on

  void generateSessionProtoMap(QMap<QString, QString> &map);

signals:
  void finished();

protected slots:
  void downloadUpdate();

private:
  QString mUrl;
  WbProtoTreeItem *mParent;
  bool mIsReady;
  WbDownloader *mDownloader;
  QString mName;
  QStringList mError;
  WbProtoTreeItem *mRoot;

  QString mRawUrl;  // this url is what is written on a world save

  bool isReady() const { return mIsReady; }
  void parseItem();
  void readyCheck();
  bool isRecursiveProto(const QString &protoUrl);
  void recursiveErrorAccumulator(QStringList &list);

  QList<WbProtoTreeItem *> mChildren;  // list of referenced sub-proto
};
