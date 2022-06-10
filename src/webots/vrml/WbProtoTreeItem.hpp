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
  WbProtoTreeItem(const QString &url, WbProtoTreeItem *parent, WbProtoTreeItem *root, bool download = true);
  ~WbProtoTreeItem();

  void recursiveRetrieval(bool value) { mFullDepth = value; }

  const QString &name() const { return mName; }
  const QString &url() const { return mUrl; }
  const QStringList &error() const { return mError; }
  const WbProtoTreeItem *parent() const { return mParent; }
  const QList<WbProtoTreeItem *> children() const { return mChildren; }

  bool isRecursiveProto(const QString &protoUrl);

  void download();
  void insert(const QString &url);  // inserts in the sub-proto list of the node its being called on

  void print(int indent = 0);
  void generateSessionProtoMap(QMap<QString, QString> &map);

signals:
  void treeUpdated();
  void finished();
  void abort();
  void downloadComplete(const QString &filename);

protected slots:
  void downloadUpdate();
  void rootUpdate();

private:
  QString mUrl;
  WbProtoTreeItem *mParent;
  bool mIsReady;  // for it to be ready, the asset must be available (on disk) and have been parsed
  WbDownloader *mDownloader;
  QString mName;       // TODO: tmp, not really needed
  QStringList mError;  // note:
  bool mFullDepth;
  WbProtoTreeItem *mRoot;

  void downloadAssets();
  bool downloadsFinished();
  void parseItem();

  void disconnectAll();
  bool isReadyToLoad();

  void failure(QString error, bool abort = true);

  QList<WbProtoTreeItem *> mChildren;  // list of referenced sub-proto
};
