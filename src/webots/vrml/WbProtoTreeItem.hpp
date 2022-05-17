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
  WbProtoTreeItem(const QString &url, WbProtoTreeItem *root);
  ~WbProtoTreeItem();

  const QString &name() { return mName; }
  const QString &url() { return mUrl; }
  const QString &error() { return mError; }

  void insert(const QString &url);  // inserts in the sub-proto list of the node its being called on

  void downloadAssets();

  void print(int indent = 0);
  void generateProtoMap(QMap<QString, QPair<QString, int>> &map, int level = 0);

signals:
  void treeUpdated();
  void readyToLoad();

protected slots:
  void downloadUpdate();
  void rootUpdate();

private:
  QString mUrl;
  bool mIsReady;  // for it to be ready, the asset must be available (on disk) and have been parsed
  WbDownloader *mDownloader;
  QString mName;  // TODO: tmp, not really needed
  QString mError;
  WbProtoTreeItem *mRoot;

  void parseItem();

  void disconnectAll();
  bool isReadyToLoad();

  QList<WbProtoTreeItem *> mSubProto;  // list of referenced sub-proto
};
