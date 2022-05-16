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

#include "WbProtoTreeItem.hpp"
#include "../nodes/utils/WbDownloader.hpp"
#include "../nodes/utils/WbUrl.hpp"
#include "WbNetwork.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtCore/QRegularExpression>

WbProtoTreeItem::WbProtoTreeItem(const QString &url, WbProtoTreeItem *root) :
  mUrl(url),
  mIsReady(false),
  mDownloader(NULL),
  mName(QUrl(url).fileName().replace(".proto", "")),
  mError() {
  // asd
  // if (!mParent) {
  mRoot = root ? root : this;
  printf("creating %s, is root %d\n", mName.toUtf8().constData(), this == mRoot);

  // connect(this, &WbProtoTreeItem::treeUpdated, mParent, &WbProtoTreeItem::readyToLoad);
  connect(this, &WbProtoTreeItem::treeUpdated, mRoot, &WbProtoTreeItem::refresh);
  // connect(mIsParsed, &bool ::changed, mParent, &WbProtoTreeItem::parentUpdate);
  //}
  // if the proto is locally available, parse it, otherwise download it first
  if (mRoot != this)  // download is triggered manually as mSubProto might need to be populated with non-referenced protos
    downloadAssets();
}
WbProtoTreeItem::~WbProtoTreeItem() {
  qDeleteAll(mSubProto);
  mSubProto.clear();
}

void WbProtoTreeItem::parseItem() {
  printf("parsing asset %s\n", mName.toUtf8().constData());

  QString path = mUrl;
  if (WbUrl::isWeb(path) && WbNetwork::instance()->isCached(path))
    path = WbNetwork::instance()->get(path);

  QFile file(path);
  if (file.open(QIODevice::ReadOnly)) {
    // check if the root file references external PROTO
    QRegularExpression re("EXTERNPROTO\\s+\n*\"(.*\\.proto)\"");  // TODO: test it more
    QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

    while (it.hasNext()) {
      QRegularExpressionMatch match = it.next();
      if (match.hasMatch()) {
        const QString subProto = match.captured(1);
        QString subProtoUrl = WbUrl::generateExternProtoPath(subProto, mUrl);  // TODO: should this func be moved here?
        // printf("  FROM %s AND %s\n    GEN %s\n", subProto.toUtf8().constData(), mUrl.toUtf8().constData(),
        //       subProtoUrl.toUtf8().constData());
        if (!subProtoUrl.endsWith(".proto")) {
          mError = QString(tr("Malformed extern proto url. The url should end with '.proto'\n"));
          return;
        }

        WbProtoTreeItem *child = new WbProtoTreeItem(subProtoUrl, mRoot);
        mSubProto.append(child);
      }
    }

    mIsReady = true;     // wait until mSubProto has been populated before flagging this item as ready
    emit treeUpdated();  // when we reach a dead end, notify parent about it
  } else
    mError = QString(tr("File '%1' is not readable.").arg(path));
}

void WbProtoTreeItem::downloadAssets() {
  printf("downloading assets for %s\n", mName.toUtf8().constData());
  mSubProto.clear();

  if (WbUrl::isLocalUrl(mUrl)) {
    // note: this condition should only be possible in development mode when loading an old world since, during the compilation,
    // proto-list.xml urls will be local (webots://) and will be loaded as such by the backwards compatibility mechanism;
    // under any other circumstance, the on-the-fly url manufacturing logic will convert any 'webots://' urls to remote ones
    mUrl = QDir::cleanPath(WbStandardPaths::webotsHomePath() + mUrl.mid(9));
  }

  if (WbUrl::isWeb(mUrl)) {
    if (!WbNetwork::instance()->isCached(mUrl)) {
      // printf("%35s not cached. Downloading it.\n", mName.toUtf8().constData());
      delete mDownloader;
      mDownloader = new WbDownloader(this);
      connect(mDownloader, &WbDownloader::complete, this, &WbProtoTreeItem::downloadUpdate);
      mDownloader->download(QUrl(mUrl));
      return;
    }
    // printf("%35s is cached\n", mName.toUtf8().constData());
  }

  parseItem();
}

void WbProtoTreeItem::downloadUpdate() {
  if (!mDownloader->error().isEmpty()) {
    mError = mDownloader->error();
    return;
  }

  assert(WbNetwork::instance()->isCached(mUrl));
  // printf("%35s downloded\n", mName.toUtf8().constData());
  parseItem();
}

void WbProtoTreeItem::refresh() {
  if (mRoot == this) {  // only true for the root element
    if (isReadyToLoad()) {
      disconnectAll();  // to a void multiple firings
      emit readyToLoad();
    }
  }
}

void WbProtoTreeItem::disconnectAll() {
  disconnect(this, &WbProtoTreeItem::treeUpdated, mRoot, &WbProtoTreeItem::refresh);

  foreach (WbProtoTreeItem *subProto, mSubProto)
    subProto->disconnectAll();
}

bool WbProtoTreeItem::isReadyToLoad() {
  bool isReady = mIsReady;
  foreach (WbProtoTreeItem *subProto, mSubProto)
    isReady = isReady && subProto->isReadyToLoad();

  return isReady;
}

void WbProtoTreeItem::generateProtoMap(QMap<QString, QString> &map) {
  if (!map.contains(mName) && mUrl.endsWith(".proto"))  // don't insert the root (world file typically)
    // printf("inserting <%s,%s>\n", mName.toUtf8().constData(), mUrl.toUtf8().constData());
    map.insert(mName, mUrl);

  foreach (WbProtoTreeItem *proto, mSubProto)
    proto->generateProtoMap(map);
}

void WbProtoTreeItem::insert(const QString &url) {
  WbProtoTreeItem *child = new WbProtoTreeItem(url, mRoot);
  mSubProto.append(child);
  // printf("%35s grafted\n", child->name().toUtf8().constData());
}

void WbProtoTreeItem::print(int indent) {
  QString spaces;
  for (int i = 0; i < indent; ++i)
    spaces += "  ";

  printf("%s%30s %p has %lld children (%d)\n", spaces.toUtf8().constData(), mName.toUtf8().constData(), this, mSubProto.size(),
         mIsReady);
  foreach (WbProtoTreeItem *subProto, mSubProto)
    subProto->print(indent + 1);
}