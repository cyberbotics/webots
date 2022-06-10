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
#include "WbDownloader.hpp"
#include "WbNetwork.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"

#include <QtCore/QDir>
#include <QtCore/QRegularExpression>

// static bool gAborting = false;

WbProtoTreeItem::WbProtoTreeItem(const QString &url, WbProtoTreeItem *parent, WbProtoTreeItem *root, bool download) :
  mUrl(url),
  mParent(parent),
  mIsReady(false),
  mDownloader(NULL),
  mName(QUrl(url).fileName().replace(".proto", "")),
  mError(),
  mFullDepth(true) {
  // every time an item has been downloaded and parsed, notify the root
  mRoot = root ? root : this;

  // if (this == mRoot)
  //  gAborting = false;  // reset global since a new tree is being generated

  // connect(this, &WbProtoTreeItem::treeUpdated, mRoot, &WbProtoTreeItem::rootUpdate);

  if (download)  // download might be triggered manually as mChildren might need to be populated with missing protos
    downloadAssets();
}

WbProtoTreeItem::~WbProtoTreeItem() {
  // disconnect(this, &WbProtoTreeItem::treeUpdated, mRoot, &WbProtoTreeItem::rootUpdate);
  qDeleteAll(mChildren);
  mChildren.clear();
}

void WbProtoTreeItem::parseItem() {
  QString path = mUrl;
  if (WbUrl::isWeb(path) && WbNetwork::instance()->isCached(path))
    path = WbNetwork::instance()->get(path);

  QFile file(path);
  if (file.open(QIODevice::ReadOnly)) {
    // if (!mFullDepth) {  // when full-depth is undesired, stop at the first parsing call (i.e. first level)
    //  mParent->readyCheck();
    //  // emit treeUpdated();
    //  return;
    //}

    // check if the root file references external PROTO
    QRegularExpression re("EXTERNPROTO\\s+\"(.*\\.proto)\"");  // TODO: test it more
    QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

    while (it.hasNext()) {
      QRegularExpressionMatch match = it.next();
      if (match.hasMatch()) {
        const QString subProto = match.captured(1);
        QString subProtoUrl = WbUrl::generateExternProtoPath(subProto, mUrl);  // TODO: should this func be moved here?
        printf("  FROM %s AND %s\n   GEN %s\n", subProto.toUtf8().constData(), mUrl.toUtf8().constData(),
               subProtoUrl.toUtf8().constData());

        if (!subProtoUrl.endsWith(".proto")) {
          // mRoot->failure(QString(tr("Malformed extern proto url. The url should end with '.proto'.")));
          mError << QString(tr("Malformed extern proto url. The url should end with '.proto'."));
          return;
        }

        if (isRecursiveProto(subProtoUrl)) {
          const QString subProtoName = QUrl(subProtoUrl).fileName().replace(".proto", "");
          mError << QString(tr("Recursive definition of PROTO node '%1' is not allowed.").arg(subProtoName));
          // mRoot->failure(QString(tr("Recursive definition of PROTO node '%1' is not allowed.").arg(subProtoName)), false);
          continue;
        }

        WbProtoTreeItem *child = new WbProtoTreeItem(subProtoUrl, this, mRoot, false);
        mChildren.append(child);
      }
    }

    download();
    // mIsReady = true;     // wait until mChildren has been populated before flagging this item as ready
    // emit treeUpdated();  // when we reach a dead end, notify parent about it
  } else
    // mRoot->failure(QString(tr("File '%1' is not readable.").arg(path)));
    mError << QString(tr("File '%1' is not readable.").arg(path));
}

void WbProtoTreeItem::download() {
  // if (this == mRoot) {  // manual triggers should occur only at the root level
  // trigger the download of the pre-existing children (typically those inserted by the backwards compatibility mechanism)

  if (mChildren.size() == 0) {
    mIsReady = true;
    mParent->readyCheck();
  } else {
    foreach (WbProtoTreeItem *subProto, mChildren)
      subProto->downloadAssets();
  }
  // trigger the download of the root item itself
  // downloadAssets();
  // TODO: investigate why swapping these breaks everything
  //}
}

void WbProtoTreeItem::downloadAssets() {
  // if (gAborting) {
  //  emit treeUpdated();
  //  return;
  //}
  // printf("downloading assets for %s\n", mName.toUtf8().constData());

  if (WbUrl::isLocalUrl(mUrl)) {
    // note: this condition should only be possible in development mode when loading an old world since, during the compilation,
    // proto-list.xml urls will be local (webots://) and will be loaded as such by the backwards compatibility mechanism;
    // under any other circumstance, the on-the-fly url manufacturing logic will convert any 'webots://' urls to remote ones
    mUrl = QDir::cleanPath(WbStandardPaths::webotsHomePath() + mUrl.mid(9));
  }

  if (WbUrl::isWeb(mUrl)) {
    if (!WbNetwork::instance()->isCached(mUrl)) {
      // printf("%35s not cached. Downloading it.\n", mName.toUtf8().constData());
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
  // if (gAborting) {
  //  emit treeUpdated();
  //  return;
  //}

  if (!mDownloader->error().isEmpty()) {
    mError << QString("Failure downloading EXTERNPROTO '%1': %2").arg(mName).arg(mDownloader->error());
    // mRoot->failure(QString("Failure downloading EXTERNPROTO '%1': %2").arg(mName).arg(mDownloader->error()));
    mIsReady = true;
    return;
  }

  assert(WbNetwork::instance()->isCached(mUrl));
  // printf("%35s downloded\n", mName.toUtf8().constData());
  parseItem();
}

// bool WbProtoTreeItem::downloadsFinished() {
//  bool isFinished = !mDownloader || (mDownloader && mDownloader->hasFinished());
//  foreach (WbProtoTreeItem *subProto, mChildren)
//    isFinished = isFinished && subProto->downloadsFinished();
//
//  return isFinished;
//}

// void WbProtoTreeItem::rootUpdate() {
//  if (mRoot == this) {  // only true for the root element
//    if (gAborting) {
//      if (downloadsFinished())  // wait until all pending connections are dealt with
//        emit abort();
//      return;
//    }
//
//    if (isReadyToLoad()) {
//      if (mFullDepth)
//        emit finished();
//      else
//        emit downloadComplete(mUrl);
//    }
//  }
//}

// void WbProtoTreeItem::failure(QString error, bool abort) {
//  if (abort) {
//    printf("!!!!!!! ABORTING !!!!!!!!!!: %s\n", error.toUtf8().constData());
//    gAborting = true;
//  }
//  mError << error;
//}

void WbProtoTreeItem::readyCheck() {
  int count = 0;
  foreach (WbProtoTreeItem *subProto, mChildren) {
    if (subProto->isReady())
      count++;
  }

  mIsReady = count == mChildren.size();
  if (mIsReady) {
    if (mParent)
      mParent->readyCheck();
    else  // only the root has not parent
      emit finished();
  }

  // if (this == mRoot && mIsReady) {
  //  // if (mFullDepth)
  //  emit finished();
  //  // else
  //  //  emit downloadComplete(mUrl)
  //} else
  //  mParent->readyCheck();
}

// bool WbProtoTreeItem::isReadyToLoad() {
//  bool isReady = mIsReady;
//  // printf(">> %p checking chilrend: %lld\n", this, mChildren.size());
//  foreach (WbProtoTreeItem *subProto, mChildren)
//    isReady = isReady && subProto->isReadyToLoad();
//
//  return isReady;
//}

void WbProtoTreeItem::generateSessionProtoMap(QMap<QString, QString> &map) {
  // in case of failure the tree might be incomplete, but what is inserted in the map must be known to be available
  if (mIsReady && !map.contains(mName) && mUrl.endsWith(".proto"))  // only insert protos, root file may be a world file
    map.insert(mName, mUrl);

  foreach (WbProtoTreeItem *proto, mChildren)
    proto->generateSessionProtoMap(map);
}

void WbProtoTreeItem::insert(const QString &url) {
  WbProtoTreeItem *child = new WbProtoTreeItem(url, this, mRoot, false);
  mChildren.append(child);
  // printf("%35s grafted\n", child->name().toUtf8().constData());
}

void WbProtoTreeItem::print(int indent) {
  QString spaces;
  for (int i = 0; i < indent; ++i)
    spaces += "  ";

  if (this == mRoot)
    printf("ROOT: %p %lld\n", mRoot, mRoot->mChildren.size());

  printf("%40s%s %p has %lld children (%d)\n", mName.toUtf8().constData(), spaces.toUtf8().constData(), this, mChildren.size(),
         mIsReady);
  foreach (WbProtoTreeItem *subProto, mChildren)
    subProto->print(indent + 1);
}

bool WbProtoTreeItem::isRecursiveProto(const QString &protoUrl) {
  const WbProtoTreeItem *p = mParent;
  while (p) {
    if (p->url() == protoUrl)
      return true;

    p = p->parent();
  }

  return false;
}