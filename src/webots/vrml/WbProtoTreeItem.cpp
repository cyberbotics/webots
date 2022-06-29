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

WbProtoTreeItem::WbProtoTreeItem(const QString &url, WbProtoTreeItem *parent) :
  mUrl(url),
  mParent(parent),
  mIsReady(false),
  mDownloader(NULL),
  mName(QUrl(url).fileName().replace(".proto", "")),
  mError() {
}

WbProtoTreeItem::~WbProtoTreeItem() {
  qDeleteAll(mChildren);
  mChildren.clear();
}

void WbProtoTreeItem::parseItem() {
  QString path = mUrl;
  if (WbUrl::isWeb(path) && WbNetwork::instance()->isCached(path))
    path = WbNetwork::instance()->get(path);

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    mError << QString(tr("File '%1' is not readable.").arg(path));
    if (mParent) {
      mIsReady = true;  // reached the end of a branch, notify the parent about it
      mParent->readyCheck();
    }
    return;
  }

  // check if the root file references external PROTO
  QRegularExpression re("EXTERNPROTO\\s+\"(.*\\.proto)\"");
  QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

  // begin by populating the list of all sub-PROTO
  while (it.hasNext()) {
    QRegularExpressionMatch match = it.next();
    if (match.hasMatch()) {
      const QString subProto = match.captured(1);
      const QString subProtoUrl = WbUrl::generateExternProtoPath(subProto, mUrl);

      if (!subProtoUrl.endsWith(".proto")) {
        mError << QString(tr("Malformed EXTERNPROTO url. The url should end with '.proto'."));
        continue;
      }

      // sanity check (must either be: relative, absolute, starts with webots://, starts with https://)
      if (!subProtoUrl.startsWith("https://") && !subProtoUrl.startsWith("webots://") && !QFileInfo(subProtoUrl).isRelative() &&
          !QFileInfo(subProtoUrl).isAbsolute()) {
        mError << QString(tr("Malformed EXTERNPROTO url. Invalid url provided: %1.").arg(subProtoUrl));
        continue;
      }

      // ensure there's no ambiguity between the declarations
      const QString subProtoName = QUrl(subProtoUrl).fileName().replace(".proto", "");
      foreach (const WbProtoTreeItem *child, mChildren) {
        if (child->name() == subProtoName && child->url() != subProtoUrl) {
          mError << QString(tr("PROTO '%1' is ambiguous, multiple references are provided: '%2' and '%3'. The first was used.")
                              .arg(subProtoName)
                              .arg(child->url())
                              .arg(subProtoUrl));
          continue;
        }
      }

      if (isRecursiveProto(subProtoUrl))
        continue;  // prevent endless downloads, the error itself is handled elsewhere

      // skip local sub-PROTO that don't actually exist on disk
      if (!WbUrl::isWeb(subProtoUrl) && !QFileInfo(subProtoUrl).exists()) {
        mError << QString(tr("Skipped PROTO '%1' as it is not available at: %2.").arg(subProtoName).arg(subProtoUrl));
        continue;
      }

      WbProtoTreeItem *child = new WbProtoTreeItem(subProtoUrl, this);
      child->setRawUrl(subProto);  // if requested to save to file, save it as it was loaded (i.e. without url manipulations)
      mChildren.append(child);
    }
  }

  // only when the list is complete trigger their download (to avoid racing conditions)
  if (mChildren.size() == 0) {
    if (mParent) {
      mIsReady = true;  // reached the end of a branch, notify the parent about it
      mParent->readyCheck();
    } else
      readyCheck();
  } else {
    foreach (WbProtoTreeItem *subProto, mChildren)
      subProto->download();
  }
}

void WbProtoTreeItem::download() {
  if (WbUrl::isLocalUrl(mUrl)) {
    // note: this condition should only be possible in development mode when loading an old world since, during the
    // compilation, proto-list.xml urls will be local (webots://) and will be loaded as such by the backwards compatibility
    // mechanism; under any other circumstance, the on-the-fly url manufacturing logic will convert any 'webots://' urls to
    // remote ones
    mUrl = QDir::cleanPath(WbStandardPaths::webotsHomePath() + mUrl.mid(9));
  }

  if (WbUrl::isWeb(mUrl)) {
    if (!WbNetwork::instance()->isCached(mUrl)) {
      mDownloader = new WbDownloader(this);
      connect(mDownloader, &WbDownloader::complete, this, &WbProtoTreeItem::downloadUpdate);
      mDownloader->download(QUrl(mUrl));
      return;
    }
  }

  parseItem();
}

void WbProtoTreeItem::downloadUpdate() {
  if (!mDownloader->error().isEmpty()) {
    mError << QString("Failure downloading EXTERNPROTO '%1': %2").arg(mName).arg(mDownloader->error());
    mIsReady = true;
    mParent->readyCheck();
    return;
  }

  assert(WbNetwork::instance()->isCached(mUrl));
  parseItem();
}

void WbProtoTreeItem::readyCheck() {
  mIsReady = true;
  foreach (WbProtoTreeItem *subProto, mChildren)
    mIsReady = mIsReady && subProto->isReady();

  if (mIsReady) {
    if (mParent)
      mParent->readyCheck();
    else {  // only the root has not parent
      // assemble all the errors in the root's variable
      recursiveErrorAccumulator(mError);
      mError.removeDuplicates();
      // notify load can begin
      emit finished();
    }
  }
}

void WbProtoTreeItem::recursiveErrorAccumulator(QStringList &list) {
  list << mError;

  foreach (WbProtoTreeItem *child, mChildren)
    child->recursiveErrorAccumulator(list);
}

void WbProtoTreeItem::generateSessionProtoMap(QMap<QString, QString> &map) {
  assert(mIsReady);
  // in case of failure the tree might be incomplete, but what is inserted in the map must be known to be available
  if (!map.contains(mName) && mUrl.endsWith(".proto"))  // only insert protos, root file may be a world file
    map.insert(mName, mUrl);

  foreach (WbProtoTreeItem *child, mChildren)
    child->generateSessionProtoMap(map);
}

void WbProtoTreeItem::insert(const QString &url) {
  WbProtoTreeItem *child = new WbProtoTreeItem(url, this);
  child->setRawUrl(url);  // if requested to save to file, save it as it was loaded (i.e. without url manipulations)
  mChildren.append(child);
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
