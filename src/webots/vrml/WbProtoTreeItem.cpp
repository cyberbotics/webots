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

#include "WbProtoTreeItem.hpp"

#include "WbDownloadManager.hpp"
#include "WbDownloader.hpp"
#include "WbNetwork.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"

#include <QtCore/QDir>
#include <QtCore/QRegularExpression>

WbProtoTreeItem::WbProtoTreeItem(const QString &url, WbProtoTreeItem *parent, bool importable) :
  mUrl(url),
  mParent(parent),
  mImportable(importable),
  mIsReady(false),
  mDownloader(NULL),
  mName(QUrl(url).fileName().replace(".proto", "", Qt::CaseInsensitive)) {
}

WbProtoTreeItem::~WbProtoTreeItem() {
  qDeleteAll(mChildren);
  mChildren.clear();
}

void WbProtoTreeItem::parseItem() {
  QString path = mUrl;
  if (WbUrl::isWeb(path) && WbNetwork::instance()->isCachedWithMapUpdate(path))
    path = WbNetwork::instance()->get(path);

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    mError << tr("File '%1' is not readable.").arg(path);
    if (mParent) {
      mIsReady = true;  // reached the end of a branch, notify the parent about it
      mParent->readyCheck();
    }
    return;
  }

  // check if the root file references external PROTO
  QRegularExpression re("^\\s*(IMPORTABLE\\s+)?EXTERNPROTO\\s+\"(.*\\.proto)\"", QRegularExpression::MultilineOption);
  QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

  // begin by populating the list of all sub-PROTO
  while (it.hasNext()) {
    const QRegularExpressionMatch match = it.next();
    if (match.hasMatch()) {
      const bool hasImportableKeyword = !match.captured(1).isEmpty();
      const QString subProto = match.captured(2);
      const QString subProtoUrl = WbUrl::combinePaths(subProto, mUrl);
      if (subProtoUrl.isEmpty())
        continue;

      if (!subProtoUrl.endsWith(".proto", Qt::CaseInsensitive)) {
        mError << tr("Malformed EXTERNPROTO URL. The URL should end with '.proto'.");
        continue;
      }

      // sanity check (must either be: relative, absolute, starts with webots://, starts with https://)
      if (!subProtoUrl.startsWith("https://") && !subProtoUrl.startsWith("webots://") && !QFileInfo(subProtoUrl).isRelative() &&
          !QFileInfo(subProtoUrl).isAbsolute()) {
        mError << tr("Malformed EXTERNPROTO URL. Invalid URL provided: %1.").arg(subProtoUrl);
        continue;
      }

      // ensure there's no ambiguity between the declarations
      const QString subProtoName = QUrl(subProtoUrl).fileName().replace(".proto", "", Qt::CaseInsensitive);
      foreach (const WbProtoTreeItem *child, mChildren) {
        if (child->name() == subProtoName && WbUrl::resolveUrl(child->url()) != WbUrl::resolveUrl(subProtoUrl)) {
          mError << tr("PROTO '%1' is ambiguous, multiple references are provided: '%2' and '%3'. The first was used.")
                      .arg(subProtoName)
                      .arg(child->url())
                      .arg(subProtoUrl);
          continue;
        }
      }

      if (isRecursiveProto(subProtoUrl))
        continue;  // prevent endless downloads, the error itself is handled elsewhere

      // skip local sub-PROTO that don't actually exist on disk
      if (!WbUrl::isWeb(subProtoUrl) && !QFileInfo(subProtoUrl).exists()) {
        mError << tr("Skipped PROTO '%1' as it is not available at: %2.").arg(subProtoName).arg(subProtoUrl);
        continue;
      }

      WbProtoTreeItem *child = new WbProtoTreeItem(subProtoUrl, this, hasImportableKeyword);
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
  if (mUrl.isEmpty() && mParent == NULL) {  // special case for multi-proto download, the children don't share a common parent
    foreach (WbProtoTreeItem *child, mChildren)
      child->download();

    readyCheck();
    return;
  }

  if (WbUrl::isLocalUrl(mUrl)) {
    // note: this condition should only be possible in development mode when loading an old world since, during the
    // compilation, proto-list.xml urls will be local (webots://) and will be loaded as such by the backwards compatibility
    // mechanism; under any other circumstance, the on-the-fly URL manufacturing logic will convert any 'webots://' urls to
    // remote ones
    mUrl = QDir::cleanPath(mUrl.replace("webots://", WbStandardPaths::webotsHomePath()));
  }

  if (WbUrl::isWeb(mUrl)) {
    if (!WbNetwork::instance()->isCachedWithMapUpdate(mUrl)) {
      delete mDownloader;
      mDownloader = WbDownloadManager::instance()->createDownloader(QUrl(mUrl), this);
      connect(mDownloader, &WbDownloader::complete, this, &WbProtoTreeItem::downloadUpdate);
      mDownloader->download();
      return;
    }
  }

  parseItem();
}

void WbProtoTreeItem::downloadUpdate() {
  if (!mDownloader->error().isEmpty()) {
    mError << tr("Error downloading EXTERNPROTO '%1': %2").arg(mName).arg(mDownloader->error());
    if (mParent)
      mParent->deleteChild(this);
    else
      readyCheck();
    return;
  }

  assert(WbNetwork::instance()->isCachedNoMapUpdate(mUrl));
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

void WbProtoTreeItem::generateSessionProtoList(QStringList &sessionList) {
  assert(mIsReady);
  // in case of failure the tree might be incomplete, but what is inserted in the map must be known to be available
  if (!sessionList.contains(mUrl) &&
      mUrl.endsWith(".proto", Qt::CaseInsensitive))  // only insert protos, root file may be a world
    sessionList << WbUrl::resolveUrl(mUrl);

  foreach (WbProtoTreeItem *child, mChildren)
    child->generateSessionProtoList(sessionList);
}

void WbProtoTreeItem::insert(const QString &url) {
  // since the insert function is used to inject missing declarations, by default they have to be considered as non-importable
  WbProtoTreeItem *child = new WbProtoTreeItem(url, this, false);
  mChildren.append(child);
}

void WbProtoTreeItem::deleteChild(const WbProtoTreeItem *child) {
  assert(mChildren.contains(child));
  mError << child->error();  // accumulate the child's errors on the parent side
  mChildren.removeAll(child);
  delete child;
  readyCheck();
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
