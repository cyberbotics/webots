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

#include "WbApplicationInfo.hpp"
#include "WbDownloader.hpp"
#include "WbNetwork.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"
#include "WbVersion.hpp"

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
  QRegularExpression re("^\\s*EXTERNPROTO\\s+\"(.*\\.proto)\"", QRegularExpression::MultilineOption);
  QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

  // begin by populating the list of all sub-PROTO
  while (it.hasNext()) {
    QRegularExpressionMatch match = it.next();
    if (match.hasMatch()) {
      const QString subProto = match.captured(1);
      const QString subProtoUrl = combinePaths(subProto, mUrl);

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
        if (child->name() == subProtoName && WbUrl::computePath(child->url()) != WbUrl::computePath(subProtoUrl)) {
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
  if (mUrl.isEmpty() && mParent == NULL) {  // special case for multi-proto download, the children don't share a common parent
    foreach (WbProtoTreeItem *child, mChildren)
      child->download();

    readyCheck();
    return;
  }

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
    mError << QString(tr("Error downloading EXTERNPROTO '%1': %2")).arg(mName).arg(mDownloader->error());
    if (mParent)
      mParent->deleteChild(this);
    else
      readyCheck();
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

QString WbProtoTreeItem::combinePaths(const QString &rawUrl, const QString &rawParentUrl) {
  // use cross-platform forward slashes
  QString url = rawUrl;
  url = url.replace("\\", "/");
  QString parentUrl = rawParentUrl;
  parentUrl = parentUrl.replace("\\", "/");

  // cases where no url manipulation is necessary
  if (WbUrl::isWeb(url))
    return url;

  if (QDir::isAbsolutePath(url))
    return QDir::cleanPath(url);

  if (WbUrl::isLocalUrl(url)) {
    // url fall-back mechanism: only trigger if the parent is a world file (.wbt), and the file (webots://) does not exist
    if (parentUrl.endsWith(".wbt") && !QFileInfo(QDir::cleanPath(WbStandardPaths::webotsHomePath() + url.mid(9))).exists()) {
      mError << QString(tr("URL '%1' changed by fallback mechanism. Ensure you are opening the correct world.")).arg(url);
      return url.replace("webots://", WbUrl::remoteWebotsAssetPrefix());
    }

    // infer url based on parent's url
    const QString &prefix = WbUrl::computePrefix(parentUrl);
    if (!prefix.isEmpty())
      return url.replace("webots://", prefix);

    if (parentUrl.isEmpty() || WbUrl::isLocalUrl(parentUrl) || QDir::isAbsolutePath(parentUrl))
      return QDir::cleanPath(WbStandardPaths::webotsHomePath() + url.mid(9));

    return QString();
  }

  if (QDir::isRelativePath(url)) {
    // for relative urls, begin by searching relative to the world and protos folders
    QStringList searchPaths = QStringList() << WbProject::current()->worldsPath() << WbProject::current()->protosPath();
    foreach (const QString &path, searchPaths) {
      QDir dir(path);
      if (dir.exists(url))
        return QDir::cleanPath(dir.absoluteFilePath(url));
    }

    // if it is not available in those folders, infer the url based on the parent's url
    if (WbUrl::isWeb(parentUrl) || QDir::isAbsolutePath(parentUrl) || WbUrl::isLocalUrl(parentUrl)) {
      // remove filename and trailing slash from parent url
      parentUrl = QUrl(parentUrl).adjusted(QUrl::RemoveFilename | QUrl::StripTrailingSlash).toString();

      if (url.startsWith("./"))
        url.remove(0, 2);

      // consume directories in both urls accordingly
      while (url.startsWith("../")) {
        parentUrl = parentUrl.left(parentUrl.lastIndexOf("/"));
        url.remove(0, 3);
      }

      const QString newUrl = parentUrl + "/" + url;
      if (WbUrl::isWeb(parentUrl) || QDir::isAbsolutePath(parentUrl))
        return newUrl;

      if (WbUrl::isLocalUrl(parentUrl))
        return QDir::cleanPath(WbStandardPaths::webotsHomePath() + newUrl.mid(9));
    }
  }

  mError << QString(tr("Impossible to infer URL from '%1' and '%2'").arg(rawUrl).arg(rawParentUrl));
  return QString();
}
