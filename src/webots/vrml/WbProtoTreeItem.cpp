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
#include "WbLog.hpp"
#include "WbNetwork.hpp"

#include <QtCore/QRegularExpression>

WbProtoTreeItem::WbProtoTreeItem(const QString &url, WbProtoTreeItem *parent) :
  mUrl(url),
  mIsAvailable(false),
  mParent(parent),
  mDownloader(NULL),
  mName(QUrl(url).fileName().replace(".proto", "")) {
  // if the proto is locally available, parse it, otherwise download it first
  downloadAssets();
}
WbProtoTreeItem::~WbProtoTreeItem() {
  qDeleteAll(mSubProto);
  mSubProto.clear();
}

void WbProtoTreeItem::parseItem() {
  assert(!mUrl.startsWith("webots://"));  // TODO: tmp, the structure should only contain direct urls (http or absolute)

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
          WbLog::error(tr("Malformed extern proto url. The url should end with '.proto'\n"));
          return;
        }

        QString protoName;
        if (WbUrl::isWeb(subProtoUrl))
          protoName = QUrl(subProtoUrl).fileName().replace(".proto", "");
        else
          assert(0);  // TODO: case not handled/tested yes (relative url, abs, ..). Clean this up

        WbProtoTreeItem *child = new WbProtoTreeItem(subProtoUrl, this);
        mSubProto.append(child);
      }
    }

    mIsAvailable = true;  // wait until mSubProto has been populated before flagging this item as ready
    emit protoTreeUpdated();
  } else
    WbLog::error(tr("File '%1' is not readable.").arg(path));
}

void WbProtoTreeItem::downloadAssets() {
  mSubProto.clear();

  if (WbUrl::isWeb(mUrl)) {
    if (!WbNetwork::instance()->isCached(mUrl)) {
      printf("%35s not cached. Downloading it.\n", mName.toUtf8().constData());
      delete mDownloader;
      mDownloader = new WbDownloader(this);
      connect(mDownloader, &WbDownloader::complete, this, &WbProtoTreeItem::downloadUpdate);
      mDownloader->download(QUrl(mUrl));
      return;
    }
    printf("%35s is cached\n", mName.toUtf8().constData());
  }

  parseItem();
}

void WbProtoTreeItem::downloadUpdate() {
  if (!mDownloader->error().isEmpty()) {
    WbLog::error(mDownloader->error());
    return;
  }

  assert(WbNetwork::instance()->isCached(mUrl));
  printf("%35s downloded\n", mName.toUtf8().constData());
  parseItem();
}

bool WbProtoTreeItem::isReadyForLoad() {
  bool isAvailable = mIsAvailable;
  foreach (WbProtoTreeItem *subProto, mSubProto)
    isAvailable = isAvailable && subProto->isReadyForLoad();
  return isAvailable;
}

void WbProtoTreeItem::generateProtoMap(QMap<QString, QString> &map) {
  if (!map.contains(mName) && mParent != NULL) {
    // printf("inserting <%s,%s>\n", mName.toUtf8().constData(), mUrl.toUtf8().constData());
    map.insert(mName, mUrl);
  }

  foreach (WbProtoTreeItem *proto, mSubProto)
    proto->generateProtoMap(map);
}