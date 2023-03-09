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

#include "WbProtoIcon.hpp"

#include "WbDownloadManager.hpp"
#include "WbDownloader.hpp"
#include "WbFileUtil.hpp"
#include "WbNetwork.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"

#include <QtCore/QDir>
#include <QtCore/QUrl>

WbProtoIcon::WbProtoIcon(const QString &modelName, const QString &protoPath, QObject *parent) :
  QObject(parent),
  mPath(QString("%1icons/%2.png").arg(QUrl(protoPath).adjusted(QUrl::RemoveFilename).toString()).arg(modelName)),
  mModelName(modelName),
  mDownloader(NULL),
  mReady(true) {
  if (WbUrl::isWeb(mPath)) {
    if (WbNetwork::instance()->isCachedWithMapUpdate(mPath))
      mPath = WbNetwork::instance()->get(mPath);
    else {
      mReady = false;
      mDownloader = WbDownloadManager::instance()->createDownloader(QUrl(mPath), this);
      connect(mDownloader, &WbDownloader::complete, this, &WbProtoIcon::updateIcon);
      mDownloader->download();
    }
  } else if (WbUrl::isLocalUrl(mPath))
    mPath = QDir::cleanPath(mPath.replace("webots://", WbStandardPaths::webotsHomePath()));
}

void WbProtoIcon::updateIcon() {
  assert(mDownloader);
  if (mDownloader->error().isEmpty())
    mPath = WbNetwork::instance()->get(mDownloader->url().toString());
  else
    mPath = QString();
  // else failure downloading or file does not exist (404)

  mReady = true;
  emit iconReady(mPath);
}

void WbProtoIcon::duplicate(QDir destinationDir) {
  assert(mReady);
  if (!QFile::exists(mPath))
    return;

  if (destinationDir.exists("icons") || destinationDir.mkdir("icons"))
    WbFileUtil::forceCopy(mPath, QString("%1/icons/%2.png").arg(destinationDir.absolutePath()).arg(mModelName));
}
