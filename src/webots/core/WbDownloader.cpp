// Copyright 1996-2025 Cyberbotics Ltd.
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

#include "WbDownloader.hpp"

#include "WbNetwork.hpp"

#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkRequest>

WbDownloader::WbDownloader(const QUrl &url, const WbDownloader *existingDownload, QObject *parent) :
  QObject(parent),
  mUrl(url),
  mNetworkReply(NULL),
  mExistingDownload(existingDownload),
  mFinished(false) {
}

WbDownloader::~WbDownloader() {
  if (mNetworkReply) {
    mNetworkReply->deleteLater();

    // properties used by WbDownloadManager
    setProperty("url", mUrl);
  }
  setProperty("finished", mFinished);
}

void WbDownloader::download() {
  if (mExistingDownload) {
    if (!mExistingDownload->hasFinished())
      connect(mExistingDownload->networkReply(), &QNetworkReply::finished, this, &WbDownloader::finished, Qt::UniqueConnection);
    else
      finished();

    return;
  }

  QNetworkRequest request;
  request.setUrl(mUrl);
  // Many PROTO and texture files are downloaded in parallel from the same host. Over HTTP/2 they share a single
  // connection, and when the server drops it (e.g. "Connection closed" or "header compression context" errors on
  // raw.githubusercontent.com) all of them fail at once. HTTP/1.1 uses separate connections, which is more robust.
  request.setAttribute(QNetworkRequest::Http2AllowedAttribute, false);
  mFinished = false;

  assert(mNetworkReply == NULL);
  mNetworkReply = WbNetwork::instance()->networkAccessManager()->get(request);
  connect(mNetworkReply, &QNetworkReply::finished, this, &WbDownloader::finished, Qt::UniqueConnection);
}

void WbDownloader::finished() {
  // cache result
  if (mNetworkReply) {
    if (mNetworkReply->error()) {
      mError = tr("Cannot download '%1', error code: %2: %3")
                 .arg(mUrl.toString())
                 .arg(mNetworkReply->error())
                 .arg(mNetworkReply->errorString());
    } else {  // only save to disk the primary download, copies don't need to
      assert(mNetworkReply != NULL);
      WbNetwork::instance()->save(mUrl.toString(), mNetworkReply->readAll());
    }
  }

  mFinished = true;
  emit complete();
}

void WbDownloader::abort() {
  if (mNetworkReply)
    mNetworkReply->abort();
};
