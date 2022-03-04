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

#include "WbDownloader.hpp"

#include "WbApplication.hpp"
#include "WbNetwork.hpp"
#include "WbSimulationState.hpp"

#include <QtCore/QDir>
#include <QtCore/QTimer>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkRequest>

static int gCount = 0;
static int gComplete = 0;
static bool gDownloading = false;
static QTimer *gTimer = NULL;
static bool gDisplayPopUp = false;
static QMap<QUrl, QNetworkReply *> gUrlCache;

int WbDownloader::progress() {
  return gCount == 0 ? 100 : 100 * gComplete / gCount;
}

void WbDownloader::reset() {
  gCount = 0;
  gComplete = 0;
}

WbDownloader::WbDownloader(QObject *parent) : QObject(parent), mNetworkReply(NULL), mFinished(false), mCopy(false) {
  gCount++;
}

WbDownloader::~WbDownloader() {
  if (mNetworkReply != NULL) {
    mNetworkReply->deleteLater();
    if (gUrlCache.contains(mUrl))
      gUrlCache.remove(mUrl);
  }

  if (!mFinished)
    gCount--;
}

// TODO: if nobody uses it, delete
QIODevice *WbDownloader::device() const {
  return dynamic_cast<QIODevice *>(mNetworkReply);
}

void WbDownloader::download(const QUrl &url) {
  WbSimulationState::instance()->pauseSimulation();

  mUrl = url;

  printf(">> contains: %d\n", gUrlCache.contains(mUrl));
  if (gUrlCache.contains(mUrl) && !mCopy) {
    printf("COPY!\n");
    mCopy = true;
    QNetworkReply *reply = gUrlCache[mUrl];
    if (reply && !reply->isFinished())
      connect(reply, &QNetworkReply::finished, this, &WbDownloader::finished, Qt::UniqueConnection);
    else
      finished();
    return;
  }

  if (!gDownloading) {
    gDownloading = true;
    gTimer = new QTimer(0);
    connect(gTimer, &QTimer::timeout, &WbDownloader::displayPopUp);
    gTimer->setInterval(1000);
    gTimer->setSingleShot(true);
    gTimer->start();
  }
  QNetworkRequest request;
  request.setUrl(url);
  mFinished = false;

  assert(mNetworkReply == NULL);
  mNetworkReply = WbNetwork::instance()->networkAccessManager()->get(request);
  printf("connect to finish\n");
  connect(mNetworkReply, &QNetworkReply::finished, this, &WbDownloader::finished, Qt::UniqueConnection);
  connect(WbApplication::instance(), &WbApplication::worldLoadingWasCanceled, mNetworkReply, &QNetworkReply::abort);

  gUrlCache.insert(url, mNetworkReply);
}

void WbDownloader::finished() {
  printf("finished\n");
  // cache result
  if (mNetworkReply && mNetworkReply->error()) {
    mError = tr("Cannot download %1: %2").arg(mUrl.toString()).arg(mNetworkReply->errorString());
    disconnect(mNetworkReply, &QNetworkReply::finished, this, &WbDownloader::finished);
  } else {
    if (!mCopy) {  // only save to disk the primary download, copies don't need to
      assert(mNetworkReply != NULL);
      WbNetwork::instance()->save(mUrl.toString(), mNetworkReply->readAll());
    }
  }

  gComplete++;
  mFinished = true;
  emit complete();

  if (gComplete == gCount) {
    gDownloading = false;
    gDisplayPopUp = false;
    gUrlCache.clear();
    emit WbApplication::instance()->deleteWorldLoadingProgressDialog();
    WbSimulationState::instance()->resumeSimulation();
  } else if (gDisplayPopUp)
    emit WbApplication::instance()->setWorldLoadingProgress(progress());
}

void WbDownloader::displayPopUp() {
  if (gDownloading) {
    WbApplication::instance()->setWorldLoadingStatus(tr("Downloading assets"));
    gDisplayPopUp = true;
  }

  if (gTimer) {
    delete gTimer;
    gTimer = NULL;
  }
}
