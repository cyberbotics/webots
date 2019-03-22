// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbWebotsUpdateManager.hpp"

#include "WbNetwork.hpp"

#include <QtCore/QCoreApplication>
#include <QtNetwork/QNetworkReply>

#include <cassert>

WbWebotsUpdateManager *WbWebotsUpdateManager::cInstance = NULL;

WbWebotsUpdateManager::WbWebotsUpdateManager() : mVersion(), mTargetVersionAvailable(false), mError() {
  sendRequest();
}

WbWebotsUpdateManager::~WbWebotsUpdateManager() {
}

WbWebotsUpdateManager *WbWebotsUpdateManager::instance() {
  if (!cInstance) {
    cInstance = new WbWebotsUpdateManager();
    qAddPostRoutine(WbWebotsUpdateManager::cleanup);
  }
  return cInstance;
}

void WbWebotsUpdateManager::cleanup() {
  if (cInstance) {
    delete cInstance;
    cInstance = NULL;
  }
}

void WbWebotsUpdateManager::sendRequest() {
  QNetworkRequest request;
  request.setUrl(QUrl("https://www.cyberbotics.com/webots_current_version.txt"));
  QNetworkReply *reply = WbNetwork::instance()->networkAccessManager()->get(request);
  connect(reply, &QNetworkReply::finished, this, &WbWebotsUpdateManager::downloadReplyFinished, Qt::UniqueConnection);
}

void WbWebotsUpdateManager::downloadReplyFinished() {
  QNetworkReply *reply = dynamic_cast<QNetworkReply *>(sender());
  assert(reply);
  if (!reply)
    return;

  if (reply->error()) {
    mError = tr("Cannot get the Webots current version due to: \"%1\"").arg(reply->errorString());
    return;
  }

  disconnect(reply, &QNetworkReply::finished, this, &WbWebotsUpdateManager::downloadReplyFinished);

  QString answer = QString::fromUtf8(reply->readAll()).trimmed();
  bool success = mVersion.fromString(answer);
  if (!success) {
    mError = tr("Invalid format of the current Webots version: \"%1\"").arg(answer);
    return;
  }

  mError = "";
  mTargetVersionAvailable = true;
  emit targetVersionAvailable();
}
