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

#include "WbWebotsUpdateManager.hpp"

#include "WbNetwork.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtNetwork/QNetworkReply>

#include <cassert>

WbWebotsUpdateManager *WbWebotsUpdateManager::cInstance = NULL;

WbWebotsUpdateManager::WbWebotsUpdateManager() : mTargetVersionAvailable(false) {
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
  request.setUrl(QUrl("https://api.github.com/repos/cyberbotics/webots/releases/latest"));
  QNetworkReply *reply = WbNetwork::instance()->networkAccessManager()->get(request);
  connect(reply, &QNetworkReply::finished, this, &WbWebotsUpdateManager::downloadReplyFinished, Qt::UniqueConnection);
}

void WbWebotsUpdateManager::downloadReplyFinished() {
  QNetworkReply *reply = dynamic_cast<QNetworkReply *>(sender());
  assert(reply);
  if (!reply)
    return;

  disconnect(reply, &QNetworkReply::finished, this, &WbWebotsUpdateManager::downloadReplyFinished);

  if (reply->error()) {
    mError = tr("Cannot get the Webots current version due to: \"%1\"").arg(reply->errorString());
    reply->deleteLater();
    return;
  }

  bool success = false;
  QJsonDocument doc = QJsonDocument::fromJson(reply->readAll());
  if (!doc.isNull()) {
    if (doc.isObject()) {
      QJsonObject obj = doc.object();
      if (obj.contains("tag_name"))
        success = mVersion.fromString(obj.value("tag_name").toString());
    }
  }

  reply->deleteLater();

  if (!success) {
    mError = tr("Invalid answer from the GitHub REST API.");
    return;
  }

  mError = "";
  mTargetVersionAvailable = true;
  emit targetVersionAvailable();
}
