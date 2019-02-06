// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbNetwork.hpp"
#include "WbPreferences.hpp"
#include "WbTelemetry.hpp"

#include <QtCore/QCoreApplication>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>

#include <cassert>

void WbTelemetry::send(const QString &message) {
  WbTelemetry *t = new WbTelemetry();
  t->sendRequest(message);
  delete t;
}

void WbTelemetry::sendRequest(const QString &message) {
  QNetworkRequest request(QUrl("https://cyberbotics.com/telemetry.php"));
  QString m;
  if (!m.isEmpty())
    m = message + "&";
  else
    m = message;
  m += "id=" + WbPreferences::instance()->value("General/TelemetryId", 0).toString();
  const QByteArray data = m.toUtf8();
  QNetworkReply *reply = WbNetwork::instance()->networkAccessManager()->post(request, data);
  connect(reply, &QNetworkReply::finished, this, &WbTelemetry::requestReplyFinished, Qt::UniqueConnection);
}

void WbTelemetry::requestReplyFinished() {
  QNetworkReply *reply = dynamic_cast<QNetworkReply *>(sender());
  assert(reply);
  if (!reply)
    return;
  if (reply->error())
    return;
  disconnect(reply, &QNetworkReply::finished, this, &WbTelemetry::requestReplyFinished);
  QString id = QString::fromUtf8(reply->readAll()).trimmed();
  WbPreferences::instance()->setValue("General/TelemetryId", id);
}
