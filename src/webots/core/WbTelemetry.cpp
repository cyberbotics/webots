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

#include "WbTelemetry.hpp"

#include "WbApplicationInfo.hpp"
#include "WbNetwork.hpp"
#include "WbPreferences.hpp"
#include "WbSysInfo.hpp"

#include <cassert>

#include <QtCore/QEventLoop>
#include <QtCore/QTimer>
#include <QtNetwork/QNetworkReply>

void WbTelemetry::send(const QString &file) {
  static WbTelemetry telemetry;
  if (!file.isEmpty()) {
    telemetry.mFile = file;
    telemetry.sendRequest("trial");
  } else
    telemetry.sendRequest("success");
}

void WbTelemetry::sendRequest(const QString &operation) {
  QNetworkRequest request(QUrl("https://www.cyberbotics.com/telemetry.php"));
  QByteArray data;
  data.append("id=");
  const int id = WbPreferences::instance()->value("General/telemetryId", 0).toString().toInt();
  data.append(QString::number(id).toUtf8());
  data.append("&operation=");
  data.append(QUrl::toPercentEncoding(operation));
  data.append("&file=");
  data.append(QUrl::toPercentEncoding(mFile));
  data.append("&version=");
  data.append(QUrl::toPercentEncoding(WbApplicationInfo::version().toString()));
  data.append("&os=");
  data.append(QUrl::toPercentEncoding(WbSysInfo::sysInfo()));
  data.append("&glVendor=");
  data.append(QUrl::toPercentEncoding(WbSysInfo::openGLVendor()));
  data.append("&glRenderer=");
  data.append(QUrl::toPercentEncoding(WbSysInfo::openGLRenderer()));
  data.append("&glVersion=");
  data.append(QUrl::toPercentEncoding(WbSysInfo::openGLVersion()));
  data.append("&textureQuality=");
  data.append(WbPreferences::instance()->value("OpenGL/textureQuality", 0).toString());
  data.append("&disableCameraAntiAliasing=");
  data.append(WbPreferences::instance()->value("OpenGL/disableCameraAntiAliasing", 0).toString());
  data.append("&disableShadows=");
  data.append(WbPreferences::instance()->value("OpenGL/disableShadows", 0).toString());
  data.append("&GTAO=");
  data.append(WbPreferences::instance()->value("OpenGL/GTAO", 0).toString());
  data.append("&SMAA=");
  data.append(WbPreferences::instance()->value("OpenGL/SMAA", 0).toString());
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");
  QNetworkReply *reply = WbNetwork::instance()->networkAccessManager()->post(request, data);
  if (id == 0) {
    QEventLoop loop;
    QTimer timer;
    timer.start(5000);  // allow a maximum of 5 seconds before giving up on the server answer
    connect(reply, &QNetworkReply::finished, this, &WbTelemetry::requestReplyFinished, Qt::UniqueConnection);
    connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);
    loop.exec();
    if (timer.remainingTime() == 0)  // time out occurred
      disconnect(reply, &QNetworkReply::finished, 0, 0);
  }
}

void WbTelemetry::requestReplyFinished() {
  QNetworkReply *reply = dynamic_cast<QNetworkReply *>(sender());
  assert(reply);
  if (!reply)
    return;
  if (reply->error())
    return;
  disconnect(reply, &QNetworkReply::finished, this, &WbTelemetry::requestReplyFinished);
  const QString answer = QString::fromUtf8(reply->readAll()).trimmed();
  QStringList answers = answer.split(" ");
  WbPreferences::instance()->setValue("General/telemetryId", answers[0]);
  WbPreferences::instance()->setValue("General/telemetryPassword", answers[1]);  // stored for later use
}
