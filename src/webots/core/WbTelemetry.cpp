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

#include <QtGui/QOpenGLFunctions_3_3_Core>
#include <QtNetwork/QNetworkReply>

void WbTelemetry::send(const QString &file, const QString &operation) {
  static WbTelemetry telemetry;
  telemetry.sendRequest(file, operation);
}

void WbTelemetry::sendRequest(const QString &file, const QString &operation) {
  QNetworkRequest request(QUrl("https://www.cyberbotics.com/telemetry.php"));
  QByteArray data;
  data.append("id=");
  data.append(WbPreferences::instance()->value("General/TelemetryId", 0).toString());
  data.append("operation=");
  data.append(QUrl::toPercentEncoding(operation));
  data.append("&file=");
  data.append(QUrl::toPercentEncoding(file));
  data.append("&version=");
  data.append(QUrl::toPercentEncoding(WbApplicationInfo::version().toString()));
  data.append("&os=");
  data.append(QUrl::toPercentEncoding(WbSysInfo::sysInfo()));
  QOpenGLFunctions_3_3_Core gl;
  gl.initializeOpenGLFunctions();
  data.append("&glVendor=");
  data.append(QUrl::toPercentEncoding((const char *)gl.glGetString(GL_VENDOR)));
  data.append("&glRenderer=");
  data.append(QUrl::toPercentEncoding((const char *)gl.glGetString(GL_RENDERER)));
  data.append("&glVersion=");
  data.append(QUrl::toPercentEncoding((const char *)gl.glGetString(GL_VERSION)));
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
  const QString id = QString::fromUtf8(reply->readAll()).trimmed();
  WbPreferences::instance()->setValue("General/telemetryId", id);
}
