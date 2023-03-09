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

#include "WbHttpReply.hpp"

#include "WbLog.hpp"

#include <QtCore/QCryptographicHash>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>

// Remarks:
// - "Access-Control-Allow-Origin" is to solve this error appearing at least on Chrome:
//    `Error: No 'Access-Control-Allow-Origin' header is present on the requested resource`

QByteArray WbHttpReply::forge404Reply(const QString &url) {
  QByteArray reply;
  const QByteArray data =
    "<!DOCTYPE html>\n<html lang=\"en\"><head><meta charset=UTF-8><title>Webots: Not Found</title></head><body>"
    "<h1>404: File Not Found</h1>" +
    url.toUtf8() + "</body></html>";
  reply.append("HTTP/1.1 404 Not Found\r\n");
  reply.append("Access-Control-Allow-Origin: *\r\n");
  reply.append("Content-Type: text/html\r\n");
  reply.append(QString("Content-Length: %1\r\n").arg(data.length()).toUtf8());
  reply.append("\r\n");
  reply.append(data);
  return reply;
}

QByteArray WbHttpReply::forgeFileReply(const QString &fileName, const QString &etag, const QString &host, const QString &url) {
  QByteArray reply;

  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly)) {
    WbLog::warning(QObject::tr("Cannot read '%1', sending \"404 Not Found\".").arg(fileName));
    return forge404Reply(url);
  }
  QByteArray data = file.readAll();
  QByteArray replace = "http://" + host.toUtf8() + "/";
  data.replace("webots://", replace);

  const QByteArray hash = QCryptographicHash::hash(data, QCryptographicHash::Md5);

  if (!etag.isEmpty() && hash.toHex().compare(etag.toLocal8Bit(), Qt::CaseSensitive) == 0) {
    reply.append("HTTP/1.1 304 Not modified\r\n");
    reply.append("Access-Control-Allow-Origin: *\r\n");
    reply.append("Cache-Control: public, max-age=3600\r\n");  // Help the browsers to cache the file for 1 hour.
    reply.append("etag: ").append(hash.toHex()).append("\r\n");
    reply.append("\r\n");
  } else {
    const QString mimeTypeString = WbHttpReply::mimeType(fileName, true);
    reply.append("HTTP/1.1 200 OK\r\n");
    reply.append("Access-Control-Allow-Origin: *\r\n");
    reply.append("Cache-Control: public, max-age=3600\r\n");  // Help the browsers to cache the file for 1 hour.
    reply.append("etag: ").append(hash.toHex()).append("\r\n");
    reply.append(QString("Content-Type: %1\r\n").arg(mimeTypeString).toUtf8());
    reply.append(QString("Content-Length: %1\r\n").arg(data.length()).toUtf8());
    reply.append("\r\n");
    reply.append(data);
  }

  return reply;
}

QString WbHttpReply::mimeType(const QString &url, bool generic) {
  const QString extension = url.mid(url.lastIndexOf('.') + 1).toLower();
  if (extension == "png" || extension == "jpg" || extension == "jpeg" || extension == "ico")
    return QString("image/%1").arg(extension);
  else if (extension == "html" || extension == "css")
    return QString("text/%1").arg(extension);
  else if (extension == "js")
    return "application/javascript";
  else if (extension == "dae")
    return "model/vnd.collada+xml";
  else if (extension == "obj")
    return "model/obj";
  else if (extension == "mtl")
    return "model/mtl";
  else if (extension == "stl")
    return "model/stl";
  else
    return generic ? "application/octet-stream" : "";  // generic binary format
}
