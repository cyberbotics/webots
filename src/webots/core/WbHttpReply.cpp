#include "WbHttpReply.hpp"

#include <QtCore/QFile>
#include <QtCore/QFileInfo>

QByteArray WbHttpReply::forgeHTMLReply(const QString &htmlContent) {
  QByteArray reply;
  reply.append("HTTP/1.1 200 OK\r\n");
  reply.append("Content-Type: text/html\r\n");
  reply.append(QString("Content-Length: %1\r\n").arg(htmlContent.length()));
  reply.append("\r\n");
  reply.append(htmlContent);
  return reply;
}

QByteArray WbHttpReply::forgeImageReply(const QString &imageFileName) {
  QByteArray reply;

  QFile imageFile(imageFileName);
  if (!imageFile.open(QIODevice::ReadOnly))
    return reply;
  QByteArray imageData = imageFile.readAll();
  int imageSize = imageData.length();
  QFileInfo fi(imageFile);
  QString imageExtension = fi.suffix().toLower();

  reply.append("HTTP/1.1 200 OK\r\n");
  reply.append(QString("Content-Type: image/%1\r\n").arg(imageExtension));
  reply.append(QString("Content-Length: %1\r\n").arg(imageSize));
  reply.append("\r\n");
  reply.append(imageData);

  return reply;
}
