#include "WbHttpReply.hpp"

#include <QtCore/QFile>
#include <QtCore/QFileInfo>

bool WbHttpReply::computeReply(QByteArray &reply) {
  reply.clear();

  if (mHtmlContent.isEmpty() && mImageFileName.isEmpty())
    return false;

  reply.append("HTTP/1.1 200 OK\r\n");

  if (!mHtmlContent.isEmpty()) {
    reply.append("Content-Type: text/html\r\n");
    reply.append(QString("Content-Length: %1\r\n").arg(mHtmlContent.length()));
    reply.append("\r\n");
    reply.append("<!DOCTYPE html>\r\n");
    reply.append(mHtmlContent);
    return true;
  } else if (!mImageFileName.isEmpty()) {
    QFile imageFile(mImageFileName);
    if (!imageFile.open(QIODevice::ReadOnly))
      return false;
    QByteArray imageData = imageFile.readAll();
    int imageSize = imageData.length();
    QFileInfo fi(imageFile);
    QString imageExtension = fi.suffix().toLower();
    reply.append(QString("Content-Type: image/%1\r\n").arg(imageExtension));
    reply.append(QString("Content-Length: %1\r\n").arg(imageSize));
    reply.append("\r\n");
    reply.append(imageData);
    return true;
  }

  return false;
}
