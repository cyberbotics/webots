#include "WbHttpReply.hpp"

bool WbHttpReply::computeReply(QString &reply) {
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
    // int imageData = ...;
    // imageData = ...;
    // QString imageExtension = ...;
    // reply.append(QString("Content-Type: text/%1\r\n").arg(imageExtension));
    // reply.append(QString("Content-Length: %1\r\n").arg(imageSize));
    // // reply.append("<!DOCTYPE html>\r\n");
    // reply.append(imageData);
    return true;
  }

  return false;
}
