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

#include "WbMultimediaStreamingServer.hpp"

#include "WbDragViewpointEvent.hpp"
#include "WbMainWindow.hpp"
#include "WbMultimediaStreamingLimiter.hpp"
#include "WbRobot.hpp"
#include "WbSimulationState.hpp"
#include "WbSimulationWorld.hpp"
#include "WbView3D.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenPicker.hpp"

#include <QtCore/QBuffer>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtGui/QMouseEvent>
#include <QtWebSockets/QWebSocket>

static WbView3D *gView3D = NULL;

WbMultimediaStreamingServer::WbMultimediaStreamingServer() :
  WbTcpServer(true),
  mImageWidth(-1),
  mImageHeight(-1),
  mImageUpdateTimeStep(50),
  mLimiter(NULL),
  mAverageBytesToWrite(0),
  mSentImagesCount(0),
  mFullResolutionOnPause(0),
  mBlockedResolutionFactor(-1),
  mTouchEventObjectPicked(false) {
  WbMatter::enableShowMatterCenter(false);
}

WbMultimediaStreamingServer::~WbMultimediaStreamingServer() {
  mTcpClients.clear();
  delete mLimiter;
}

void WbMultimediaStreamingServer::setView3D(WbView3D *view3D) {
  gView3D = view3D;
  gView3D->setVideoStreamingServer(this);
}

void WbMultimediaStreamingServer::start(int port) {
  WbTcpServer::start(port);
  WbLog::info(
    tr("Webots multimedia streamer started: resolution %1x%2 on port %3").arg(mImageWidth).arg(mImageHeight).arg(port));
  mWriteTimer.setSingleShot(true);
  connect(&mWriteTimer, &QTimer::timeout, this, &WbMultimediaStreamingServer::sendImageOnTimeout);
  connect(&mLimiterTimer, &QTimer::timeout, this, &WbMultimediaStreamingServer::processLimiterTimeout);
}

void WbMultimediaStreamingServer::sendTcpRequestReply(const QString &requestedUrl, const QString &etag, const QString &host,
                                                      QTcpSocket *socket) {
  if (requestedUrl != "mjpeg") {
    WbTcpServer::sendTcpRequestReply(requestedUrl, etag, host, socket);
    return;
  }
  socket->readAll();

  static const QByteArray &contentType = ("HTTP/1.0 200 OK\r\nServer: Webots\r\nConnection: close\r\nMax-Age: 0\r\n"
                                          "Expires: 0\r\nCache-Control: no-cache, private\r\nPragma: no-cache\r\n"
                                          "Content-Type: multipart/x-mixed-replace; boundary=WebotsFrame\r\n\r\n");
  socket->write(contentType);
  connect(socket, &QTcpSocket::disconnected, this, &WbMultimediaStreamingServer::removeTcpClient);
  mTcpClients.append(socket);
  // if available immediately send the latest image to the client
  if (mUpdateTimer.isValid())
    sendLastImage(socket);
  else if (WbSimulationState::instance()->isPaused())
    // request new image if none has been generated yet
    gView3D->refresh();

  if (!mLimiterTimer.isActive())
    mLimiterTimer.start(1000);
}

int WbMultimediaStreamingServer::bytesToWrite() {
  const QSslSocket *socket = dynamic_cast<QSslSocket *>(mTcpClients[0]);
  if (socket)
    return socket->encryptedBytesToWrite();
  return mTcpClients[0]->bytesToWrite();
}

void WbMultimediaStreamingServer::removeTcpClient() {
  QTcpSocket *client = qobject_cast<QTcpSocket *>(sender());
  if (client)
    mTcpClients.removeAll(client);
  if (mTcpClients.isEmpty())
    mLimiterTimer.stop();
}

bool WbMultimediaStreamingServer::isNewFrameNeeded() const {
  if (!isActive() || mTcpClients.isEmpty())
    return false;

  if (!mUpdateTimer.isValid() || WbSimulationState::instance()->isPaused())
    return true;

  const qint64 msecs = mUpdateTimer.elapsed();
  return msecs >= mImageUpdateTimeStep;  // maximum update time step
}

void WbMultimediaStreamingServer::sendImage(const QImage &image) {
  const double simulationTime = WbSimulationState::instance()->time();
  sendToClients(QString("time: %1").arg(simulationTime));

  QBuffer bufferJpeg(&mSceneImage);
  bufferJpeg.open(QIODevice::WriteOnly);
  image.save(&bufferJpeg, "JPG");

  const qint64 msecs = mUpdateTimer.isValid() ? mUpdateTimer.elapsed() : mImageUpdateTimeStep + 1;
  if (WbSimulationState::instance()->isPaused() && (msecs < mImageUpdateTimeStep))
    mWriteTimer.start(2 * mImageUpdateTimeStep - msecs);
  else
    sendImageOnTimeout();
}

void WbMultimediaStreamingServer::sendImageOnTimeout() {
  mWriteTimer.stop();

  sendLastImage();
  if (WbSimulationState::instance()->isPaused())
    // force the update on the client side
    // note that on Firefox it is enough to send the boundary line without image
    // but this trick doesn't work on Chrome
    sendLastImage();
  mUpdateTimer.restart();
}

void WbMultimediaStreamingServer::processLimiterTimeout() {
  if (mFullResolutionOnPause == 2)
    return;
  if (mLimiter->isStopped()) {
    if (bytesToWrite() == 0)
      mLimiter->resetStop();
    return;
  }
  if (mSentImagesCount == 0) {
    if (WbSimulationState::instance()->isPaused() && mLimiter->resolutionFactor() > 1) {
      // nothing sent since a while
      // send one image in full resolution
      mFullResolutionOnPause = 2;
      const QSize &fullSize(mLimiter->fullResolution());
      cMainWindow->setView3DSize(fullSize);
    }
    mLimiter->resetStop();
    return;
  }

  const double bytes = (mSentImagesCount > 0) ? ((double)mAverageBytesToWrite) / mSentImagesCount : 0;
  updateStreamingParameters(bytes / mSceneImage.size());
}

void WbMultimediaStreamingServer::updateStreamingParameters(int skippedImagesCount) {
  mLimiter->recomputeStreamingLimits(skippedImagesCount);
  if ((mBlockedResolutionFactor < 0) && (mFullResolutionOnPause > 0 || mLimiter->resolutionChanged())) {
    const QSize &newSize(mLimiter->resolution());
    cMainWindow->setView3DSize(newSize);
    mFullResolutionOnPause = 0;
  }
  mImageUpdateTimeStep = mLimiter->updateTimeStep();
  mAverageBytesToWrite = 0;
  mSentImagesCount = 0;
}

void WbMultimediaStreamingServer::sendLastImage(QTcpSocket *client) {
  if (client && (client->state() != QAbstractSocket::ConnectedState || !client->isValid()))
    return;

  if (mLimiter->isStopped()) {
    if (bytesToWrite() == 0) {
      mLimiter->resetStop();
    } else
      return;
  }

  mAverageBytesToWrite += bytesToWrite();
  mSentImagesCount++;

  const QByteArray &boundaryString =
    QString("--WebotsFrame\r\nContent-Type: image/jpeg\r\nContent-Length: %1\r\n\r\n").arg(mSceneImage.length()).toUtf8();
  QList<QTcpSocket *> clients;
  if (client)
    clients << client;
  else
    clients = mTcpClients;
  foreach (QTcpSocket *c, clients) {
    c->write(boundaryString);
    c->write(mSceneImage);
    c->write(QByteArray("\r\n"));
    c->flush();
  }
}

void WbMultimediaStreamingServer::sendContextMenuInfo(const WbMatter *node) {
  QJsonObject object;
  object.insert("name", node->name());
  object.insert("docUrl", node->documentationUrl());
  const WbRobot *robot = dynamic_cast<const WbRobot *>(node);
  object.insert("controller", robot ? robot->controllerName() : "");
  const WbSolid *const solid = dynamic_cast<const WbSolid *>(node);
  if (solid) {
    const WbViewpoint *viewpoint = WbWorld::instance()->viewpoint();
    const bool isFollowed = viewpoint->isFollowed(solid);
    object.insert("follow", isFollowed ? viewpoint->followType() : 0);
  } else
    object.insert("follow", -1);
  QJsonDocument jsonDocument(object);
  sendToClients("context menu: " + jsonDocument.toJson(QJsonDocument::Compact));
}

void WbMultimediaStreamingServer::processTextMessage(QString message) {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());
  if (mFullResolutionOnPause == 2)
    mFullResolutionOnPause = 1;

  if (message.startsWith("mouse")) {
    int action, button, buttons, x, y, modifiers, wheel;
    QString skip;  // will receive "mouse"
    QTextStream(&message) >> skip >> action >> button >> buttons >> x >> y >> modifiers >> wheel;
    if (mBlockedResolutionFactor < 0)
      mBlockedResolutionFactor = mLimiter->resolutionFactor();
    if (mFullResolutionOnPause == 0 && mLimiter && mBlockedResolutionFactor > 1) {
      const double factor = pow(2, mBlockedResolutionFactor - 1);
      x /= factor;
      y /= factor;
    }
    const QPointF point(x, y);
    const Qt::MouseButtons buttonsPressed = ((buttons & 1) ? Qt::LeftButton : Qt::NoButton) |
                                            ((buttons & 2) ? Qt::RightButton : Qt::NoButton) |
                                            ((buttons & 4) ? Qt::MiddleButton : Qt::NoButton);
    const Qt::KeyboardModifiers keyboardModifiers = ((modifiers & 1) ? Qt::ShiftModifier : Qt::NoModifier) |
                                                    ((modifiers & 2) ? Qt::ControlModifier : Qt::NoModifier) |
                                                    ((modifiers & 4) ? Qt::AltModifier : Qt::NoModifier);
    if (action <= 1) {
      QInputEvent::Type type;
      Qt::MouseButton buttonPressed;
      if (action == 0) {
        type = QEvent::MouseMove;
        buttonPressed = Qt::NoButton;
      } else {
        switch (button) {
          case 1:
            buttonPressed = Qt::LeftButton;
            break;
          case 2:
            buttonPressed = Qt::RightButton;
            break;
          case 3:
            buttonPressed = Qt::MiddleButton;
            break;
          default:
            buttonPressed = Qt::NoButton;
            break;
        }
        if (action == -1)
          type = QEvent::MouseButtonPress;
        else if (action == 1) {
          type = QEvent::MouseButtonRelease;
          mBlockedResolutionFactor = -1;
        } else
          type = QEvent::MouseMove;
      }
      QMouseEvent event(type, point, QCursor::pos(), buttonPressed, buttonsPressed, keyboardModifiers);
      if (gView3D) {
        const WbMatter *contextMenuNode = gView3D->remoteMouseEvent(&event);
        if (contextMenuNode)
          sendContextMenuInfo(contextMenuNode);
      }
    } else if (action == 2) {
      wheel = -wheel;  // Wheel delta is inverted in JS and Webots
      QWheelEvent wheelEvent(point, point, QPoint(), QPoint(0, wheel), buttonsPressed, keyboardModifiers, Qt::ScrollUpdate,
                             false);
      if (gView3D)
        gView3D->remoteWheelEvent(&wheelEvent);
    }
  } else if (message.startsWith("touch")) {
    int action, eventType, x, y;
    QString skip;  // will receive "touch"
    QTextStream stream(&message);
    stream >> skip >> action >> eventType;
    if (action == -1) {  // store touch event center
      stream >> x >> y;
      if (mFullResolutionOnPause == 0 && mLimiter && mLimiter->resolutionFactor() > 1) {
        const double factor = pow(2, mLimiter->resolutionFactor() - 1);
        x /= factor;
        y /= factor;
      }
      WbWrenPicker picker;
      picker.pick(x, y);
      WbVector3 screenCoords = picker.screenCoordinates();
      screenCoords[0] = (screenCoords[0] / mImageWidth) * 2 - 1;
      screenCoords[1] = (screenCoords[1] / mImageHeight) * 2 - 1;
      mTouchEventObjectPicked = picker.selectedId() != -1;
      WbViewpoint *viewpoint = WbWorld::instance()->viewpoint();
      viewpoint->toWorld(screenCoords, mTouchEventRotationCenter);
      if (eventType == 2) {
        double distanceToPickPosition;
        if (mTouchEventObjectPicked)
          distanceToPickPosition = (viewpoint->position()->value() - viewpoint->rotationCenter()).length();
        else
          distanceToPickPosition = viewpoint->position()->value().length();
        if (distanceToPickPosition < 0.001)
          distanceToPickPosition = 0.001;
        mTouchEventZoomScale =
          distanceToPickPosition * 2 * tan(viewpoint->fieldOfView()->value() / 2) / std::max(mImageWidth, mImageHeight);
      } else
        mTouchEventZoomScale = 1.0;
    } else if (action == 0 && eventType == 1) {  // touch rotate event
      stream >> x >> y;
      if (mFullResolutionOnPause == 0 && mLimiter && mLimiter->resolutionFactor() > 1) {
        const double factor = pow(2, mLimiter->resolutionFactor() - 1);
        x /= factor;
        y /= factor;
      }
      WbRotateViewpointEvent::applyToViewpoint(QPoint(x, y), mTouchEventRotationCenter,
                                               -WbWorld::instance()->worldInfo()->gravityUnitVector(), mTouchEventObjectPicked,
                                               WbWorld::instance()->viewpoint());

      gView3D->refresh();
    } else if (action == 0 && eventType == 2) {  // touch zoom/tilt event
      double tiltAngle, zoom;
      stream >> tiltAngle >> zoom;
      WbZoomAndRotateViewpointEvent::applyToViewpoint(tiltAngle, zoom, mTouchEventZoomScale, WbWorld::instance()->viewpoint());
      gView3D->refresh();
    }
  } else if (message.startsWith("mjpeg: ")) {
    const QStringList &resolution = message.mid(7).split("x");
    const int width = resolution[0].toInt();
    const int height = resolution[1].toInt();
    QString args;
    if ((mImageWidth <= 0 && mImageHeight <= 0) || client == mWebSocketClients.first()) {
      cMainWindow->setView3DSize(QSize(width, height));
      mImageWidth = width;
      mImageHeight = height;
      WbLog::info(tr("Streaming server: Resolution changed to %1x%2.").arg(width).arg(height));
      delete mLimiter;
      mLimiter = new WbMultimediaStreamingLimiter(QSize(mImageWidth, mImageHeight), 50);
    } else {
      // Video streamer already initialized
      WbLog::info(tr("Streaming server: Ignored new client request of resolution: %1x%2.").arg(width).arg(height));
      args = QString("%1 %2").arg(mImageWidth).arg(mImageHeight);
    }
    client->sendTextMessage(QString("multimedia: mjpeg %2 %3").arg(simulationStateString(false)).arg(args));
    const QString &stateMessage = simulationStateString();
    if (!stateMessage.isEmpty())
      client->sendTextMessage(stateMessage);
    sendWorldToClient(client);
    sendToClients();  // send possible bufferized messages
  } else if (message.startsWith("resize: ")) {
    if (client == mWebSocketClients.first()) {
      const QStringList &resolution = message.mid(8).split("x");
      mImageWidth = resolution[0].toInt();
      mImageHeight = resolution[1].toInt();
      WbLog::info(tr("Streaming server: Client resize: new resolution %1x%2.").arg(mImageWidth).arg(mImageHeight));
      cMainWindow->setView3DSize(QSize(mImageWidth, mImageHeight));
      sendToClients(QString("resize: %1 %2").arg(mImageWidth).arg(mImageHeight));
      mLimiter->resetResolution(QSize(mImageWidth, mImageHeight));
    } else
      WbLog::info(tr("Streaming server: Invalid client resize: only the first connected client can resize the simulation."));
  } else if (message.startsWith("follow: ")) {
    const int separatorIndex = message.indexOf(',');
    const QString &mode = message.mid(8, separatorIndex - 8);
    const QString &solidId = message.mid(separatorIndex + 1);
    WbSolid *const solid = WbSolid::findSolidFromUniqueName(solidId);
    WbViewpoint *const viewpoint = WbWorld::instance()->viewpoint();
    if (viewpoint->followedSolid())
      viewpoint->terminateFollowUp();
    if (solid) {
      viewpoint->setFollowType(mode.toInt());
      viewpoint->startFollowUp(solid, true);
    }
  } else if (message.startsWith("x3d")) {
    WbLog::error(tr("Streaming server received unsupported X3D message: '%1'. You should run Webots with the "
                    "'--stream=\"mode=x3d\"' command line option.")
                   .arg(message));
    return;
  } else
    WbTcpServer::processTextMessage(message);
}

void WbMultimediaStreamingServer::sendWorldToClient(QWebSocket *client) {
  const WbWorldInfo *currentWorldInfo = WbWorld::instance()->worldInfo();
  QJsonObject infoObject;
  infoObject.insert("window", currentWorldInfo->window());
  infoObject.insert("title", currentWorldInfo->title());
  const QJsonDocument infoDocument(infoObject);
  client->sendTextMessage("world info: " + infoDocument.toJson(QJsonDocument::Compact));
  WbTcpServer::sendWorldToClient(client);

  const QList<WbRobot *> &robots = WbWorld::instance()->robots();
  foreach (const WbRobot *robot, robots)
    WbTcpServer::sendRobotWindowInformation(client, robot);

  client->sendTextMessage("scene load completed");
}
