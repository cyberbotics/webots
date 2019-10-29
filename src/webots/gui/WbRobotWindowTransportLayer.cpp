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

#include "WbRobotWindowTransportLayer.hpp"

#include <QtCore/QString>

WbRobotWindowTransportLayer::WbRobotWindowTransportLayer(QObject *parent) :
  QObject(parent),
  mAckReceived(false),
  mWaitingAckCount(0) {
  connect(this, &WbRobotWindowTransportLayer::waitingAckCountChanged, this,
          &WbRobotWindowTransportLayer::updateWaitingAckCount);
}

void WbRobotWindowTransportLayer::receiveFromJavascript(const QByteArray &javascript) {
  emit javascriptReceived(javascript);
}

void WbRobotWindowTransportLayer::setTitle(const QString &title, const QString &tabbedTitle) {
  emit titleSet(title, tabbedTitle);
}

void WbRobotWindowTransportLayer::requestAck() {
  assert(mWaitingAckCount >= 0);
  mWaitingAckCount++;
}

void WbRobotWindowTransportLayer::updateWaitingAckCount() {
  mAckReceived = false;
  assert(mWaitingAckCount > 0);
  mWaitingAckCount--;
  if (mWaitingAckCount == 0)
    emit ackReceived();
}
