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

#ifndef WB_ROBOT_WINDOW_TRANSPORT_LAYER
#define WB_ROBOT_WINDOW_TRANSPORT_LAYER

#include <QtCore/QObject>

// cppcheck-suppress noConstructor
class WbRobotWindowTransportLayer : public QObject {
  // This WebChannel object is used to send JS data from the robot window to WbRobotWindow.cpp.
  Q_OBJECT
  Q_PROPERTY(bool ackReceived MEMBER mAckReceived NOTIFY waitingAckCountChanged FINAL)

public:
  explicit WbRobotWindowTransportLayer(QObject *parent = NULL);
  void requestAck();

public slots:
  void receiveFromJavascript(const QByteArray &javascript);
  void setTitle(const QString &title, const QString &tabbedTitle);

signals:
  void javascriptReceived(const QByteArray &javascript);
  void titleSet(const QString &title, const QString &tabbedTitle);
  void ackReceived();
  void waitingAckCountChanged();

private slots:
  void updateWaitingAckCount();

private:
  bool mAckReceived;
  int mWaitingAckCount;
};

#endif
