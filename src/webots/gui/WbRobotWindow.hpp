// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_ROBOT_WINDOW_HPP
#define WB_ROBOT_WINDOW_HPP

#include <QtCore/QObject>

#include "WbRobot.hpp"

#ifndef _WIN32
class WbRobotWindowTransportLayer;
#endif

class WbRobotWindow : public QObject {
  Q_OBJECT
public:
  explicit WbRobotWindow(WbRobot *);
 
  WbRobot *robot() { return mRobot; }
  const QString *name() { return &mRobot->window(); }
  void setupPage();
public slots:
  void sendToJavascript(const QByteArray &);
private slots:
  void runJavaScript(const QString &message);
#if defined(__APPLE__) || defined(__linux__)
  void notifyLoadCompleted();
  void notifyAckReceived();
#endif

private:
  WbRobot *mRobot;
  int mResetCount;
  bool mLoaded;
#if defined(__APPLE__) || defined(__linux__)
  QStringList mWaitingSentMessages;
  WbRobotWindowTransportLayer *mTransportLayer;
#endif
  static QString escapeString(const QString &text);
};

#endif  // WB_ROBOT_WINDOW_HPP
