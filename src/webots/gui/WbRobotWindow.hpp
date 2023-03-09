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

#ifndef WB_ROBOT_WINDOW_HPP
#define WB_ROBOT_WINDOW_HPP

#include <QtCore/QObject>

#include "WbRobot.hpp"

class WbRobotWindow : public QObject {
  Q_OBJECT
public:
  explicit WbRobotWindow(WbRobot *);

  WbRobot *robot() { return mRobot; }
  const QString getClientID() { return mClientID; }
  void setupPage(int port);

public slots:
  void setClientID(const QString &clientID, const QString &robotName, const QString &socketStatus);
signals:
  void socketOpened();

private:
  WbRobot *mRobot;
  QString mClientID;

  bool openOnWebBrowser(const QString &url, const QString &program, const bool newBrowserWindow);
};

#endif  // WB_ROBOT_WINDOW_HPP
