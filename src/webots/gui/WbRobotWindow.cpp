// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbRobotWindow.hpp"

#include <QtCore/qdebug.h>
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>

WbRobotWindow::WbRobotWindow(WbRobot *robot) : mRobot(robot) {
}

void WbRobotWindow::setupPage() {
  QString windowFileName = mRobot->windowFile("html");
  if (windowFileName.isEmpty()) {
    mRobot->parsingWarn(QString("No dockable HTML robot window is set in the 'window' field."));
    return;
  }
  windowFileName = windowFileName.mid(windowFileName.indexOf("/robot_windows"));  // remove content before robot_windows
  QDesktopServices::openUrl(QUrl("http://localhost:1234" + windowFileName + "?name=" + mRobot->name()));
}

void WbRobotWindow::setClientID(QString clientID, QString socketStatus) {
  if ((mClientID == "") && (socketStatus == "connected")) {
    mClientID = clientID;
    qDebug() << "new connection: " << mClientID;
  } else if ((clientID == mClientID) && (socketStatus == "disconnected")) {
    qDebug() << "delete connection: " << mClientID;
    mClientID = "";
  } else
    qDebug() << "already one client" << mClientID << "unknown client:" << clientID;
}