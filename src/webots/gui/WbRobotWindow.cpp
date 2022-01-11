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

#include "WbApplicationInfo.hpp"
#include "WbLog.hpp"
#include "WbProject.hpp"
#include "WbRobot.hpp"
#include "WbRobotWindowTransportLayer.hpp"
#include "WbStandardPaths.hpp"
#include "WbVersion.hpp"
#include "WbWebPage.hpp"

#include <QtCore/qdebug.h>
#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtCore/QRegularExpression>
#include <QtCore/QTextStream>
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#include <QtWebSockets/QWebSocket>

WbRobotWindow::WbRobotWindow(WbRobot *robot) : mRobot(robot) {
  QString title = "Robot: " + robot->name();

  const QString &windowFileName = robot->windowFile("html");
  if (windowFileName.isEmpty()) {
    robot->parsingWarn(QString("No dockable HTML robot window is set in the 'window' field."));
    return;
  }

  connect(robot, &WbRobot::sendToJavascript, this, &WbRobotWindow::sendToJavascript);
  connect(robot, &WbRobot::controllerChanged, this, &WbRobotWindow::setupPage);
}

void WbRobotWindow::setupPage() {
  QString windowFileName = mRobot->windowFile("html");
  if (windowFileName.isEmpty()) {
    mRobot->parsingWarn(QString("No dockable HTML robot window is set in the 'window' field."));
    return;
  }
  windowFileName = windowFileName.mid(windowFileName.indexOf("/robot_windows"));  // remove content before robot_windows

  qDebug() << "windowFileName:"
           << "http://localhost:1234" + windowFileName;
  QDesktopServices::openUrl(QUrl("http://localhost:1234" + windowFileName));
  startControllerIfNeeded();  // TODO1: not sure it is needed
  // connect(mTransportLayer, &WbRobotWindowTransportLayer::javascriptReceived, mRobot, &WbRobot::receiveFromJavascript);
}

void WbRobotWindow::sendToJavascript(const QByteArray &string) {
  // qDebug() << "runJavaScript Robotwin:" << string;
  return;
}

void WbRobotWindow::startControllerIfNeeded() {
  if (!mRobot->isControllerStarted())
    mRobot->startController();
  mRobot->updateControllerWindow();
}
