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
#include "WbDesktopServices.hpp"
#include "WbLog.hpp"
#include "WbPreferences.hpp"

#include <QtCore/QUrl>

WbRobotWindow::WbRobotWindow(WbRobot *robot, WbMainWindow *mainWindow) : mRobot(robot), mMainWindow(mainWindow) {
}

void WbRobotWindow::setupPage() {
  QString windowFileName = mRobot->windowFile("html");
  const QString port = WbPreferences::instance()->value("Streaming/port", 1234).toString();
  if (windowFileName.isEmpty()) {
    mRobot->parsingWarn(QString("No HTML robot window is set in the 'window' field."));
    return;
  }
  windowFileName = windowFileName.mid(windowFileName.indexOf("/robot_windows"));  // remove content before robot_windows
  QString completeURL = "http://localhost:" + port + windowFileName + "?name=" + mRobot->name();

  QString browserProgram = WbPreferences::instance()->value("RobotWindow/browser").toString();
  bool newBrowserWindow = WbPreferences::instance()->value("RobotWindow/newBrowserWindow").toBool();
  bool success = false;
  if (!browserProgram.isEmpty()) {
    success = WbDesktopServices::openUrlWithArgs(completeURL, browserProgram, newBrowserWindow);
    if (!success)
      WbLog::warning(tr("Failed to open web browser: %1. Open robot window in default browser.").arg(browserProgram));
  }
  if (!success)
    WbDesktopServices::openUrlWithArgs(completeURL, "", newBrowserWindow);
}

void WbRobotWindow::setClientID(const QString &clientID, const QString &robotName, const QString &socketStatus) {
  if (robotName == mRobot->name() && socketStatus == "connected") {
    mClientID = clientID;
    emit socketOpened();
  } else if (clientID == mClientID && socketStatus == "disconnected")
    mClientID = "";
}