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
#include "WbPreferences.hpp"

WbRobotWindow::WbRobotWindow(WbRobot *robot) : mRobot(robot) {
}

void WbRobotWindow::setupPage() {
  QString windowFileName = mRobot->windowFile("html");
  const QString port = WbPreferences::instance()->value("Streaming/port", 1234).toString();
  if (windowFileName.isEmpty()) {
    mRobot->parsingWarn(tr("No HTML robot window is set in the 'window' field."));
    return;
  }
  windowFileName = windowFileName.mid(windowFileName.indexOf("/robot_windows"));  // remove content before robot_windows

  WbDesktopServices::openUrlWithArgs("http://localhost:" + port + windowFileName + "?name=" + mRobot->name(),
                                     WbPreferences::instance()->value("RobotWindow/browser").toString(),
                                     WbPreferences::instance()->value("RobotWindow/newBrowserWindow").toBool());
}

void WbRobotWindow::setClientID(const QString &clientID, const QString &robotName, const QString &socketStatus) {
  if (robotName == mRobot->name() && socketStatus == "connected") {
    mClientID = clientID;
    emit socketOpened();
  } else if (clientID == mClientID && socketStatus == "disconnected")
    mClientID = "";
}
