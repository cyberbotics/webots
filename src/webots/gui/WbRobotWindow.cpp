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

#include "WbDesktopServices.hpp"
#include "WbLog.hpp"
#include "WbPreferences.hpp"

WbRobotWindow::WbRobotWindow(WbRobot *robot) : mRobot(robot) {
}

void WbRobotWindow::setupPage() {
  QString windowFileName = mRobot->windowFile("html");
  const QString port = WbPreferences::instance()->value("port", 1234).toString();
  if (windowFileName.isEmpty()) {
    mRobot->parsingWarn(tr("No HTML robot window is set in the 'window' field."));
    return;
  }
  windowFileName = windowFileName.mid(windowFileName.indexOf("/robot_windows"));  // remove content before robot_windows

  openOnWebBrowser("http://localhost:" + port + windowFileName + "?name=" + mRobot->name(),
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

bool WbRobotWindow::openOnWebBrowser(const QString &url, const QString &program, const bool newBrowserWindow) {
  QString systemProgram;
  QStringList arguments;

#ifdef _WIN32
  // The TEMP/TMP environment variables set my the MSYS2 console are confusing Visual C++ (among possibly other apps)
  // as they refer to "/tmp" which is not a valid Windows path. It is therefore safer to remove them
  const QByteArray TEMP = qgetenv("TEMP");
  const QByteArray TMP = qgetenv("TMP");
  qunsetenv("TMP");
  qunsetenv("TEMP");
  if (program.isEmpty())
    return openUrl(url);

  systemProgram = "cmd";
  arguments << "/Q"
            << "/C"
            << "\"start " + program + "\"";
#elif __linux__
  if (program.isEmpty())
    return WbDesktopServices::openUrl(url);

  systemProgram = program;
#else
  if (program.isEmpty())
    return openUrl(url);

  systemProgram = "open";  // set argument
  arguments << "-a " + program;
#endif

  QProcess mCurrentProcess;
  mCurrentProcess.setProgram(systemProgram);
  qDebug() << systemProgram << arguments;
  if (newBrowserWindow)
    mCurrentProcess.setArguments(arguments << "-new-window" << url);
  else
    mCurrentProcess.setArguments(arguments << url);
  /*   mCurrentProcess.setStandardErrorFile(QProcess::nullDevice());
    mCurrentProcess.setStandardOutputFile(QProcess::nullDevice()); */
  qDebug() << systemProgram << arguments;
  bool result = mCurrentProcess.startDetached();
  if (!result) {
    WbLog::warning(QObject::tr("Failed to open web browser: %1. Opening robot window in default browser.").arg(program));
    result = WbDesktopServices::openUrl(url);
  }
#ifdef _WIN32
  qputenv("TEMP", TEMP);
  qputenv("TMP", TMP);
#endif
  return result;
}