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

#include <QtCore/QProcess>
#if __APPLE__
#include <QtCore/QRandomGenerator>
#endif

#include "WbDesktopServices.hpp"
#include "WbFileUtil.hpp"
#include "WbLog.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"

WbRobotWindow::WbRobotWindow(WbRobot *robot) : mRobot(robot) {
}

void WbRobotWindow::setupPage(int port) {
  QString windowFileName = mRobot->windowFile("html");
  if (windowFileName.isEmpty()) {
    mRobot->parsingWarn(tr("No HTML robot window is set in the 'window' field."));
    return;
  }

  // if the file is located in Webots installation directory, the WEBOTS_HOME part is replaced by "/~/" in the absolute path
  // if the file is located at another place, only the relative path is kept
  if (WbFileUtil::isLocatedInInstallationDirectory(windowFileName, true))
    windowFileName = "/~WEBOTS_HOME" + windowFileName.mid(WbStandardPaths::webotsHomePath().length() - 1);
  else
    windowFileName = windowFileName.mid(windowFileName.indexOf("/robot_windows"));

  openOnWebBrowser("http://localhost:" + QString::number(port) + windowFileName + "?name=" + mRobot->name(),
                   WbPreferences::instance()->value("RobotWindow/browser").toString(),
                   WbPreferences::instance()->value("RobotWindow/newBrowserWindow").toBool());
}

void WbRobotWindow::setClientID(const QString &clientID, const QString &robotName, const QString &socketStatus) {
  if (robotName == mRobot->name() && socketStatus == "connected") {
    mClientID = clientID;
    emit socketOpened();
  } else if (clientID == mClientID && socketStatus == "disconnected")
    mClientID = "-1";
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
    return WbDesktopServices::openUrl(url);

  systemProgram = "cmd";
  arguments << "/Q"
            << "/C"
            << "start" << program;
#elif __linux__
  if (program.isEmpty())
    return WbDesktopServices::openUrl(url);

  systemProgram = program;
#elif __APPLE__
  if (program.isEmpty())
    // safari fails to open the same url that has just been closed, generate random number to fix the issue.
    return WbDesktopServices::openUrl(url + '#' + QString::number(QRandomGenerator::global()->generate()));

  systemProgram = "open";  // set argument
  arguments << "-a" + program;
#endif

  QProcess currentProcess;
  bool success = false;

  currentProcess.setProgram(systemProgram);
  if (newBrowserWindow)
    currentProcess.setArguments(arguments << "-new-window" << url);
  else
    currentProcess.setArguments(arguments << url);

  QString error;
#ifdef __linux__
  currentProcess.setStandardErrorFile(QProcess::nullDevice());
  currentProcess.setStandardOutputFile(QProcess::nullDevice());
  success = currentProcess.startDetached();
  error = tr("Cannot start %1.").arg(systemProgram);
#else
  currentProcess.start();
  if (currentProcess.waitForFinished()) {
    error = currentProcess.readAllStandardError().trimmed();
    success = error.isEmpty();
  }
#endif
#ifdef _WIN32
  qputenv("TEMP", TEMP);
  qputenv("TMP", TMP);
#endif

  if (!success) {
    WbLog::warning(
      tr("Unable to open web browser program: %1. %2 Opening robot window in default browser.").arg(program).arg(error));
    success = WbDesktopServices::openUrl(url);
  }

  return success;
}
