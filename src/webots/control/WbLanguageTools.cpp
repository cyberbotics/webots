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

#include "WbLanguageTools.hpp"
#include "WbLog.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"
#include "WbSysInfo.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QProcess>
#include <QtCore/QStringList>
#include <QtCore/QTextStream>

#ifdef _WIN32
static const QChar PATHS_SEPARATOR(';');
#else
static const QChar PATHS_SEPARATOR(':');
#endif

static QString gJavaCommand;
static QString gMatlabCommand;

void WbLanguageTools::prependToPath(const QString &dir, QString &path) {
  if (path.isEmpty())
    path = dir;
  else
    path = dir + PATHS_SEPARATOR + path;
}

const QString &WbLanguageTools::javaCommand() {
  if (gJavaCommand.isEmpty()) {
#ifdef _WIN32
    gJavaCommand = "javaw.exe";
#else
    gJavaCommand = "java";
#endif
#ifdef __APPLE__
    // In order to run the robot window on the gui thread (thread 0)
    // which is a requirement of Qt, this option is required
    gJavaCommand += " -XstartOnFirstThread";
#endif
  }
  return gJavaCommand;
}

QString WbLanguageTools::pythonCommand(QString &shortVersion, const QString &command) {
  QString pythonCommand;
  const QString advice =
    QObject::tr("Webots requires Python version 3."
#ifdef __linux__
                "x"  // we support 3.6 on ubuntu 18.04 and 3.5 and 3.6 on ubuntu 16.04
#else
                "7"
#endif
                " or 2.7 (64 bit) from python.org in your current PATH.\n"
                "To fix the problem, you should:\n"
                "1. Check the Python command set in the Webots preferences.\n"
                "2. Check the COMMAND set in the [python] section of the runtime.ini file of your controller program if any.\n"
                "3. Fix your PATH environment variable to use the required Python 64 bit version (if available).\n"
                "4. Install the required Python 64 bit version and ensure your PATH environment variable points to it.\n");
#ifdef _WIN32
  QString windowsCommand(command);
  if (!command.endsWith(".exe", Qt::CaseInsensitive))
    windowsCommand += ".exe";
  pythonCommand = windowsCommand + " -u";
  QProcess process;
  process.start(windowsCommand, QStringList() << "-c"
                                              << "import sys;print(sys.version);print(sys.maxsize > 2**32)");
  process.waitForFinished();
  const QString output = process.readAll();
  // "3.6.3 (v3.6.3:2c5fed8, Oct  3 2017, 18:11:49) [MSC v.1900 64 bit (AMD64)]\nTrue\n" or the like
  const QStringList version = output.split("\n");
  if (!version[0].startsWith("3.7.") && !version[0].startsWith("2.7.")) {
    WbLog::warning(QObject::tr("\"%1\" was not found.\n").arg(windowsCommand) + advice);
    pythonCommand = "!";
  } else if (version.size() > 1 && version[1].startsWith("False")) {
    WbLog::warning(QObject::tr("\"%1\" 64 bit was not found, but the 32 bit version was found.\n").arg(windowsCommand) +
                   advice);
    pythonCommand = "!";
  } else
    shortVersion = QString(version[0][0]) + version[0][2];
#else  // macOS and Linux
  pythonCommand = command + " -u";
  QProcess process;
  process.start(command, QStringList() << "-c"
                                       << "import sys;print(sys.version);");
  process.waitForFinished();
  const QString output = process.readAll();
  // "2.7.6 (default, Nov 23 2017, 15:49:48) \n[GCC 4.8.4]\n" or the like
  const QStringList version = output.split(" ");
  if (!version[0].startsWith("3.") && !version[0].startsWith("2.7.")) {
    WbLog::warning(QObject::tr("\"%1\" was not found.\n").arg(command) + advice);
    pythonCommand = "!";
  } else
    shortVersion = QString(version[0][0]) + version[0][2];
#endif
  return pythonCommand;
}

const QString &WbLanguageTools::matlabCommand() {
  if (gMatlabCommand.isEmpty()) {
    QString arguments = " -nosplash -nodesktop";
#ifdef _WIN32
    // minimize option is only supported on Windows
    // http://www.mathworks.ch/ch/help/matlab/matlab_env/startup-options.html
    arguments += " -minimize";
    // on Windows there are two MATLAB .exe files, one is located in
    // bin/matlab.exe and the other one in bin/win64/MATLAB.exe.
    // bin/matlab.exe is normally in the PATH, but we must call bin/win64/MATLAB.exe
    // because bin/matlab.exe is just a launcher that causes problem with stdout/stderr
    // and with the termination of the QProcess.
    QString PATH = qgetenv("PATH");
    QStringList dirs = PATH.split(';', QString::SkipEmptyParts);
    bool matlabFound = false;
    foreach (QString dir, dirs) {
      if (dir.contains("matlab", Qt::CaseInsensitive)) {
        matlabFound = QDir(dir).exists();
        QString file = dir + "\\win64\\MATLAB.exe";
        if (QFile::exists(file)) {
          gMatlabCommand = '"' + file + '"' + arguments;
          break;
        }
      }
    }
    if (gMatlabCommand.isEmpty()) {
      if (matlabFound)
        WbLog::warning(QObject::tr("To run Matlab controllers, you need to install a 64-bit version of Matlab."));
      else
        WbLog::warning(QObject::tr("To run Matlab controllers, you need to install Matlab 64-bit and ensure it is available "
                                   "from the DOS CMD.EXE console."));
      gMatlabCommand = "!";
    }
#else

#ifdef __linux__
    if (WbSysInfo::isPointerSize64bits())
      arguments += " -glnxa64";
    else
      arguments += " -glnx86";
#endif

    gMatlabCommand = "matlab " + arguments;
#endif
  }

  return gMatlabCommand;
}
