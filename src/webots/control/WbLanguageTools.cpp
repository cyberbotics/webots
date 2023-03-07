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

#include <cstdlib>

#ifdef _WIN32
static const QChar PATHS_SEPARATOR(';');
#else
static const QChar PATHS_SEPARATOR(':');
#endif

static QString gJavaCommand;

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
  }
  return gJavaCommand;
}

const QStringList WbLanguageTools::javaArguments() {
#ifdef __APPLE__
  // In order to run the robot window on the gui thread (thread 0)
  // which is a requirement of Qt, this option is required
  return QStringList("-XstartOnFirstThread");
#else
  return QStringList();
#endif
}

QString WbLanguageTools::pythonCommand(QString &shortVersion, const QString &command, QProcessEnvironment &env) {
  QString pythonCommand = command;
  if (pythonCommand.isEmpty())
#ifdef _WIN32
    pythonCommand = "python";
#else
    pythonCommand = "python3";
#endif
  const QString advice =
#ifdef __APPLE__
    "To fix the problem, you should set the full path of your python command in "
    "Webots->preferences->python command.\n";
#else
    QObject::tr("Webots requires Python version 3.7 or newer in your current PATH.\n"
                "To fix the problem, you should:\n"
                "1. Check the Python command set in the Webots preferences.\n"
                "2. Check the COMMAND set in the [python] section of the runtime.ini file of your controller program if any.\n"
                "3. Install a recent Python 64 bit version and ensure your PATH environment variable points to it.\n");
#endif
#ifdef _WIN32
  if (!command.endsWith(".exe", Qt::CaseInsensitive))
    pythonCommand += ".exe";
  QProcess process;
  process.setProcessEnvironment(env);
  process.start(pythonCommand, QStringList() << "-u"
                                             << "-c"
                                             << "import sys;print(sys.version);print(sys.maxsize > 2**32)");
  process.waitForFinished();
  const QString output = process.readAll();
  // "3.6.3 (v3.6.3:2c5fed8, Oct  3 2017, 18:11:49) [MSC v.1900 64 bit (AMD64)]\nTrue\n" or the like
  const QStringList version = output.split("\n");
  const int v = (QString(version[0][2]) + (version[0][3] != '.' ? QString(version[0][3]) : "")).toInt();
  if (!version[0].startsWith("3.") || v < 7) {
    WbLog::warning(QObject::tr("\"%1\" was not found.\n").arg(pythonCommand) + advice);
    pythonCommand = "!";
  } else if (version.size() > 1 && version[1].startsWith("False")) {
    WbLog::warning(QObject::tr("\"%1\" 64 bit was not found, but the 32 bit version was found.\n").arg(pythonCommand) + advice);
    pythonCommand = "!";
  } else
    shortVersion = QString(version[0][0]) + version[0][2];
  if (version[0][3] != '.')
    shortVersion += version[0][3];  // handle versions 310, 311, 321, etc.
#elif __APPLE__
  if (std::getenv("PWD"))
    shortVersion = checkIfPythonCommandExist(pythonCommand, env, true);
  else if (pythonCommand == "python" || pythonCommand == "python3") {
    for (int minorVersion = 11; minorVersion >= 7; minorVersion--) {
      const QString versionString = QString::number(minorVersion);
      const QString fullVersionString = "3." + versionString;
      pythonCommand = findWorkingPythonPath(fullVersionString, env, false);
      if (pythonCommand != "!") {
        shortVersion = QString("3") + versionString;
        break;
      }
    }
  } else if (pythonCommand.startsWith("python3.")) {
    pythonCommand = findWorkingPythonPath(pythonCommand.mid(6), env, true);
    shortVersion = QString("3") + pythonCommand[8];
    if (pythonCommand.length() > 9 && pythonCommand[9] != '.')
      shortVersion += pythonCommand[9];
  } else
    shortVersion = checkIfPythonCommandExist(pythonCommand, env, true);
  if (shortVersion.isEmpty())
    pythonCommand = "!";

  if (pythonCommand == "!")
    WbLog::warning(QObject::tr("Python was not found.\n") + advice);
#else  // __linux__
      shortVersion = checkIfPythonCommandExist(pythonCommand, env, true);
  if (shortVersion.isEmpty()) {
    pythonCommand = "!";
    WbLog::warning(QObject::tr("Python was not found.\n") + advice);
  }

#endif
  return pythonCommand;
}

#if defined __APPLE__ || defined __linux__
const QString WbLanguageTools::checkIfPythonCommandExist(const QString &pythonCommand, QProcessEnvironment &env, bool log) {
  QString shortVersion;
  QProcess process;
  process.setProcessEnvironment(env);
  process.start(pythonCommand, QStringList() << "-c"
                                             << "import sys;print(sys.version);");
  process.waitForFinished();
  const QString output = process.readAll();
  // "3.8.10 (tags/v3.8.10:3d8993a, May  3 2021, 11:48:03) [MSC v.1928 64 bit (AMD64)]" or the like
  const QStringList version = output.split(" ");
  if (!version[0].startsWith("3.")) {
    if (log)
      WbLog::warning(QObject::tr("\"%1\" was not found.\n").arg(pythonCommand));
    shortVersion = QString();
  } else {
    const QStringList version_numbers(version[0].split("."));
    shortVersion = version_numbers[0] + version_numbers[1];
  }
  return shortVersion;
}
#endif

#ifdef __APPLE__
QString WbLanguageTools::findWorkingPythonPath(const QString &pythonVersion, QProcessEnvironment &env, bool log) {
  QString shortVersion;

  // look for python from python.org
  QString pythonCommandString =
    "/Library/Frameworks/Python.framework/Versions/" + pythonVersion + "/bin/python" + pythonVersion;
  shortVersion = checkIfPythonCommandExist(pythonCommandString, env, false);
  if (shortVersion.isEmpty()) {
    // look first possible path for python from homebrew
    pythonCommandString = "/usr/local/opt/python@" + pythonVersion + " /bin/python" + pythonVersion;
    shortVersion = checkIfPythonCommandExist(pythonCommandString, env, false);
    if (shortVersion.isEmpty()) {
      // look a second possible path for python from homebrew
      pythonCommandString = "/usr/local/bin/python" + pythonVersion;
      shortVersion = checkIfPythonCommandExist(pythonCommandString, env, log);
      if (shortVersion.isEmpty())
        pythonCommandString = "!";
    }
  }

  return pythonCommandString;
}
#endif

const QStringList WbLanguageTools::pythonArguments() {
  return QStringList("-u");
}

QString WbLanguageTools::matlabCommand() {
#ifdef __APPLE__
  const QString matlabPath = "/Applications/";
  const QString matlabAppWc = "MATLAB_R20???.app";
  const QDir matlabDir(matlabPath);
  const QStringList matlabVersions = matlabDir.entryList(QStringList(matlabAppWc), QDir::Dirs, QDir::Name);
  if (matlabVersions.isEmpty())
    return "";
#else
  const QString matlabVersionsWc = "R20???";
#ifdef _WIN32
  const QString matlabPath = "C:\\Program Files\\MATLAB\\";
  const QString matlabExecPath = "\\bin\\matlab.exe";
#else  // __linux__
  const QString matlabPath = "/usr/local/MATLAB/";
  // cppcheck-suppress unreadVariable
  const QString matlabExecPath = "/bin/matlab";
#endif
  const QDir matlabDir(matlabPath);
  if (!matlabDir.exists()) {
    return "";
  }
  const QStringList matlabVersions = matlabDir.entryList(QStringList(matlabVersionsWc), QDir::Dirs, QDir::Name);
#endif

  QString command = matlabPath + matlabVersions.last();
#if defined _WIN32 || defined __linux__
  command += matlabExecPath;
#endif

  return command;
}

const QStringList WbLanguageTools::matlabArguments() {
  QStringList arguments("");
#ifdef __linux__
  arguments << (WbSysInfo::isPointerSize64bits() ? "-glnxa64" : "-glnx86");
#endif
  return arguments;
}
