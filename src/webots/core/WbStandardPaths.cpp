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

#include "WbStandardPaths.hpp"

#include "WbApplicationInfo.hpp"
#include "WbLog.hpp"
#include "WbPreferences.hpp"
#include "WbSimulationState.hpp"
#include "WbSysInfo.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QProcess>
#include <QtCore/QStandardPaths>
#include <QtCore/QString>
#include <QtCore/QTextStream>
#include <QtCore/QTimer>

#ifdef _WIN32
#include "../../../include/controller/c/webots/utils/system.h"
#endif

const QString &WbStandardPaths::webotsHomePath() {
  static QString path;
#ifdef __linux__
  // on Linux, the webots binary is located in $WEBOTS_HOME/bin/webots-bin
  const int depth = 1;
#elif defined(__APPLE__)
  // on macOS, the webots binary is located in $WEBOTS_HOME/Contents/MacOS/webots
  const int depth = 2;
#else
  // on Windows, the webots binary is located in $WEBOTS_HOME/msys64/mingw64/bin/webots
  const int depth = 3;
#endif
  if (path.isEmpty()) {
    QDir dir(QCoreApplication::applicationDirPath());
    for (int i = 0; i < depth; i++)
      dir.cdUp();
    path = dir.absolutePath() + "/";
  }
  return path;
};

#ifdef __APPLE__
static const QString cMacOsContents = "Contents/";
#else
static const QString cMacOsContents;
#endif

const QString &WbStandardPaths::webotsLibPath() {
  static QString path = webotsHomePath() + cMacOsContents + "lib/webots/";
  return path;
}

const QString &WbStandardPaths::controllerLibPath() {
  static QString path = webotsHomePath() + cMacOsContents + "lib/controller/";
  return path;
}

#ifdef _WIN32
const QString &WbStandardPaths::webotsMsys64Path() {
  static QString path = webotsHomePath() + "msys64/";
  return path;
}
#endif

const QString &WbStandardPaths::localDocPath() {
  static QString url(webotsHomePath() + cMacOsContents + "docs/");
  return url;
};

const QString &WbStandardPaths::projectsPath() {
  static QString path(webotsHomePath() + cMacOsContents + "projects/");
  return path;
};

const QString &WbStandardPaths::resourcesPath() {
#ifdef __APPLE__
  static QString path(webotsHomePath() + "Contents/Resources/");
#else
  static QString path(webotsHomePath() + "resources/");
#endif
  return path;
};

const QString &WbStandardPaths::translationsPath() {
  static QString path(resourcesPath() + "translations/");
  return path;
};

const QString &WbStandardPaths::templatesPath() {
  static QString path(resourcesPath() + "templates/");
  return path;
};

const QString &WbStandardPaths::fontsPath() {
  static QString path(resourcesPath() + "fonts/");
  return path;
};

const QString &WbStandardPaths::resourcesPicoPath() {
  static QString path(resourcesPath() + "pico/lang/");
  return path;
};

const QString &WbStandardPaths::resourcesProjectsPath() {
  static QString path(resourcesPath() + "projects/");
  return path;
};

const QString &WbStandardPaths::resourcesControllersPath() {
  static QString path(resourcesProjectsPath() + "controllers/");
  return path;
};

const QString &WbStandardPaths::resourcesPhysicsPluginsPath() {
  static QString path(resourcesProjectsPath() + "plugins/physics/");
  return path;
};

const QString &WbStandardPaths::resourcesSoundPluginsPath() {
  static QString path(resourcesProjectsPath() + "plugins/sound/");
  return path;
};

const QString &WbStandardPaths::resourcesRobotWindowsPluginsPath() {
  static QString path(resourcesProjectsPath() + "plugins/robot_windows/");
  return path;
}

const QString &WbStandardPaths::resourcesWebPath() {
  static QString path(resourcesPath() + "web/");
  return path;
}

const QString &WbStandardPaths::cyberboticsUrl() {
  static QString url("https://cyberbotics.com");
  return url;
};

const QString &WbStandardPaths::githubRepositoryUrl() {
  static const QString url("https://github.com/cyberbotics/webots");
  return url;
};

const QString &WbStandardPaths::dynamicLibraryExtension() {
#ifdef __APPLE__
  static QString extension(".dylib");
#elif defined(_WIN32)
  static QString extension(".dll");
#else  // __linux__
  static QString extension(".so");
#endif
  return extension;
}

const QString &WbStandardPaths::dynamicLibraryPrefix() {
#ifdef _WIN32
  static QString suffix("");
#else
  static QString suffix("lib");
#endif
  return suffix;
}

const QString &WbStandardPaths::executableExtension() {
#ifdef _WIN32
  static QString extension(".exe");
#else
  static QString extension("");
#endif
  return extension;
}

const QString &WbStandardPaths::emptyProjectPath() {
  if (qgetenv("WEBOTS_EMPTY_PROJECT_PATH").isEmpty())
    return resourcesProjectsPath();

  static QString path;
  if (path.isEmpty())
    path = QDir(qgetenv("WEBOTS_EMPTY_PROJECT_PATH")).absolutePath() + "/";
  return path;
}

const QString &WbStandardPaths::unnamedTextFile() {
  static QString fileName("unnamed.txt");
  return fileName;
};

static void liveWebotsTmpPath() {
  QFile file(WbStandardPaths::webotsTmpPath() + "live.txt");
  if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
    QTextStream out(&file);
    out << QDateTime::currentSecsSinceEpoch();
    file.close();
  }
}

static QString cWebotsTmpPath;
static int cWebotsTmpPathId = -1;

bool WbStandardPaths::webotsTmpPathCreate(const int id) {
  assert(cWebotsTmpPathId == -1 && cWebotsTmpPath.isEmpty());  // we should create it once
#ifdef _WIN32
  // We do not use QDir::tempPath() as it relies on the TEMP/TMP environment variables which are overriden by the MSYS2
  // console to C:\msys2\tmp whereas the libController uses the LOCALAPPDATA version, e.g., C:\Users\user\AppData\Local\Temp
  cWebotsTmpPath =
    QDir::fromNativeSeparators(WbSysInfo::environmentVariable("LOCALAPPDATA")) + QString("/Temp/webots-%1/").arg(id);
#else
  QString username = qgetenv("USER");
  if (username.isEmpty()) {
    username = qgetenv("USERNAME");
    if (username.isEmpty()) {
      WbLog::error(QObject::tr("USER or USERNAME environment variable not set, falling back to 'default' username."));
      username = "default";
    }
  }
#if defined(__APPLE__)
  cWebotsTmpPath = QString("/tmp/webots/%1/%2/").arg(username).arg(id);
#else  // __linux__
  const QString WEBOTS_TMPDIR = WbSysInfo::environmentVariable("WEBOTS_TMPDIR");
  if (!WEBOTS_TMPDIR.isEmpty() && QDir(WEBOTS_TMPDIR).exists())
    cWebotsTmpPath = QString("%1/webots/%2/%3/").arg(WEBOTS_TMPDIR).arg(username).arg(id);
  else {
    cWebotsTmpPath = QString("/tmp/webots/%1/%2/").arg(username).arg(id);
    WbLog::error(QObject::tr("Webots has not been started regularly. Some features may not work. "
                             "Please start Webots from its launcher."));
  }
#endif
#endif
  // cleanup old and unused tmp directories
  QDir directory(cWebotsTmpPath);
  directory.cdUp();
  const QStringList &webotsTmp = directory.entryList(QStringList() << "webots-*", QDir::Dirs | QDir::Writable);
  foreach (const QString &dirname, webotsTmp) {
    const QString fullName(directory.absolutePath() + "/" + dirname);
    const QFileInfo fileInfo(fullName + "/live.txt");
    const QDateTime &lastModified = fileInfo.fileTime(QFileDevice::FileModificationTime);
    const qint64 diff = lastModified.secsTo(QDateTime::currentDateTime());
    if (diff > 3600) {  // if the live.txt file was not modified for more than one hour, delete the tmp folder
      QDir d(fullName);
      d.removeRecursively();
    }
  }

  // create the required tmp directories
  QDir dir(cWebotsTmpPath);
  if (!dir.exists() && !dir.mkpath("."))
    return false;

  // write a new live.txt file in the webots tmp folder every hour to prevent any other webots process to delete it
  static QTimer timer;
  liveWebotsTmpPath();
  QTimer::connect(&timer, &QTimer::timeout, liveWebotsTmpPath);
  timer.start(30 * 60 * 1000);  // call every 30 minutes
  cWebotsTmpPathId = id;
  return true;
}

int WbStandardPaths::webotsTmpPathId() {
  return cWebotsTmpPathId;
}

const QString &WbStandardPaths::webotsTmpPath() {
  return cWebotsTmpPath;
}

const QString &WbStandardPaths::cachedAssetsPath() {
  static QString path(QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + "/assets/");
  return path;
}

const QString &WbStandardPaths::vehicleLibraryPath() {
  static QString path(webotsHomePath() + cMacOsContents + "projects/default/libraries/vehicle/");
  return path;
}
