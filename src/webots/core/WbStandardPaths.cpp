// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbStandardPaths.hpp"

#include "WbApplicationInfo.hpp"
#include "WbLog.hpp"
#include "WbSysInfo.hpp"
#include "WbWebotsInstancesCounter.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QStandardPaths>
#include <QtCore/QString>

#ifdef _WIN32
#include "../../../include/controller/c/webots/utils/system.h"
#endif

const QString &WbStandardPaths::webotsDataPath() {
  static const QString dataPath = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/";
  return dataPath;
}

const QString &WbStandardPaths::webotsHomePath() {
  static QString path;
#ifdef __linux__
  // on Linux,    the webots binary is located in $WEBOTS_HOME/bin/webots-bin
  const int depth = 1;
#elif defined(__APPLE__)
  // on macOS, the webots binary is located in $WEBOTS_HOME/Contents/MacOS/webots-bin
  const int depth = 2;
#else
  // on Windows,  the webots binary is located in $WEBOTS_HOME/msys64/mingw64/bin/webots
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

const QString &WbStandardPaths::webotsLibPath() {
  static QString path = webotsHomePath() + "lib/";
  return path;
}

#ifdef _WIN32
const QString &WbStandardPaths::webotsMsys64Path() {
  static QString path = webotsHomePath() + "msys64/";
  return path;
}
#endif

const QString &WbStandardPaths::localDocPath() {
  static QString url(webotsHomePath() + "docs/");
  return url;
};

const QString &WbStandardPaths::projectsPath() {
  static QString path(webotsHomePath() + "projects/");
  return path;
};

const QString &WbStandardPaths::resourcesPath() {
  static QString path(webotsHomePath() + "resources/");
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
  static QString url("https://www.cyberbotics.com");
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
  path = qgetenv("WEBOTS_EMPTY_PROJECT_PATH");
  return path;
}

const QString &WbStandardPaths::unnamedWorld() {
  static QString fileName("unnamed.wbt");
  return fileName;
};

const QString &WbStandardPaths::unnamedTextFile() {
  static QString fileName("unnamed.txt");
  return fileName;
};

const QString &WbStandardPaths::webotsTmpPath() {
  static QString webotsTmpPath;
  if (webotsTmpPath.isEmpty()) {
#ifdef _WIN32
    webotsTmpPath = QDir::tempPath() + QString("/webots-%1/").arg(QCoreApplication::applicationPid());
#elif defined(__APPLE__)
    webotsTmpPath = QString("/var/tmp/webots-%1/").arg(QCoreApplication::applicationPid());
#else  // __linux__
    const QString WEBOTS_TMP_PATH = WbSysInfo::environmentVariable("WEBOTS_TMP_PATH");
    if (!WEBOTS_TMP_PATH.isEmpty() && QDir(WEBOTS_TMP_PATH).exists())
      webotsTmpPath = WEBOTS_TMP_PATH;
    else {
      WbLog::error(QObject::tr(
        "Webots has not been started regularly. Some features may not work. Please start Webots from its launcher."));
      webotsTmpPath = QString("/tmp/webots-%1/").arg(QCoreApplication::applicationPid());
    }
#endif

    // remove the other Webots instances from the temp folder if needed
    if (WbWebotsInstancesCounter::numberOfInstances() == 1) {
      QDir tmpDir(webotsTmpPath);
      tmpDir.cdUp();

      tmpDir.setFilter(QDir::Dirs);
      QStringList filters;
      filters << "webots-*";
      tmpDir.setNameFilters(filters);

      QFileInfoList list = tmpDir.entryInfoList();
      for (int i = 0; i < list.size(); ++i) {
        QDir dirToRemove(list.at(i).absoluteFilePath());
        dirToRemove.removeRecursively();
      }
    }

    // create the required tmp directories
    QDir dir(webotsTmpPath);
    if (!dir.exists()) {
      if (!dir.mkpath("."))
        WbLog::fatal(QObject::tr("Cannot create the Webots temporary directory \"%1\"").arg(webotsTmpPath));
#ifndef _WIN32
      if (!dir.mkpath("lib"))  // used for the controller libraries
        WbLog::fatal(QObject::tr("Cannot create a directory in the Webots temporary directory \"%1\"").arg(webotsTmpPath));
#endif
    }
  }
  return webotsTmpPath;
}
