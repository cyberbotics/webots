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

#include "WbControllerPlugin.hpp"
#include "WbFileUtil.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtCore/QString>
#include <QtCore/QStringList>

static QString gTypeNames[2] = {
  "robot_windows",
  "remote_controls",
};

static void searchPossibleControllerPlugins(QStringList &out, const QStringList &plugins, WbControllerPlugin::Type type) {
  foreach (const QString &pluginPath, plugins) {
    QDir dir(pluginPath + gTypeNames[type]);
    if (dir.exists()) {
      QStringList subDirs = dir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
      foreach (const QString &subDir, subDirs) {
        QString filename = dir.absolutePath() + '/' + subDir + '/' + WbStandardPaths::dynamicLibraryPrefix() + subDir +
                           WbStandardPaths::dynamicLibraryExtension();
        out << filename;
      }
    }
  }
}

const QStringList &WbControllerPlugin::defaultList(Type type) {
  static QStringList lists[2];
  static bool firstCall = true;
  if (firstCall) {
    firstCall = false;

    QStringList pluginsList;
    WbFileUtil::searchDirectoryNameRecursively(pluginsList, "plugins", WbStandardPaths::projectsPath() + "default/");
    WbFileUtil::searchDirectoryNameRecursively(pluginsList, "plugins", WbStandardPaths::resourcesProjectsPath());
    foreach (const WbProject *extraProject, *WbProject::extraProjects())
      WbFileUtil::searchDirectoryNameRecursively(pluginsList, "plugins", extraProject->path());

    searchPossibleControllerPlugins(lists[ROBOT_WINDOW], pluginsList, ROBOT_WINDOW);
    lists[ROBOT_WINDOW].sort();

    searchPossibleControllerPlugins(lists[REMOTE_CONTROL], pluginsList, REMOTE_CONTROL);
    lists[REMOTE_CONTROL].sort();
  }
  return lists[type];
}

WbControllerPlugin::Type WbControllerPlugin::pluginSubDirectoryToType(const QString &pluginSubDirectory) {
  if (pluginSubDirectory == "remote_controls")
    return REMOTE_CONTROL;
  else
    return ROBOT_WINDOW;
}
