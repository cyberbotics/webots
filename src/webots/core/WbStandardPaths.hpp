// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_STANDARD_PATHS_HPP
#define WB_STANDARD_PATHS_HPP

class QString;

namespace WbStandardPaths {
  // directories, every path is terminated by a /
  const QString &webotsHomePath();     // e.g. /usr/local/webots/
  const QString &webotsLibPath();      // e.g. /usr/local/webots/lib/webots/
  const QString &controllerLibPath();  // e.g. /usr/local/webots/lib/controller/
#ifdef _WIN32
  const QString &webotsMsys64Path();  // e.g. C:/Program Files/Webots/msys64/
#endif
  const QString &localDocPath();                      // e.g. /usr/local/webots/docs/
  const QString &projectsPath();                      // e.g. /usr/local/webots/projects/
  const QString &resourcesPath();                     // e.g. /usr/local/webots/resources/
  const QString &translationsPath();                  // e.g. /usr/local/webots/resources/translations/
  const QString &templatesPath();                     // e.g. /usr/local/webots/resources/templates/
  const QString &fontsPath();                         // e.g. /usr/local/webots/resources/fonts/
  const QString &resourcesPicoPath();                 // e.g. /usr/local/webots/resources/pico/lang/
  const QString &resourcesProjectsPath();             // e.g. /usr/local/webots/resources/projects/
  const QString &resourcesControllersPath();          // e.g. /usr/local/webots/resources/projects/controllers/
  const QString &resourcesPhysicsPluginsPath();       // e.g. /usr/local/webots/resources/projects/plugins/physics/
  const QString &resourcesSoundPluginsPath();         // e.g. /usr/local/webots/resources/projects/plugins/sound/
  const QString &resourcesRobotWindowsPluginsPath();  // e.g. /usr/local/webots/resources/projects/plugins/robot_windows/
  const QString &resourcesWebPath();                  // e.g. /usr/local/webots/resources/web/

  const QString &emptyProjectPath();  // equal to resourcesProjectsPath() if WEBOTS_EMPTY_PROJECT_PATH is not set

  // urls
  const QString &cyberboticsUrl();       // https://cyberbotics.com
  const QString &githubRepositoryUrl();  // https://github.com/cyberbotics/webots

  // utility function
  const QString &dynamicLibraryExtension();  // e.g. .so, .dll or .dylib
  const QString &dynamicLibraryPrefix();     // e.g. lib or ''
  const QString &executableExtension();      // e.g. .exe or ''

  // file names
  const QString &unnamedWorld();     // "unnamed.wbt"
  const QString &unnamedTextFile();  // "unnamed.txt"

  // temporary directory
  const QString &webotsTmpPath();  // e.g. /var/tmp/webots/ or /var/tmp/webots-<PID>/
};                                 // namespace WbStandardPaths

#endif
