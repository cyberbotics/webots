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

#ifndef WB_PROJECT_HPP
#define WB_PROJECT_HPP

//
// Description: project structure and paths
//

#include <QtCore/QDir>
#include <QtCore/QList>
#include <QtCore/QString>

class WbProject : public QObject {
  Q_OBJECT

public:
  // singleton: for the current Webots project
  static WbProject *current();
  static void setCurrent(WbProject *project);

  // the special "system" project: "WEBOTS_HOME/resources/projects"
  static WbProject *system();

  // the special "default" project: "WEBOTS_HOME/projects/default"
  static WbProject *defaultProject();

  // extra projects defined in the preferences or 'WEBOTS_EXTRA_PROJECT_PATH'
  static QList<WbProject *> *extraProjects();

  // e.g. /home/yvan/project/worlds/ghostdog.wbt -> /home/yvan/project
  static QString projectPathFromWorldFile(const QString &fileName, bool &valid);
  static QString projectNameFromWorldFile(const QString &fileName);
  static QString computeBestPathForSaveAs(const QString &fileName);

  explicit WbProject(const QString &path);
  virtual ~WbProject();

  // the project's root path
  void setPath(const QString &path);

  // return the slash-terminated absolute project path
  const QString &path() const { return mPath; }
  QString dirName() const;
  QDir dir() const;

  // true if located in WEBOTS_HOME
  bool isReadOnly() const;

  // get absolute paths
  QString worldsPath() const;
  QString controllersPath() const;
  QString protosPath() const;
  QString pluginsPath() const;
  QString librariesPath() const;
  QString physicsPluginsPath() const;
  QString remoteControlPluginsPath() const;
  QString robotWindowPluginsPath() const;

  // create files for new project
  bool createNewProjectFolders();
  QStringList newProjectFiles() const;
  static QString newWorldPath();

  // e.g. "<absolute path to project>/controllers/four_wheel/"
  //       -> "<absolute path to project>/controllers/four_wheel/four_wheel.c"
  QString controllerPathFromDir(const QString &dirPath);

signals:
  void pathChanged(const QString &oldPath, const QString &newPath);

private:
  QString mPath;

  static void cleanupCurrentProject();
  static void cleanupDefaultProject();
  static void cleanupExtraProjects();
  static void cleanupSystemProject();
};

#endif
