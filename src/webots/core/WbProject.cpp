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

#include "WbProject.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <cassert>
#include "WbFileUtil.hpp"
#include "WbLanguage.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"

static const QString WORLDS_DIR("worlds");
static const QString CONTROLLERS_DIR("controllers");
static const QString PROTOS_DIR("protos");
static const QString PLUGINS_DIR("plugins");
static const QString LIBRARIES_DIR("libraries");
static const QString PHYSICS_PLUGINS_DIR("plugins/physics");
static const QString REMOTE_CONTROL_PLUGINS_DIR("plugins/remote_controls");
static const QString ROBOT_WINDOW_PLUGINS_DIR("plugins/robot_windows");
static const QString NEW_WORLD_FILE_NAME("empty.wbt");

static QString gPreviousPath = QString();

static WbProject *gCurrentProject = NULL;
static WbProject *gSystemProject = NULL;
static WbProject *gDefaultProject = NULL;
static WbProject *gExtraDefaultProject = NULL;

void WbProject::cleanupCurrentProject() {
  delete gCurrentProject;
}

void WbProject::cleanupDefaultProject() {
  delete gDefaultProject;
}

void WbProject::cleanupExtraDefaultProject() {
  delete gExtraDefaultProject;
}

void WbProject::cleanupSystemProject() {
  delete gSystemProject;
}

WbProject *WbProject::current() {
  if (gCurrentProject == NULL) {
    gCurrentProject = new WbProject(QDir::currentPath());
    qAddPostRoutine(WbProject::cleanupCurrentProject);
  }

  return gCurrentProject;
}

WbProject *WbProject::defaultProject() {
  if (gDefaultProject == NULL) {
    gDefaultProject = new WbProject(WbStandardPaths::projectsPath() + "default/");
    qAddPostRoutine(WbProject::cleanupDefaultProject);
  }

  return gDefaultProject;
}

WbProject *WbProject::extraDefaultProject() {
  if (gExtraDefaultProject == NULL && !WbPreferences::instance()->value("General/extraProjectsPath").toString().isEmpty() &&
      QDir(WbPreferences::instance()->value("General/extraProjectsPath").toString() + "/default/").exists()) {
    gExtraDefaultProject =
      new WbProject(WbPreferences::instance()->value("General/extraProjectsPath").toString() + "/default/");
    qAddPostRoutine(WbProject::cleanupExtraDefaultProject);
  }
  return gExtraDefaultProject;
}

WbProject *WbProject::system() {
  if (gSystemProject == NULL) {
    gSystemProject = new WbProject(WbStandardPaths::resourcesProjectsPath());
    qAddPostRoutine(WbProject::cleanupSystemProject);
  }

  return gSystemProject;
}

void WbProject::setCurrent(WbProject *project) {
  delete gCurrentProject;
  gCurrentProject = project;
}

QString WbProject::projectPathFromWorldFile(const QString &fileName, bool &valid) {
  QFileInfo info(fileName);
  assert(info.suffix() == "wbt");
  QDir dir = info.absoluteDir();
  valid = dir.dirName() == "worlds";
  dir.cdUp();  // remove "worlds"
  return dir.absolutePath() + "/";
}

QString WbProject::projectNameFromWorldFile(const QString &fileName) {
  QFileInfo info(fileName);
  assert(info.suffix() == "wbt");
  QDir dir = info.absoluteDir();
  if (dir.dirName() == "worlds") {
    dir.cdUp();  // remove "worlds"
    return dir.dirName();
  }

  return "";
}

QString WbProject::computeBestPathForSaveAs(const QString &fileName) {
  QString suffix = QString("/") + QFileInfo(fileName).fileName();

  QFileInfo fileInfo(fileName);
  if (fileInfo.fileName() != fileName && fileInfo.absoluteDir().exists()) {
    if (!WbFileUtil::isLocatedInInstallationDirectory(fileName))
      return fileName;
  } else {
    QString projectPath;
    if (fileName == WbStandardPaths::unnamedWorld())
      projectPath = gPreviousPath;
    else if (WbProject::current())
      projectPath = WbProject::current()->path();

    if (!projectPath.isEmpty() && !WbFileUtil::isLocatedInInstallationDirectory(projectPath))
      return projectPath + suffix;
  }
  return QDir::homePath() + suffix;
}

WbProject::WbProject(const QString &path) {
  if (path.endsWith(".wbt")) {
    bool isValidProject = true;
    setPath(projectPathFromWorldFile(path, isValidProject));
  } else
    setPath(path);
}

WbProject::~WbProject() {
  gPreviousPath = mPath;
}

void WbProject::setPath(const QString &path) {
  if (!mPath.isEmpty())
    gPreviousPath = mPath;
  QString oldPath = mPath;
  mPath = QDir(path).absolutePath() + "/";
  emit pathChanged(oldPath, mPath);
}

QString WbProject::dirName() const {
  return QDir(mPath).dirName();
}

QString WbProject::worldsPath() const {
  return mPath + WORLDS_DIR + "/";
}

QDir WbProject::dir() const {
  return QDir(mPath);
}

QString WbProject::controllersPath() const {
  return mPath + CONTROLLERS_DIR + "/";
}

QString WbProject::librariesPath() const {
  return mPath + LIBRARIES_DIR + "/";
}

QString WbProject::protosPath() const {
  return mPath + PROTOS_DIR + "/";
}

QString WbProject::pluginsPath() const {
  return mPath + PLUGINS_DIR + "/";
}

QString WbProject::physicsPluginsPath() const {
  return mPath + PHYSICS_PLUGINS_DIR + "/";
}

QString WbProject::remoteControlPluginsPath() const {
  return mPath + REMOTE_CONTROL_PLUGINS_DIR + "/";
}

QString WbProject::robotWindowPluginsPath() const {
  return mPath + ROBOT_WINDOW_PLUGINS_DIR + "/";
}

QStringList WbProject::newProjectFiles() const {
  QStringList list;
  list << mPath;
  list << worldsPath();
  list << worldsPath() + NEW_WORLD_FILE_NAME;
  list << controllersPath();
  list << protosPath();
  list << pluginsPath();
  list << physicsPluginsPath();
  list << remoteControlPluginsPath();
  list << robotWindowPluginsPath();
  list << librariesPath();
  return list;
}

QString WbProject::newWorldFileName() {
  return NEW_WORLD_FILE_NAME;
}

bool WbProject::createNewProjectFiles(QString newWorldName) {
  QDir dir(mPath);

  // create sub dirs
  bool success = dir.mkpath(WORLDS_DIR);
  success = success && dir.mkpath(CONTROLLERS_DIR);
  success = success && dir.mkpath(PROTOS_DIR);
  success = success && dir.mkpath(PLUGINS_DIR);
  success = success && dir.mkpath(PHYSICS_PLUGINS_DIR);
  success = success && dir.mkpath(REMOTE_CONTROL_PLUGINS_DIR);
  success = success && dir.mkpath(ROBOT_WINDOW_PLUGINS_DIR);
  success = success && dir.mkpath(LIBRARIES_DIR);

  // copy new world file
  QString orig = WbStandardPaths::resourcesProjectsPath() + WORLDS_DIR + "/" + NEW_WORLD_FILE_NAME;
  QString dest = worldsPath() + NEW_WORLD_FILE_NAME;
  if (!newWorldName.isEmpty())
    dest = worldsPath() + newWorldName;
  success = success && QFile::copy(orig, dest);

  return success;
}

bool WbProject::isReadOnly() const {
#ifdef _WIN32
  Qt::CaseSensitivity caseSensitivity = Qt::CaseInsensitive;
#else
  Qt::CaseSensitivity caseSensitivity = Qt::CaseSensitive;
#endif
  return mPath.startsWith(WbStandardPaths::webotsHomePath(), caseSensitivity);
}

QString WbProject::controllerPathFromDir(const QString &dirPath) {
  if (dirPath.isEmpty())
    return QString();

  QDir controllersDir(dirPath);
  if (!controllersDir.exists())
    return QString();

  QString controllersName = controllersDir.dirName();
  QStringList fileNameFilters = WbLanguage::sourceFileExtensions();
  fileNameFilters.replaceInStrings(QRegExp("^"), controllersName);  // prepend controller name to each item

  // Search into the current controllers directory (perfect match)
  // case sensitive
  QStringList fileList = controllersDir.entryList(fileNameFilters, QDir::Files | QDir::CaseSensitive);
  if (!fileList.isEmpty())
    return controllersDir.absoluteFilePath(fileList.at(0));

  // case insensitive
  fileList = controllersDir.entryList(fileNameFilters, QDir::Files);
  if (!fileList.isEmpty())
    return controllersDir.absoluteFilePath(fileList.at(0));

  // any source file
  QStringList sourceFileFilters = WbLanguage::sourceFileExtensions();
  sourceFileFilters.replaceInStrings(QRegExp("^"), "*");                // prepend "*" to each item
  fileList = controllersDir.entryList(sourceFileFilters, QDir::Files);  // case insensitive
  if (!fileList.isEmpty())
    return controllersDir.absoluteFilePath(fileList.at(0));

  return QString();
}
