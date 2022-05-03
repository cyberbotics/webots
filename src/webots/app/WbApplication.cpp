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

#include "WbApplication.hpp"

#include "WbAnimationRecorder.hpp"
#include "WbApplicationInfo.hpp"
#include "WbBoundingSphere.hpp"
#include "WbControlledWorld.hpp"
#include "WbLog.hpp"
#include "WbNodeOperations.hpp"
#include "WbParser.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoList.hpp"
#include "WbSimulationState.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbSysInfo.hpp"
#include "WbTelemetry.hpp"
#include "WbTokenizer.hpp"
#include "WbWorld.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QElapsedTimer>

#include <cassert>

WbApplication *WbApplication::cInstance = NULL;
static QString gProjectLibsInPath;

WbApplication::WbApplication() {
  assert(cInstance == NULL);
  cInstance = this;

  mWorld = NULL;
  mWorldLoadingCanceled = false;
  mWorldLoadingProgressDialogCreated = false;

  // create the Webots temporary path early in the process
  // in order to be sure that the Qt internal files will be stored
  // at the right place
  WbStandardPaths::webotsTmpPath();

  WbPreferences::createInstance("Cyberbotics", "Webots", WbApplicationInfo::version());

  mStartupPath = QDir::currentPath();

  // compute WEBOTS_HOME without trailing "/"
  QString WEBOTS_HOME(QDir::toNativeSeparators(WbStandardPaths::webotsHomePath()));
  WEBOTS_HOME.chop(1);

  // Webots must execute in its home directory
  QDir::setCurrent(WEBOTS_HOME);

  // needed for Webots controllers
  qputenv("WEBOTS_HOME", WEBOTS_HOME.toUtf8());

#ifdef _WIN32
  // On Windows, if Webots is started from a DOS console or from the
  // Windows desktop, we need to remove the path to msys\1.0\bin (if any)
  // to prevent the Makefile to use the mkdir.exe and rmdir.exe provided
  // by MSYS which conflict with the corresponding DOS commands (same
  // name but different syntax) and cause the Makefile to fail.
  // If Webots is started from MSYS, we shouldn't remove this path.
  QString MSYSTEM(qgetenv("MSYSTEM"));
  QString TERM(qgetenv("TERM"));
  if (MSYSTEM != "MINGW32" && TERM != "cygwin") {  // we are in the DOS or Windows Desktop case, not MSYS
    QString path(qgetenv("Path"));
    QString newPath(path);
    while (1) {
      const int i = newPath.indexOf("\\msys\\1.0\\bin", 0, Qt::CaseInsensitive);
      if (i == -1)
        break;
      int j = 0;
      for (j = i; j > 0; j--) {
        if (newPath[j] == ':')
          break;  // Volume separator
      }
      j--;  // points to volume name (e.g., "C")
      newPath = newPath.mid(0, j) + newPath.mid(i + 14);
    }
    qputenv("Path", QByteArray(newPath.toUtf8()));
  }
#endif

  qputenv("WEBOTS_DISABLE_BINARY_COPY", "True");
}

WbApplication::~WbApplication() {
  delete mWorld;
  WbPreferences::cleanup();
  WbNodeOperations::cleanup();
  cInstance = NULL;

  // remove links to project dynamic libraries
  removeOldLibraries();

  // remove temporary folder
  QDir tmpDir(WbStandardPaths::webotsTmpPath());
  tmpDir.removeRecursively();
}

void WbApplication::setup() {
  WbNodeOperations *nodeOperations = WbNodeOperations::instance();

  // create and connect WbAnimationRecorder
  WbAnimationRecorder *recorder = WbAnimationRecorder::instance();
  connect(recorder, &WbAnimationRecorder::animationStartStatusChanged, this, &WbApplication::animationStartStatusChanged);
  connect(recorder, &WbAnimationRecorder::animationStopStatusChanged, this, &WbApplication::animationStopStatusChanged);
  connect(this, &WbApplication::animationCaptureStarted, recorder, &WbAnimationRecorder::start);
  connect(this, &WbApplication::animationCaptureStopped, recorder, &WbAnimationRecorder::stop);
  connect(nodeOperations, &WbNodeOperations::nodeAdded, recorder, &WbAnimationRecorder::propagateNodeAddition);
  connect(this, &WbApplication::deleteWorldLoadingProgressDialog, this,
          &WbApplication::setWorldLoadingProgressDialogCreatedtoFalse);
}

void WbApplication::removeOldLibraries() {
#ifdef _WIN32
  // remove previous project lib folders from PATH
  QString PATH(qgetenv("PATH"));
  PATH.remove(gProjectLibsInPath);
  qputenv("PATH", PATH.toUtf8());
#else  // __linux__ || __APPLE__
  QDir tmpLibDir(WbStandardPaths::webotsTmpPath() + "lib/");

  // remove links to libraries of previous project
  if (tmpLibDir.exists()) {
    const QStringList &files = tmpLibDir.entryList(QDir::Files | QDir::NoDotAndDotDot);
    foreach (const QString fileName, files)
      tmpLibDir.remove(fileName);
  }
#endif
}

void WbApplication::linkLibraries(QString projectLibrariesPath) {
  // remove previous links
  removeOldLibraries();

  if (projectLibrariesPath.startsWith(WbStandardPaths::resourcesProjectsPath()))
    // do not link resources libraries
    return;

  QString projectLibPath(QDir::toNativeSeparators(projectLibrariesPath));

#ifdef _WIN32
  // add project lib folder and subfolders to the PATH
  QDir projectLibDir(projectLibPath);
  QStringList libDirs = projectLibDir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
  if (projectLibDir.exists() && !libDirs.isEmpty()) {
    QString PATH(projectLibPath);
    foreach (QString libDirName, libDirs)
      PATH += ";" + projectLibPath + libDirName;

    gProjectLibsInPath = PATH;
    QString PATH_BEFORE(qgetenv("PATH"));
    if (!PATH_BEFORE.isEmpty()) {
      PATH += ";" + PATH_BEFORE;
      gProjectLibsInPath += ";";
    }
    qputenv("PATH", PATH.toUtf8());
  }
#else  // __linux__ || __APPLE__
  const QString tmpLibPath(WbStandardPaths::webotsTmpPath() + "lib/");
  const QString dynamicLibraryExtension(WbStandardPaths::dynamicLibraryExtension());

  QDirIterator iterator(projectLibPath, QDirIterator::Subdirectories);
  bool success = false;
  QString suffix, fileName, filePath;
  while (iterator.hasNext()) {
    iterator.next();

    if (dynamicLibraryExtension == ("." + iterator.fileInfo().suffix())) {
      filePath = iterator.fileInfo().absoluteFilePath();
      fileName = iterator.fileName();

      if (QFile::exists(tmpLibPath + fileName))
        continue;

      // create soft link of dynamic library in the project lib folder
      success = QFile::link(filePath, tmpLibPath + fileName);
      if (!success)
        WbLog::error(tr("Could not create a symbolic link of dynamic library: '%1'.").arg(fileName));
    }
  }
#endif
}

void WbApplication::setWorldLoadingProgress(const int progress) {
  if (!mWorldLoadingProgressDialogCreated) {
    // more than 2 seconds that world is loading
    emit createWorldLoadingProgressDialog();
    mWorldLoadingProgressDialogCreated = true;
  }
  emit worldLoadingHasProgressed(progress);
}

void WbApplication::setWorldLoadingStatus(const QString &status) {
  if (!mWorldLoadingProgressDialogCreated) {
    // more than 2 seconds that world is loading
    emit createWorldLoadingProgressDialog();
    mWorldLoadingProgressDialogCreated = true;
  }
  emit worldLoadingStatusHasChanged(status);
}

void WbApplication::setWorldLoadingCanceled() {
  mWorldLoadingCanceled = true;
  emit worldLoadingWasCanceled();
}

void WbApplication::setWorldLoadingProgressDialogCreatedtoFalse() {
  mWorldLoadingProgressDialogCreated = false;
}

bool WbApplication::wasWorldLoadingCanceled() const {
  return mWorldLoadingCanceled;
}

bool WbApplication::cancelWorldLoading(bool loadEmptyWorld, bool deleteWorld) {
  emit deleteWorldLoadingProgressDialog();

  if (deleteWorld) {
    delete mWorld;
    mWorld = NULL;
  }
  if (loadEmptyWorld)
    return loadWorld(WbStandardPaths::emptyProjectPath() + "worlds/" + WbProject::newWorldFileName(), false);
  return false;
}

bool WbApplication::isValidWorldFileName(const QString &worldName) {
  QFileInfo worldNameInfo(worldName);
  if (!worldNameInfo.exists() || !worldNameInfo.isFile() || !worldNameInfo.isReadable()) {
    WbLog::error(tr("Could not open file: '%1'.").arg(worldName));
    return false;
  }
  if (QString::compare(worldNameInfo.suffix(), "wbt", Qt::CaseInsensitive) != 0) {
    WbLog::error(tr("Could not open file: '%1'. The world file extension must be '.wbt'.").arg(worldName));
    return false;
  }
  return true;
}

bool WbApplication::loadWorld(QString worldName, bool reloading) {
  printf("#########################################\nWbApplication::loadWorld()\n");

  printf("loading: %s\n", worldName.toUtf8().constData());
  WbProtoList::current()->resetCurrentProjectProtoList();
  if (!WbProtoList::current()->areProtoAssetsAvailable(worldName)) {
    printf("> some proto assets are missing, downloading them.\n");
    WbProtoList::current()->retrieveAllExternProto(worldName, reloading);
    return false;  // when all extern proto are downloaded, loadWorld is called again
  }

  printf("> proto assets available, begin load\n");
  WbProtoList::current()->printCurrentProjectProtoList();

  mWorldLoadingCanceled = false;
  mWorldLoadingProgressDialogCreated = false;

  WbNodeOperations::instance()->enableSolidNameClashCheckOnNodeRegeneration(false);

  worldName = QDir::cleanPath(worldName);

  QString fileName;
  if (WbPreferences::instance()->value("General/telemetry").toBool()) {
    QFileInfo fi(worldName);
    fileName = fi.fileName();
    const QDir dir = fi.absoluteDir();
    const QString &WEBOTS_HOME = WbStandardPaths::webotsHomePath();
    const QString truncatedFilePath = dir.canonicalPath().mid(0, WEBOTS_HOME.length());
    if (truncatedFilePath.compare(WEBOTS_HOME, Qt::CaseInsensitive) == 0)
      WbTelemetry::send("trial", fileName);  // log attempt to open world file
    else
      fileName = "";
  }
  const bool useTelemetry = WbPreferences::instance()->value("General/telemetry").toBool() && !fileName.isEmpty();

  bool isValidProject = true;
  QString newProjectPath = WbProject::projectPathFromWorldFile(worldName, isValidProject);
  // WbProtoList *protoList = new WbProtoList(isValidProject ? newProjectPath + "protos" : "");
  WbProtoList *protoList = WbProtoList::current();

  setWorldLoadingStatus(tr("Reading world file "));
  if (wasWorldLoadingCanceled()) {
    delete protoList;
    if (useTelemetry)
      WbTelemetry::send("cancel");
    return cancelWorldLoading(true);
  }

  WbTokenizer tokenizer;
  int errors = tokenizer.tokenize(worldName);
  if (errors) {
    WbLog::error(tr("'%1': Failed to load due to invalid token(s).").arg(worldName));
    delete protoList;
    if (useTelemetry)
      WbTelemetry::send("cancel");
    return cancelWorldLoading(false);
  }

  setWorldLoadingStatus(tr("Parsing world"));
  if (wasWorldLoadingCanceled()) {
    delete protoList;
    if (useTelemetry)
      WbTelemetry::send("cancel");
    return cancelWorldLoading(true);
  }

  WbParser parser(&tokenizer);
  if (!parser.parseWorld(worldName)) {
    WbLog::error(tr("'%1': Failed to load due to syntax error(s).").arg(worldName));
    delete protoList;
    if (useTelemetry)
      WbTelemetry::send("cancel");
    return cancelWorldLoading(true);
  }

  emit preWorldLoaded(reloading);

  bool isFirstLoad = (mWorld == NULL);
  delete mWorld;

  if (wasWorldLoadingCanceled()) {
    delete protoList;
    if (useTelemetry)
      WbTelemetry::send("cancel");
    return cancelWorldLoading(true, true);
  }

  WbBoundingSphere::enableUpdates(false);

  // the world takes ownership of the proto list
  WbProject::setCurrent(new WbProject(newProjectPath));
  linkLibraries(WbProject::current()->librariesPath());
  mWorld = new WbControlledWorld(protoList, &tokenizer);
  if (mWorld->wasWorldLoadingCanceled()) {
    if (useTelemetry)
      WbTelemetry::send("cancel");
    return cancelWorldLoading(true, true);
  }
  WbSimulationState::instance()->setEnabled(true);

  WbNodeOperations::instance()->updateDictionary(true, mWorld->root());

  emit postWorldLoaded(reloading, isFirstLoad);

  emit deleteWorldLoadingProgressDialog();

  WbNodeOperations::instance()->enableSolidNameClashCheckOnNodeRegeneration(true);
  WbBoundingSphere::enableUpdates(WbSimulationState::instance()->isRayTracingEnabled(), mWorld->root()->boundingSphere());

  if (useTelemetry)
    WbTelemetry::send("success");  // confirm the file previously sent was opened successfully

  return true;
}

void WbApplication::takeScreenshot(const QString &fileName, int quality) {
  emit requestScreenshot(fileName, quality);
}

void WbApplication::simulationQuit(int exitStatus) {
  emit simulationQuitRequested(exitStatus);
}

void WbApplication::worldReload() {
  emit worldReloadRequested();
}

void WbApplication::simulationReset(bool restartControllers) {
  WbWorld::instance()->reset(restartControllers);
  emit simulationResetRequested(restartControllers);
}

void WbApplication::startVideoCapture(const QString &fileName, int type, int width, int height, int quality, int acceleration,
                                      bool showCaption) {
  emit videoCaptureStarted(fileName, type, width, height, quality, acceleration, showCaption);
}

void WbApplication::stopVideoCapture() {
  emit videoCaptureStopped(false);
}

void WbApplication::startAnimationCapture(const QString &fileName) {
  emit animationCaptureStarted(fileName);
}

void WbApplication::stopAnimationCapture() {
  emit animationCaptureStopped();
}

void WbApplication::resetPhysics() {
  foreach (WbSolid *solid, WbWorld::instance()->topSolids())
    solid->resetPhysics();
}
