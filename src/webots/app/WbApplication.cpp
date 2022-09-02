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
#include "WbProtoManager.hpp"
#include "WbSimulationState.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbSysInfo.hpp"
#include "WbTelemetry.hpp"
#include "WbTokenizer.hpp"
#include "WbVersion.hpp"
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

void WbApplication::setWorldLoadingProgress(const int progress) {
  static int previousProgress = 0;
  if (progress == previousProgress)
    return;
  previousProgress = progress;
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

void WbApplication::cancelWorldLoading(bool loadEmpty, bool deleteWorld) {
  emit deleteWorldLoadingProgressDialog();

  if (deleteWorld) {
    delete mWorld;
    mWorld = NULL;
  }

  WbLog::setConsoleLogsPostponed(false);
  WbLog::showPendingConsoleMessages();

  if (loadEmpty)
    loadWorld(WbProject::newWorldPath(), false);
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

void WbApplication::loadWorld(QString worldName, bool reloading, bool isLoadingAfterDownload) {
  bool isValidProject = true;
  const QString newProjectPath = WbProject::projectPathFromWorldFile(worldName, isValidProject);
  WbProject::setCurrent(new WbProject(newProjectPath));

  // decisive load signal should come from WbProtoManager (to ensure all assets are available)
  if (!isLoadingAfterDownload) {
    WbProtoManager::instance()->retrieveExternProto(worldName, reloading);
    return;
  }

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

  setWorldLoadingStatus(tr("Reading world file "));
  if (wasWorldLoadingCanceled()) {
    if (useTelemetry)
      WbTelemetry::send("cancel");
    cancelWorldLoading(true);
    return;
  }

  WbTokenizer tokenizer;
  const int errors = tokenizer.tokenize(worldName);
  if (errors > 0) {
    WbLog::error(tr("'%1': Failed to load due to invalid token(s).").arg(worldName));
    if (useTelemetry)
      WbTelemetry::send("cancel");
    cancelWorldLoading(false);
    return;
  }

  setWorldLoadingStatus(tr("Parsing world"));
  if (wasWorldLoadingCanceled()) {
    if (useTelemetry)
      WbTelemetry::send("cancel");
    cancelWorldLoading(true);
    return;
  }

  WbParser parser(&tokenizer);
  if (!parser.parseWorld(worldName)) {
    WbLog::error(tr("'%1': Failed to load due to syntax error(s).").arg(worldName));
    if (useTelemetry)
      WbTelemetry::send("cancel");
    cancelWorldLoading(true);
    return;
  }

  emit preWorldLoaded(reloading);

  bool isFirstLoad = (mWorld == NULL);
  delete mWorld;

  if (wasWorldLoadingCanceled()) {
    if (useTelemetry)
      WbTelemetry::send("cancel");
    cancelWorldLoading(true, true);
    return;
  }

  WbBoundingSphere::enableUpdates(false);

  mWorld = new WbControlledWorld(&tokenizer);
  if (mWorld->wasWorldLoadingCanceled()) {
    if (useTelemetry)
      WbTelemetry::send("cancel");
    cancelWorldLoading(true, true);
    return;
  }

  WbSimulationState::instance()->setEnabled(true);

  WbNodeOperations::instance()->updateDictionary(true, mWorld->root());

  emit postWorldLoaded(reloading, isFirstLoad);

  emit deleteWorldLoadingProgressDialog();

  WbNodeOperations::instance()->enableSolidNameClashCheckOnNodeRegeneration(true);
  WbBoundingSphere::enableUpdates(WbSimulationState::instance()->isRayTracingEnabled(), mWorld->root()->boundingSphere());

  if (useTelemetry)
    WbTelemetry::send("success");  // confirm the file previously sent was opened successfully

  emit worldLoadCompleted();
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
