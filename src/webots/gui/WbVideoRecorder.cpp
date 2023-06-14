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

#include "WbVideoRecorder.hpp"

#include "WbApplication.hpp"
#include "WbDesktopServices.hpp"
#include "WbFileUtil.hpp"
#include "WbLog.hpp"
#include "WbMainWindow.hpp"
#include "WbMessageBox.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbSimulationState.hpp"
#include "WbSimulationView.hpp"
#include "WbStandardPaths.hpp"
#include "WbVideoRecorderDialog.hpp"
#include "WbView3D.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"
#include "WbWrenLabelOverlay.hpp"

#include <QtCore/QBuffer>
#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtCore/QProcess>
#include <QtCore/QTextStream>
#include <QtCore/QThread>
#include <QtGui/QImage>
#include <QtGui/QImageWriter>
#include <QtGui/QScreen>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFileDialog>

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/types.h>
#endif

#define EXPECTED_FRAME_STEP 40  // ms (corresponding to 25 fps)

class FrameWriterThread : public QThread {
public:
  FrameWriterThread(unsigned char *frame, const QString &fileName, const QSize &resolution, int pixelRatio, int quality) :
    mFileName(fileName),
    mResolution(resolution),
    mPixelRatio(pixelRatio),
    mQuality(quality),
    mSuccess(false) {
    const int w = mResolution.width() / mPixelRatio;
    const int h = mResolution.height() / mPixelRatio;
    mFrame = new unsigned char[4 * w * h];
    WbView3D::flipAndScaleDownImageBuffer(frame, mFrame, mResolution.width(), mResolution.height(), mPixelRatio);
  }

  virtual ~FrameWriterThread() { delete[] mFrame; }

  void run() override {
    const int w = mResolution.width() / mPixelRatio;
    const int h = mResolution.height() / mPixelRatio;
    QImage img = QImage(mFrame, w, h, QImage::Format_RGB32);
    QImageWriter writer(mFileName);
    writer.setQuality(mQuality);
    mSuccess = writer.write(img);
    if (!mSuccess) {
      WbLog::warning(
        QObject::tr("Problem while saving file: '%1'").arg(QDir::toNativeSeparators(mFileName)) + "\n" + writer.errorString(),
        false);
      QString supportedFormatsLog = QObject::tr("Supported image formats:") + " ";
      QList<QByteArray> supportedFormats = QImageWriter::supportedImageFormats();
      for (int i = 0; i < supportedFormats.size(); ++i)
        supportedFormatsLog.append(QString::fromUtf8(supportedFormats[i]) + " ");
      WbLog::info(supportedFormatsLog, false);
    }
  }

  bool success() const { return mSuccess; }

private:
  unsigned char *mFrame;
  const QString mFileName;
  const QSize mResolution;
  const int mPixelRatio;
  const int mQuality;
  bool mSuccess;
};

static const QString TEMP_FRAME_FILENAME_PREFIX = "webotsFrame_";

WbVideoRecorder *WbVideoRecorder::cInstance = NULL;
WbMainWindow *WbVideoRecorder::cMainWindow = NULL;
int WbVideoRecorder::cDisplayRefresh = 1;

WbVideoRecorder *WbVideoRecorder::instance() {
  if (cInstance == NULL)
    cInstance = new WbVideoRecorder();

  return cInstance;
}

WbVideoRecorder::WbVideoRecorder() :
  mIsGraphicFeedbackEnabled(false),
  mIsInitialized(false),
  mIsFullScreen(false),
  mFrameFilePrefix(TEMP_FRAME_FILENAME_PREFIX + QString::number(QCoreApplication::applicationPid()) + "_"),
  mLastFileNumber(-1),
  mScreenPixelRatio(1),
  mVideoQuality(0),
  mVideoAcceleration(1),
  mShowCaption(false),
  mMovieFPS(25.0),
  mSimulationView(NULL),
  mScriptProcess(NULL) {
}

WbVideoRecorder::~WbVideoRecorder() {
}

bool WbVideoRecorder::initRecording(WbSimulationView *view, double basicTimeStep) {
  // show dialog to select parameters
  // compute maximum slow down with the current basicTimeStep
  WbVideoRecorderDialog dialog(NULL, view->view3D()->size(), 1.0 / ceil(EXPECTED_FRAME_STEP / basicTimeStep));
  bool accept = dialog.exec();
  if (!accept) {
    // cancel button - reject
    if (mIsInitialized) {
      // reset state
      disconnect(view->view3D(), &WbView3D::mainRenderingEnded, this, &WbVideoRecorder::requestSnapshotIfNeeded);
      disconnect(view->view3D(), &WbView3D::videoImageReady, this, &WbVideoRecorder::writeSnapshot);
      // enable window resize
      view->disableView3DFixedSize();
      WbLog::info(tr("Video creation canceled."));
    }
    return false;
  }

  // initialize recording
  return initRecording(view, basicTimeStep, dialog.resolution(), dialog.quality(), 0, dialog.acceleration(),
                       dialog.showCaption());
}

bool WbVideoRecorder::setMainWindowFullScreen(bool fullScreen) {
  bool success = false;
  if (fullScreen) {
    success = cMainWindow->setFullScreen(true, true);
    if (success)
      cMainWindow->lockFullScreen(true);
  } else {
    // first unlock, otherwise not possible
    // to switch to normal mode
    cMainWindow->lockFullScreen(false);
    success = cMainWindow->setFullScreen(false, true);
  }

  return success;
}

void WbVideoRecorder::estimateMovieInfo(double basicTimeStep) {
  const int ceilBasicTimeStep = ceil(basicTimeStep);
  const double refresh = mVideoAcceleration * EXPECTED_FRAME_STEP / (double)ceilBasicTimeStep;
  const int floorRefresh = floor(refresh);
  const int ceilRefresh = ceil(refresh);
  const int frameStep0 = floorRefresh * ceilBasicTimeStep;
  const int frameStep1 = ceilRefresh * ceilBasicTimeStep;

  if (frameStep0 == 0 || abs(frameStep0 - EXPECTED_FRAME_STEP) > abs(frameStep1 - EXPECTED_FRAME_STEP)) {
    mMovieFPS = mVideoAcceleration * 1000.0 / frameStep1;
    cDisplayRefresh = frameStep1;
  } else {
    mMovieFPS = mVideoAcceleration * 1000.0 / frameStep0;
    cDisplayRefresh = frameStep0;
  }
}

bool WbVideoRecorder::initRecording(WbSimulationView *view, double basicTimeStep, const QSize &videoResolution, int quality,
                                    int codec, double acceleration, bool caption, const QString &filename) {
  cDisplayRefresh = 1;
  mSimulationView = view;
  mVideoName = filename;
  mIsGraphicFeedbackEnabled = filename.isEmpty();

  if (mIsInitialized) {
    // reset state
    disconnect(mSimulationView->view3D(), &WbView3D::mainRenderingEnded, this, &WbVideoRecorder::requestSnapshotIfNeeded);
    disconnect(mSimulationView->view3D(), &WbView3D::videoImageReady, this, &WbVideoRecorder::writeSnapshot);

    // enable window resize
    mSimulationView->disableView3DFixedSize();
  }

  // set video parameters
  mVideoQuality = quality;
  mVideoAcceleration = acceleration;
  mVideoResolution = videoResolution;

  mShowCaption = caption;
  if (mShowCaption) {
    // add caption overlay
    WbWrenLabelOverlay *label = WbWrenLabelOverlay::createOrRetrieve(WbWrenLabelOverlay::movieCaptionOverlayId(),
                                                                     WbStandardPaths::fontsPath() + "Arial.ttf");
    label->setText(QString::number(mVideoAcceleration) + "x");
    // the resulting label text size depends on the 3D view height
    // 0.227 is an empirical value so that the label "0.01x" is fully displayed
    label->setPosition(1.0 - (0.227 * mVideoResolution.height() / mVideoResolution.width()), 0.025);
    label->setSize(0.2);
    label->setColor(0xffffff);
    label->applyChangesToWren();
  }

  // set folder where temp files are stored
  mTempDirPath = WbStandardPaths::webotsTmpPath();

  mLastFileNumber = -1;

  // remove old files
  removeOldTempFiles();

  const QScreen *screen = QGuiApplication::screenAt(QCursor::pos());
  const QSize fullScreen(screen->geometry().width(), screen->geometry().height());

  mIsFullScreen = (mVideoResolution == fullScreen);
  if (mIsFullScreen) {
    bool success = setMainWindowFullScreen(true);
    if (!success) {
      // do not start recording
      removeOldTempFiles();
      // enable window resize
      mSimulationView->disableView3DFixedSize();

      // remove caption if needed
      if (mShowCaption)
        WbWrenLabelOverlay::removeLabel(WbWrenLabelOverlay::movieCaptionOverlayId());

      WbLog::info(tr("Video creation canceled."));
      emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_READY);
      return false;
    }
  }

  // disable window resize while making movie
  mSimulationView->enableView3DFixedSize(mVideoResolution);

  // disable some menus while the movie is beeing created
  // ...

  // estimate movie parameters
  estimateMovieInfo(basicTimeStep);

  connect(mSimulationView->view3D(), &WbView3D::mainRenderingEnded, this, &WbVideoRecorder::requestSnapshotIfNeeded);
  connect(mSimulationView->view3D(), &WbView3D::videoImageReady, this, &WbVideoRecorder::writeSnapshot);
  mSimulationView->view3D()->initVideoPBO();

  if (mIsGraphicFeedbackEnabled)
    WbLog::info(tr("Video recording starts when you run a simulation..."));

  emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_RECORDING);
  mIsInitialized = true;
  return true;
}

void WbVideoRecorder::stopRecording(bool canceled) {
  disconnect(mSimulationView->view3D(), &WbView3D::mainRenderingEnded, this, &WbVideoRecorder::requestSnapshotIfNeeded);
  disconnect(mSimulationView->view3D(), &WbView3D::videoImageReady, this, &WbVideoRecorder::writeSnapshot);
  mSimulationView->view3D()->completeVideoPBOProcessing(canceled);

  // enable window resize
  mSimulationView->disableView3DFixedSize();

  // Fullscreen
  if (mIsFullScreen)
    setMainWindowFullScreen(false);

  // remove caption if needed
  if (mShowCaption)
    WbWrenLabelOverlay::removeLabel(WbWrenLabelOverlay::movieCaptionOverlayId());

  if (mLastFileNumber == -1 || canceled) {
    cancelRecording();
    emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_SIMULATION_ERROR);

    if (!canceled)
      WbMessageBox::warning("Nothing was recorded because the simulation didn't run.", cMainWindow, "Warning");

    return;
  }

  emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_SAVING);
  WbLog::info(tr("Creating video..."));

  if (mVideoName.isEmpty()) {
    // pause simulation before recording video
    WbSimulationState::Mode currentMode = WbSimulationState::instance()->mode();
    if (!WbSimulationState::instance()->isPaused())
      WbSimulationState::instance()->setMode(WbSimulationState::PAUSE);

    // ask for video name
    static QString videoFilter = ".mp4";

    QFileInfo fi(WbWorld::instance()->fileName());
    QString worldBaseName = fi.baseName();

    QString proposedFilename;
    for (int i = 0; i < 100; ++i) {
      QString suffix = i == 0 ? "" : QString("_%1").arg(i);
      proposedFilename =
        WbPreferences::instance()->value("Directories/movies").toString() + worldBaseName + suffix + videoFilter;
      if (!QFileInfo::exists(proposedFilename))
        break;
    }

    mVideoName =
      QFileDialog::getSaveFileName(cMainWindow, tr("Save Video"), WbProject::computeBestPathForSaveAs(proposedFilename),
                                   tr("Videos (*%1)").arg(videoFilter));
    if (mVideoName.isEmpty()) {
      // canceled by user
      cancelRecording();
      // reset simulation mode
      WbSimulationState::instance()->setMode(currentMode);
      return;
    }

    WbPreferences::instance()->setValue("Directories/movies", QFileInfo(mVideoName).absolutePath() + "/");
    mVideoName = QDir::toNativeSeparators(mVideoName);

    // reset simulation mode
    WbSimulationState::instance()->setMode(currentMode);
  }

  createMpeg();

  mIsInitialized = false;
}

void WbVideoRecorder::writeSnapshot(unsigned char *frame) {
  QString fileName = nextFileName();
  FrameWriterThread *thread =
    new FrameWriterThread(frame, fileName, mVideoResolution * mScreenPixelRatio, mScreenPixelRatio, mVideoQuality);
  connect(thread, &QThread::finished, this, &WbVideoRecorder::terminateSnapshotWrite);
  thread->start();
}

void WbVideoRecorder::terminateSnapshotWrite() {
  FrameWriterThread *thread = dynamic_cast<FrameWriterThread *>(sender());
  if (!thread->success())
    emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_WRITE_ERROR);
  delete thread;
}

void WbVideoRecorder::requestSnapshotIfNeeded(bool fromPhysics) {
  if (!fromPhysics)
    return;
  // request asynchronous OpenGL pixels read
  mSimulationView->view3D()->requestGrabWindowBuffer();
}

void WbVideoRecorder::terminateVideoCreation(int exitCode, QProcess::ExitStatus exitStatus) {
  // cleanup
  delete mScriptProcess;
  mScriptProcess = NULL;
  removeOldTempFiles();

  // remove script file
  QFile scriptFile(mScriptPath);
  if (scriptFile.exists()) {
    bool success = scriptFile.remove();
    if (!success)
      WbLog::warning(tr("Impossible to delete temporary file: '%1'").arg(QDir::toNativeSeparators(mScriptPath)), false);
  }

  // report exit status to user or supervisor controller
  if (exitCode > 0 || exitStatus == QProcess::CrashExit) {
    WbLog::error(tr("Video generation failed."));
    emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_ENCODING_ERROR);

    if (mIsGraphicFeedbackEnabled)
      WbMessageBox::warning(tr("Video generation failed due to an encoding problem\n"), mSimulationView, tr("Make Movie"));

    return;
  }

  QFile::remove(mVideoName);
  QDir().rename(mTempDirPath + "video.mp4", mVideoName);

  WbLog::info(tr("Video creation finished."));
  emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_READY);

  if (mIsGraphicFeedbackEnabled) {
    QCheckBox *checkBox = new QCheckBox(tr("Open containg folder and YouTube upload page."));
    QMessageBox box(cMainWindow);
    box.setWindowTitle(tr("Make Movie"));
    box.setText(tr("The movie has been created:\n%1\n\nDo you want to play it back?\n").arg(mVideoName));
    box.setIcon(QMessageBox::Icon::Question);
    box.addButton(QMessageBox::Yes);
    box.addButton(QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Cancel);
    box.setCheckBox(checkBox);
    if (box.exec() == QMessageBox::Yes) {
      if (checkBox->isChecked()) {
        WbDesktopServices::openUrl("https://www.youtube.com/upload");
        WbFileUtil::revealInFileManager(mVideoName);
      }
      WbDesktopServices::openUrl(QUrl::fromLocalFile(mVideoName).toString());
    }
  }
}

void WbVideoRecorder::cancelRecording() {
  removeOldTempFiles();
  mIsInitialized = false;
  WbLog::info(tr("Video creation canceled."));
}

void WbVideoRecorder::readStdout() {
  QByteArray bytes = mScriptProcess->readAllStandardOutput();
  QString out = QString::fromUtf8(bytes.data(), bytes.size());
  WbLog::appendStdout(out);
}

void WbVideoRecorder::readStderr() {
  QByteArray bytes = mScriptProcess->readAllStandardError();
  QString err = QString::fromUtf8(bytes.data(), bytes.size());
  WbLog::appendStdout(err);  // ffmpeg/avconv prints all the messages on stderr
}

void WbVideoRecorder::removeOldTempFiles() {
  QDir tempDir(mTempDirPath);
  if (!tempDir.exists()) {
    WbLog::error(tr("Temporary directory '%1' does not exist.").arg(QDir::toNativeSeparators(mTempDirPath)), false);
    return;
  }

  // remove all files starting with selected prefix
  QStringList nameFilters;
  nameFilters.append(mFrameFilePrefix + "*");
  QStringList tempFiles = tempDir.entryList(nameFilters);
  foreach (QString file, tempFiles) {
    bool success = tempDir.remove(file);
    if (!success) {
      WbLog::warning(tr("Impossible to delete temporary file: '%1'").arg(QDir::toNativeSeparators(file.toUtf8())), false);
    }
  }
}

QString WbVideoRecorder::nextFileName() {
  mLastFileNumber++;
  return mTempDirPath + mFrameFilePrefix + QString::asprintf("%06d", mLastFileNumber) + ".jpg";
}

void WbVideoRecorder::createMpeg() {
#ifdef __linux__
  static const QString ffmpeg("ffmpeg");
  static const QString percentageChar = "%";
  mScriptPath = "ffmpeg_script.sh";
#elif defined(__APPLE__)
  static const QString ffmpeg(QString("\"%1Contents/util/ffmpeg\"").arg(WbStandardPaths::webotsHomePath()));
  static const QString percentageChar = "%";
  mScriptPath = "ffmpeg_script.sh";
#else  // _WIN32
  static const QString ffmpeg = "ffmpeg.exe";
  static const QString percentageChar = "%%";
  mScriptPath = "ffmpeg_script.bat";
#endif

  const QString initialDir = QDir::currentPath();
  QDir::setCurrent(mTempDirPath);
  // for MPEG-4: requires ffmpeg / avconv (installed on Linux, distributed on Win32 and Mac)
  QFile ffmpegScript(mScriptPath);
  if (ffmpegScript.open(QIODevice::WriteOnly)) {
    // bitrate range between 4 and 24000000
    // cast into 'long long int' is mandatory on 32-bit machine
    long long int bitrate = (long long int)mVideoQuality * mMovieFPS * mVideoResolution.width() * mVideoResolution.height() /
                            256 / (mScreenPixelRatio * mScreenPixelRatio);

    QTextStream stream(&ffmpegScript);
#ifndef _WIN32
    stream << "#!/bin/sh\n";
    static const QString openParenthesis = "\\(";
    static const QString closeParenthesis = "\\)";
#else
    stream << "@echo off\n";
    static const QString openParenthesis = "(";
    static const QString closeParenthesis = ")";
#endif
    stream << "echo " + tr("Recording at %1 FPS, %2 bit/s.").arg(mMovieFPS).arg(bitrate) + "\n";
    stream << "echo " + tr("Video encoding stage 1... ") + openParenthesis + tr("please wait") + closeParenthesis + "\n";
    stream << ffmpeg << " -loglevel warning -y -f image2 -r " << (float)mMovieFPS << " -i \"" << mFrameFilePrefix
           << percentageChar << "06d.jpg\" -b:v " << bitrate;
    stream << " -vcodec libx264 -pass 1 -g 132 -an -pix_fmt yuvj420p video.mp4\n";
#ifdef _WIN32
    stream << "IF ERRORLEVEL 1 Exit 1\n";
#else
    stream << "rc=$?\n";
    stream << "if [ $rc != 0 ] ; then\n";
    stream << "  exit 1\n";
    stream << "fi\n";
#endif

    stream << "echo " + tr("Video encoding stage 2... ") + openParenthesis + tr("please wait") + closeParenthesis + "\n";
    stream << ffmpeg << " -loglevel warning -y -f image2 -r " << (float)mMovieFPS << " -i \"" << mFrameFilePrefix
           << percentageChar << "06d.jpg\" -b:v " << bitrate;
    stream << " -vcodec libx264 -pass 2 -g 132 -an -pix_fmt yuvj420p video.mp4\n";
#ifdef _WIN32
    stream << "IF ERRORLEVEL 1 Exit 1\n";
#else
    stream << "rc=$?\n";
    stream << "if [ $rc != 0 ] ; then\n";
    stream << "  exit 1\n";
    stream << "fi\n";
#endif

    // at the end remove log files
#ifdef _WIN32
    stream << "del ffmpeg2pass-0.log\n";
    stream << "del ffmpeg2pass-0.log.mbtree\n";
    stream << "Exit 0\n";
#else  // __APPLE__ and __linux__
    stream << "rm -f *2pass-0.log*\n";
    stream << "exit 0\n";
#endif

    stream << "echo Video Encoding complete.\n";

    // close file
    ffmpegScript.close();

    // change file properties
    QFile::setPermissions(mScriptPath, QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner);

    // run script
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("AV_LOG_FORCE_COLOR", "1");  // force output message to use ANSI Escape sequences
    mScriptProcess = new QProcess();
    mScriptProcess->setProcessEnvironment(env);
    mScriptProcess->start("./" + mScriptPath, QStringList());
    // clang-format off
    connect(mScriptProcess, (void (QProcess::*)(int, QProcess::ExitStatus)) & QProcess::finished, this,
            &WbVideoRecorder::terminateVideoCreation);
    // clang-format on
    connect(mScriptProcess, &QProcess::readyReadStandardOutput, this, &WbVideoRecorder::readStdout);
    connect(mScriptProcess, &QProcess::readyReadStandardError, this, &WbVideoRecorder::readStderr);
  } else {
    WbLog::error(tr("Impossible to write file: '%1'.").arg(mScriptPath) + "\n" + tr("Video generation failed."),
                 mIsGraphicFeedbackEnabled);
    emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_WRITE_ERROR);
  }

  QDir::setCurrent(initialDir);
}
