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

#ifndef WB_VIDEO_RECORDER_HPP
#define WB_VIDEO_RECORDER_HPP

//
// Description: class managing the make movie functionality
//

#include <QtCore/QDir>
#include <QtCore/QObject>
#include <QtCore/QProcess>
#include <QtCore/QSize>

class WbSimulationView;
class WbMainWindow;

class WbVideoRecorder : public QObject {
  Q_OBJECT
public:
  // singleton: avoid multiple simultaneous recordings
  static WbVideoRecorder *instance();

  static void setMainWindow(WbMainWindow *mainWindow) { cMainWindow = mainWindow; }

  static int displayRefresh() { return cDisplayRefresh; }

  void setScreenPixelRatio(int ratio) { mScreenPixelRatio = ratio; }

  // initialize recording parameters by GUI
  bool initRecording(WbSimulationView *view, double basicTimeStep);

  // initialize recording parameters
  // quality between 1 and 100,
  // winCompression: compression used in windows AVI creation
  bool initRecording(WbSimulationView *view, double basicTimeStep, const QSize &videoResolution, int quality, int codec = 0,
                     double acceleration = 1.0, bool caption = false, const QString &filename = "");

  bool isRecording() const { return mIsInitialized; }

signals:
  void videoCreationStatusChanged(int status);
  void requestOpenUrl(const QString &fileName, const QString &message, const QString &title);

public slots:
  void stopRecording(bool canceled = false);

private:
  static WbVideoRecorder *cInstance;

  bool mIsGraphicFeedbackEnabled;
  bool mIsInitialized;
  bool mIsFullScreen;

  QString mScriptPath;
  QString mTempDirPath;
  QString mFrameFilePrefix;
  QString mTempVideoName;
  QString mVideoName;
  int mLastFileNumber;

  QSize mVideoResolution;
  int mScreenPixelRatio;
  int mVideoQuality;
  double mVideoAcceleration;
  bool mShowCaption;
  double mMovieFPS;
  static int cDisplayRefresh;

  WbSimulationView *mSimulationView;
  static WbMainWindow *cMainWindow;

  QProcess *mScriptProcess;

  WbVideoRecorder();
  virtual ~WbVideoRecorder();
  void cancelRecording();
  void removeOldTempFiles();
  QString nextFileName();
  bool setMainWindowFullScreen(bool fullScreen);
  void createMpeg();
  void estimateMovieInfo(double basicTimeStep);

private slots:
  void requestSnapshotIfNeeded(bool fromPhysics);
  void writeSnapshot(unsigned char *frame);
  void terminateSnapshotWrite();
  void terminateVideoCreation(int exitCode, QProcess::ExitStatus exitStatus);
  void readStderr();
  void readStdout();
};

#endif
