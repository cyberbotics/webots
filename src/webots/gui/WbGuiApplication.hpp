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

#ifndef WB_GUI_APPLICATION_HPP
#define WB_GUI_APPLICATION_HPP

//
// Description: Webots main gui application
//

#include <QtWidgets/QApplication>

#include "WbSimulationState.hpp"

class WbApplication;
class WbMainWindow;
class WbSplashScreen;
class WbTcpServer;
class WbSingleTaskApplication;

class WbGuiApplication : public QApplication {
  Q_OBJECT

public:
  WbGuiApplication(int &argc, char **argv);
  virtual ~WbGuiApplication();

  int exec();
  void restart();
  static void setWindowsDarkMode(QWidget *);

  enum Task { NORMAL, SYSINFO, HELP, VERSION, UPDATE_WORLD, INVALID_LOGIN, FAILURE, QUIT, CONVERT };
  WbApplication *application() const { return mApplication; };

protected:
#ifdef __APPLE__
  virtual bool event(QEvent *event) override;
#endif
  void timerEvent(QTimerEvent *event) override;

private:
  WbApplication *mApplication;
  WbSplashScreen *mSplash;
  bool mShouldMinimize;         // main window should be minimized on start
  bool mShouldStartFullscreen;  // main window should be fullscreen on start
  QString mStartWorldName;
  WbSimulationState::Mode mStartupMode;
  WbMainWindow *mMainWindow;
  bool mShouldDoRendering;
  QString mThemeLoaded;
  char mStream;
  int mHeartbeat;

  Task mTask;
  QStringList mTaskArguments;

  WbTcpServer *mTcpServer;

  void commandLineError(const QString &message, bool fatal = true);
  void parseArguments();
  void showHelp();
  void showSysInfo();
  bool setup();
  void setSplashMessage(const QString &);
  void closeSplashScreenIfNeeded();
  WbSimulationState::Mode startupModeFromPreferences() const;
  bool renderingFromPreferences() const;
  void loadInitialWorld();
  void updateStyleSheet();
  const WbSingleTaskApplication *taskExecutor();
};

#endif
