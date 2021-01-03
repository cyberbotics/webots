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
class WbStreamingServer;

class WbGuiApplication : public QApplication {
  Q_OBJECT

public:
  WbGuiApplication(int &argc, char **argv);
  virtual ~WbGuiApplication();

  int exec();
  void restart();

  enum Task { NORMAL, SYSINFO, HELP, VERSION, UPDATE_PROTO_CACHE, UPDATE_WORLD, INVALID_LOGIN, FAILURE, QUIT };
#ifdef __APPLE__
protected:
  virtual bool event(QEvent *event);
#endif

private:
  WbApplication *mApplication;
  WbSplashScreen *mSplash;
  bool mShouldMinimize;         // main window should be minimized on start
  bool mShouldStartFullscreen;  // main window should be fullscreen on start
  QString mStartWorldName;
  WbSimulationState::Mode mStartupMode;
  WbMainWindow *mMainWindow;
  bool mShouldDoRendering;

  Task mTask;
  QString mTaskArgument;

  WbStreamingServer *mStreamingServer;

  void parseArguments();
  void parseStreamArguments(const QString &streamArguments);
  void showHelp();
  void showSysInfo();
  bool setup();
  void setSplashMessage(const QString &);
  void closeSplashScreenIfNeeded();
  WbSimulationState::Mode startupModeFromPreferences() const;
  bool renderingFromPreferences() const;
  void loadInitialWorld();

  void udpateStyleSheet();
};

#endif
