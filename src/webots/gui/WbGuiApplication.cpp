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

#ifdef _WIN32
#include <dwmapi.h>
#include <windows.h>
#include <QtGui/QWindow>
#endif

#include "WbGuiApplication.hpp"

#include "WbApplication.hpp"
#include "WbApplicationInfo.hpp"
#include "WbConsole.hpp"
#include "WbMainWindow.hpp"
#include "WbMessageBox.hpp"
#include "WbMultimediaStreamingServer.hpp"
#include "WbNetwork.hpp"
#include "WbNewVersionDialog.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbSingleTaskApplication.hpp"
#include "WbSplashScreen.hpp"
#include "WbStandardPaths.hpp"
#include "WbSysInfo.hpp"
#include "WbTranslator.hpp"
#include "WbVersion.hpp"
#include "WbW3dStreamingServer.hpp"
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QProcess>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtCore/QUrl>
#include <QtGui/QFontDatabase>
#include <QtGui/QScreen>

#ifdef __APPLE__
#include <QtGui/QFileOpenEvent>
#endif

#include <iostream>

using namespace std;

// QApplication needs the reference to the original argc directly to run properly.
// Otherwise, bugs related with threads (splashscreen, tooltips, ...) can appear.
// This was observed on Linux 64
// cf:
// - http://lists-archives.org/kde-devel/20232-qt-4-5-related-crash-on-kdm-startup.html
// - http://www.qtcentre.org/archive/index.php/t-28785.html
WbGuiApplication::WbGuiApplication(int &argc, char **argv) :
  QApplication(argc, argv),
  mMainWindow(NULL),
  mHeartbeat(0),
  mTask(NORMAL),
  mTcpServer(NULL) {
  setApplicationName("Webots");
  setApplicationVersion(WbApplicationInfo::version().toString(true, false, true));
  setOrganizationName("Cyberbotics");
  setOrganizationDomain("cyberbotics.com");
#ifdef _WIN32
  QApplication::setStyle("windowsvista");
#endif

  mApplication = new WbApplication();  // creates WbApplication singleton
  connect(mApplication, &WbApplication::createWorldLoadingProgressDialog, this, &WbGuiApplication::closeSplashScreenIfNeeded);
  // translation settings is the first thing to do in order
  // to have all the messages in the right language
  // (even the first failure messages)
  // most recently installed translation file is searched for translations first
  installTranslator(WbTranslator::instance()->basicTranslator());
  installTranslator(WbTranslator::instance()->translator());

  QDir::addSearchPath("icons", WbStandardPaths::resourcesPath() + "nodes/icons");
  QDir::addSearchPath("images", WbStandardPaths::resourcesPath() + "images");

  QFontDatabase::addApplicationFont(WbStandardPaths::fontsPath() + "Raleway-Light.ttf");

  // setup the stylesheet for the application
  updateStyleSheet();

  // Qt has its own arguments, see Qt doc
  mShouldMinimize = false;
  mShouldStartFullscreen = false;
  mStartupMode = WbSimulationState::NONE;
  mShouldDoRendering = true;

  parseArguments();
}

WbGuiApplication::~WbGuiApplication() {
  delete mMainWindow;
}

void WbGuiApplication::restart() {
  if (mMainWindow)
    mMainWindow->close();
  else
    qApp->quit();
  QStringList nonProgramArgs = qApp->arguments();
  nonProgramArgs.removeFirst();
#ifdef __linux__
  QProcess::startDetached("./webots", nonProgramArgs);
#elif defined(_WIN32)
  exit(3030);  // this special code tells the launcher to restart Webots, see launcher.c
#else  // macOS
  QProcess::startDetached(qApp->arguments()[0], nonProgramArgs);
#endif
}

void WbGuiApplication::commandLineError(const QString &message, bool fatal) {
  cerr << "webots: " << message.toUtf8().constData() << endl;
  cerr << tr("Try 'webots --help' for more information.").toUtf8().constData() << endl;
  if (fatal)
    mTask = FAILURE;
}

void WbGuiApplication::parseArguments() {
  // faster when copied according to Qt's doc
  QStringList args = arguments();
  bool logPerformanceMode = false, batch = false;
  int port = 1234;  // default value
  mStream = '\0';

  const int size = args.size();
  for (int i = 1; i < size; ++i) {
    const QString &arg = args[i];
    if (arg == "--minimize")
      mShouldMinimize = true;
    else if (arg == "--fullscreen")
      mShouldStartFullscreen = true;
    else if (arg == "--mode=stop") {
      commandLineError(tr("'--mode=stop' is deprecated."), false);
      mStartupMode = WbSimulationState::PAUSE;
    } else if (arg == "--mode=pause")
      mStartupMode = WbSimulationState::PAUSE;
    else if (arg == "--mode=realtime")
      mStartupMode = WbSimulationState::REALTIME;
    else if (arg == "--mode=fast")
      mStartupMode = WbSimulationState::FAST;
    else if (arg == "--mode=run") {
      commandLineError(tr("`--mode=run` is deprecated, falling back to `fast` mode."), false);
      mStartupMode = WbSimulationState::FAST;
    } else if (arg == "--no-rendering")
      mShouldDoRendering = false;
    else if (arg == "convert") {
      mTask = CONVERT;
      mTaskArguments = args.mid(i);
      break;
    } else if (arg == "--help")
      mTask = HELP;
    else if (arg == "--sysinfo")
      mTask = SYSINFO;
    else if (arg == "--version")
      mTask = VERSION;
    else if (arg == "--batch") {
      batch = true;
      WbMessageBox::disable();
    } else if (arg.startsWith("--update-world"))
      mTask = UPDATE_WORLD;
    else if (arg.startsWith("--port")) {
      int index = arg.indexOf('=');
      if (index == -1)
        commandLineError(tr("webots: missing '=' sign right after --port option").arg(arg));
      else {
        port = arg.mid(index + 1).toInt();
        if (port < 1 || port > 65535) {
          commandLineError(tr("webots: port value %1 out of range [1;65535], reverting to 1234 default value").arg(port));
          port = 1234;
        }
      }
    } else if (arg == "--stream") {
      mStream = 'w';  // w3d is the default mode
    } else if (arg.startsWith("--stream=")) {
      const QString mode = arg.mid(arg.indexOf('=') + 1);
      if (mode != "w3d" && mode != "mjpeg")
        commandLineError(tr("invalid value \"%1\" to '--stream' option.").arg(mode));
      else
        mStream = mode[0].toLatin1();
    } else if (arg == "--extern-urls")
      WbWorld::setPrintExternUrls();
    else if (arg == "--heartbeat")
      mHeartbeat = startTimer(1000);
    else if (arg.startsWith("--heartbeat=")) {
      bool ok;
      const int value = arg.mid(arg.indexOf('=') + 1).toInt(&ok);
      if (ok)
        mHeartbeat = startTimer(value);
      else
        commandLineError(tr("invalid value \"%1\" to '--heartbeat' option.").arg(arg.mid(arg.indexOf('=') + 1)));
    } else if (arg == "--stdout")
      WbLog::enableStdOutRedirectToTerminal();
    else if (arg == "--stderr")
      WbLog::enableStdErrRedirectToTerminal();
    else if (arg == "--log-performance")
      commandLineError(tr("invalid '--log-performance' option: log file path is missing."), false);
    else if (arg.startsWith("--log-performance=")) {
      QString logArgument = arg.mid(arg.indexOf('=') + 1);
      // remove starting/trailing double quotes
      if (logArgument.startsWith('"'))
        logArgument = logArgument.right(logArgument.size() - 1);
      if (logArgument.endsWith('"'))
        logArgument = logArgument.left(logArgument.size() - 1);
      if (logArgument.contains(",")) {
        QStringList argumentsList = logArgument.split(",");
        WbPerformanceLog::createInstance(argumentsList[0], argumentsList[1].trimmed().toInt());
      } else
        WbPerformanceLog::createInstance(logArgument);
      logPerformanceMode = true;
    } else if (arg == "--clear-cache")
      WbNetwork::instance()->clearCache();
    else if (arg.startsWith("-")) {
      commandLineError(tr("invalid option: '%1'").arg(arg));
    } else {
      if (mStartWorldName.isEmpty())
        mStartWorldName = QDir::fromNativeSeparators(arg);
      else
        commandLineError(tr("too many arguments."));
    }
  }
  if (mStream == '\0')  // we need a simple streaming server for robot windows and remote controllers
    mTcpServer = new WbTcpServer(mStream);
  else {
    if (!batch)
      commandLineError(tr("you should also use --batch (in addition to --stream) for production."), false);
    if (mStream == 'm')
      mTcpServer = new WbMultimediaStreamingServer();
    else {  // w3d
      mTcpServer = new WbW3dStreamingServer();
      WbWorld::enableW3dStreaming();
    }
  }
  mTcpServer->start(port);
  if (mTcpServer->port() == -1)
    commandLineError(tr("failed to open TCP server in the port range [%1-%2]\n").arg(port).arg(port + 10));

  // create the Webots temporary path based on the TCP port early in the process
  // in order to be sure that the Qt internal files will be stored at the right place
  else if (!WbStandardPaths::webotsTmpPathCreate(mTcpServer->port()))
    commandLineError(tr("failed to create the Webots temporary path \"%1\".\n").arg(WbStandardPaths::webotsTmpPath()));

  if (logPerformanceMode) {
    WbPerformanceLog::enableSystemInfoLog(mTask == SYSINFO);
    mTask = NORMAL;
  }

  if (WbPreferences::booleanEnvironmentVariable("WEBOTS_SAFE_MODE")) {
    WbPreferences::instance()->setValue("OpenGL/disableShadows", true);
    WbPreferences::instance()->setValue("OpenGL/disableAntiAliasing", true);
    WbPreferences::instance()->setValue("OpenGL/GTAO", 0);
    WbPreferences::instance()->setValue("OpenGL/textureQuality", 0);
    WbPreferences::instance()->setValue("OpenGL/textureFiltering", 0);
    mStartupMode = WbSimulationState::PAUSE;
    mStartWorldName = WbStandardPaths::resourcesPath() + "projects/worlds/empty.wbt";
  }
}

int WbGuiApplication::exec() {
  if (mTask == NORMAL || mTask == UPDATE_WORLD) {
    if (mTask == UPDATE_WORLD)
      connect(mApplication, &WbApplication::worldLoadCompleted, this, &WbGuiApplication::taskExecutor);

    if (setup()) {
      QApplication::processEvents();
      loadInitialWorld();
    }
  }

  // with the addition of EXTERNPROTO, the load of a world is not linear anymore and takes two passes hence the task must be
  // invoked only when the loading effectively takes place
  const WbSingleTaskApplication *task = NULL;
  if (mTask != NORMAL && mTask != UPDATE_WORLD)
    task = taskExecutor();

  const int status = QApplication::exec();
  delete task;
  return status;
}

const WbSingleTaskApplication *WbGuiApplication::taskExecutor() {
  assert(mTask != NORMAL);

  if (mTask == UPDATE_WORLD)
    disconnect(mApplication, &WbApplication::worldLoadCompleted, this, &WbGuiApplication::taskExecutor);

  const WbSingleTaskApplication *task = new WbSingleTaskApplication(mTask, mTaskArguments, this, mApplication->startupPath());
  if (mMainWindow)
    connect(task, &WbSingleTaskApplication::finished, mMainWindow, &WbMainWindow::close);
  else
    connect(task, &WbSingleTaskApplication::finished, this, &QApplication::exit);
  // run the task from the application event loop
  QTimer::singleShot(0, task, SLOT(run()));

  return task;
}

bool WbGuiApplication::setup() {
  WbPreferences *const prefs = WbPreferences::instance();
  if (mStartupMode == WbSimulationState::NONE)
    mStartupMode = startupModeFromPreferences();
  if (mShouldDoRendering)
    mShouldDoRendering = renderingFromPreferences();

  WbSimulationState::instance()->setMode(mStartupMode);
  WbSimulationState::instance()->setRendering(mShouldDoRendering);

  // check specified world file if any
  if (!mStartWorldName.isEmpty()) {
    // if relative, make absolute
    if (QDir::isRelativePath(mStartWorldName))
      mStartWorldName = mApplication->startupPath() + '/' + mStartWorldName;

    QFileInfo info(mStartWorldName);
    if (!info.isReadable())
      commandLineError(tr("could not open file: '%1'.").arg(mStartWorldName));
  }

  if (WbMessageBox::enabled() &&
      (!WbPreferences::instance()->contains("General/theme") || !WbPreferences::instance()->contains("General/telemetry"))) {
    if (WbNewVersionDialog::run() != QDialog::Accepted) {
      mTask = QUIT;
      return false;
    } else if (WbPreferences::instance()->value("General/theme").toString() != mThemeLoaded)
      updateStyleSheet();
  }

  // Show guided tour if first ever launch and no command line world argument is given
  bool showGuidedTour =
    prefs->value("Internal/firstLaunch", true).toBool() && mStartWorldName.isEmpty() && WbMessageBox::enabled();

#ifndef _WIN32
  // create main window on Linux and macOS before the splash screen otherwise, the
  // image in the splash screen is empty...
  // Doing the same on Windows slows down the popup of the SplashScreen, therefore
  // the main window is created later on Windows.
  mMainWindow = new WbMainWindow(mShouldMinimize, mTcpServer);
#endif

  if (!mShouldMinimize) {
    // splash screen
    // Warning: using heap allocated splash screen and/or pixmap cause crash while
    // showing tooltips in the main window under Linux.
    const QDir screenshotLocation = QDir("images:splash_images/", "*.jpg");
    const QString webotsLogoName("webots.png");
    mSplash = new WbSplashScreen(screenshotLocation.entryList(), webotsLogoName);
    if (WbPreferences::instance()->value("MainWindow/maximized", false).toBool()) {
      // we need to center the splash screen on the same window as the mainWindow,
      // which is positioned wherever the mouse is on launch
      const QScreen *mainWindowScreen = QGuiApplication::screenAt(QCursor::pos());
      const QRect mainWindowScreenRect = mainWindowScreen->geometry();
      QPoint targetPosition = mainWindowScreenRect.center();
      targetPosition.setX(targetPosition.x() - mSplash->width() / 2);
      targetPosition.setY(targetPosition.y() - mSplash->height() / 2);
      mSplash->move(targetPosition);
    }

    // now we can safely show the splash screen, knowing it will be in the right place
    mSplash->show();
#ifdef __APPLE__
    // On macOS, when the WbSplashScreen is shown, Qt calls a resize event on the QMainWindow (not shown yet) with the size of
    // the splash screen. This overrides the WbMainWindow size preferences. This sounds like a Qt bug.
    mMainWindow->restorePreferredGeometry(mShouldMinimize);
#endif
    connect(WbLog::instance(), &WbLog::popupOpen, mSplash, &QSplashScreen::hide);
    connect(WbLog::instance(), &WbLog::popupClosed, mSplash, &QSplashScreen::show);
    processEvents();
    setSplashMessage(tr("Starting up..."));
  } else
    mSplash = NULL;
  // otherwise get it from the list of recent files
  if (mStartWorldName.isEmpty() || showGuidedTour)
    mStartWorldName = prefs->value("RecentFiles/file0", WbProject::newWorldPath()).toString();

  setSplashMessage(tr("Loading world..."));

#ifdef __APPLE__
  /**
   * Fixed the floating docks which are not rendered (gray panel)
   * This is a know issue between Qt 5.0.0 and 5.0.2
   * Hopefully this can be removed later.
   * cf: https://bugreports.qt-project.org/browse/QTBUG-30655
   **/
  setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
#endif

#ifdef _WIN32
  // create main window
  mMainWindow = new WbMainWindow(mShouldMinimize, mTcpServer);
#endif

  if (mShouldMinimize)
    mMainWindow->showMinimized();
  else {
    if (prefs->value("MainWindow/maximized", false).toBool())
      mMainWindow->showMaximized();
    else
      mMainWindow->showNormal();
  }

  connect(mMainWindow, &WbMainWindow::restartRequested, this, &WbGuiApplication::restart);
  connect(mMainWindow, &WbMainWindow::splashScreenCloseRequested, this, &WbGuiApplication::closeSplashScreenIfNeeded);
  mApplication->setup();

#ifdef __linux__
  // popup a warning message if the preferences file is not writable
  prefs->checkIsWritable();
  if (WbSysInfo::isRootUser() && WbSysInfo::environmentVariable("CI").isEmpty())
    WbLog::warning("It is not recommended to run Webots as root.");

  if (prefs->value("Internal/firstLaunch", true).toBool()) {
    // delete previous desktop application info file in order to update it to the current installation data
    const QString desktopFilePath = QDir::homePath() + "/.local/share/applications/webots.desktop";
    if (QFile::exists(desktopFilePath))
      QFile::remove(desktopFilePath);
  }
#endif

  WbWrenOpenGlContext::makeWrenCurrent();
  WbSysInfo::initializeOpenGlInfo();
  WbWrenOpenGlContext::doneWren();

  if (showGuidedTour)
    mMainWindow->showUpdatedDialog();  // the guided tour will be shown after the updated dialog

  return true;
}

WbSimulationState::Mode WbGuiApplication::startupModeFromPreferences() const {
  WbPreferences *const prefs = WbPreferences::instance();
  const QString startupMode(prefs->value("General/startupMode").toString());

  if (startupMode == "Real-time")
    return WbSimulationState::REALTIME;
  if (startupMode == "Fast")
    return WbSimulationState::FAST;
  return WbSimulationState::PAUSE;
}

bool WbGuiApplication::renderingFromPreferences() const {
  WbPreferences *const prefs = WbPreferences::instance();
  return prefs->value("General/rendering", true).toBool();
}

#ifdef __APPLE__
// Fixed the Webots opening by double-clicking on .wbt file in the Finder
bool WbGuiApplication::event(QEvent *event) {
  switch (event->type()) {
    case QEvent::FileOpen:
      mStartWorldName = static_cast<QFileOpenEvent *>(event)->file();
      return true;
    default:
      return QApplication::event(event);
  }
}
#endif

void WbGuiApplication::setSplashMessage(const QString &message) {
  if (!mSplash)
    return;

  QString copyright = tr("Copyright \u00A9 1998 - %1 Cyberbotics Ltd.").arg(WbApplicationInfo::version().majorNumber());
  mSplash->setLiveMessage(copyright + "\n" + message);
  mSplash->repaint();
}

void WbGuiApplication::closeSplashScreenIfNeeded() {
  if (mSplash) {
    mSplash->finish(mMainWindow);
    delete mSplash;
    mSplash = NULL;
  }
}

void WbGuiApplication::loadInitialWorld() {
  if (!mShouldMinimize && mShouldStartFullscreen)
    mMainWindow->setFullScreen(true, false, false, true);

  mMainWindow->loadWorld(mStartWorldName);
}

#ifdef _WIN32
static bool windowsDarkMode = false;

enum PreferredAppMode { Default, AllowDark, ForceDark, ForceLight, Max };

enum WINDOWCOMPOSITIONATTRIB {
  WCA_UNDEFINED = 0,
  WCA_NCRENDERING_ENABLED = 1,
  WCA_NCRENDERING_POLICY = 2,
  WCA_TRANSITIONS_FORCEDISABLED = 3,
  WCA_ALLOW_NCPAINT = 4,
  WCA_CAPTION_BUTTON_BOUNDS = 5,
  WCA_NONCLIENT_RTL_LAYOUT = 6,
  WCA_FORCE_ICONIC_REPRESENTATION = 7,
  WCA_EXTENDED_FRAME_BOUNDS = 8,
  WCA_HAS_ICONIC_BITMAP = 9,
  WCA_THEME_ATTRIBUTES = 10,
  WCA_NCRENDERING_EXILED = 11,
  WCA_NCADORNMENTINFO = 12,
  WCA_EXCLUDED_FROM_LIVEPREVIEW = 13,
  WCA_VIDEO_OVERLAY_ACTIVE = 14,
  WCA_FORCE_ACTIVEWINDOW_APPEARANCE = 15,
  WCA_DISALLOW_PEEK = 16,
  WCA_CLOAK = 17,
  WCA_CLOAKED = 18,
  WCA_ACCENT_POLICY = 19,
  WCA_FREEZE_REPRESENTATION = 20,
  WCA_EVER_UNCLOAKED = 21,
  WCA_VISUAL_OWNER = 22,
  WCA_HOLOGRAPHIC = 23,
  WCA_EXCLUDED_FROM_DDA = 24,
  WCA_PASSIVEUPDATEMODE = 25,
  WCA_USEDARKMODECOLORS = 26,
  WCA_LAST = 27
};

struct WINDOWCOMPOSITIONATTRIBDATA {
  // cppcheck-suppress unusedStructMember
  WINDOWCOMPOSITIONATTRIB Attrib;
  PVOID pvData;
  SIZE_T cbData;
};

using fnAllowDarkModeForWindow = BOOL(WINAPI *)(HWND hWnd, BOOL allow);
using fnSetPreferredAppMode = PreferredAppMode(WINAPI *)(PreferredAppMode appMode);
using fnSetWindowCompositionAttribute = BOOL(WINAPI *)(HWND hwnd, WINDOWCOMPOSITIONATTRIBDATA *);

static void setDarkTitlebar(HWND hwnd) {
  static fnAllowDarkModeForWindow AllowDarkModeForWindow = NULL;
  static fnSetWindowCompositionAttribute SetWindowCompositionAttribute = NULL;
  if (!AllowDarkModeForWindow) {  // first call
    HMODULE hUxtheme = LoadLibraryExW(L"uxtheme.dll", NULL, LOAD_LIBRARY_SEARCH_SYSTEM32);
    HMODULE hUser32 = GetModuleHandleW(L"user32.dll");
    AllowDarkModeForWindow = reinterpret_cast<fnAllowDarkModeForWindow>(GetProcAddress(hUxtheme, MAKEINTRESOURCEA(133)));
    SetWindowCompositionAttribute =
      reinterpret_cast<fnSetWindowCompositionAttribute>(GetProcAddress(hUser32, "SetWindowCompositionAttribute"));
    fnSetPreferredAppMode SetPreferredAppMode =
      reinterpret_cast<fnSetPreferredAppMode>(GetProcAddress(hUxtheme, MAKEINTRESOURCEA(135)));
    SetPreferredAppMode(AllowDark);
  }
  BOOL dark = TRUE;
  AllowDarkModeForWindow(hwnd, dark);
  WINDOWCOMPOSITIONATTRIBDATA data = {WCA_USEDARKMODECOLORS, &dark, sizeof(dark)};
  SetWindowCompositionAttribute(hwnd, &data);
}
#endif  // _WIN32

void WbGuiApplication::updateStyleSheet() {
  mThemeLoaded = WbPreferences::instance()->value("General/theme").toString();
  QFile qssFile(WbStandardPaths::resourcesPath() + mThemeLoaded);
  qssFile.open(QFile::ReadOnly);
  QString styleSheet = QString::fromUtf8(qssFile.readAll());

#ifdef __APPLE__
  QFile macOSQssFile(WbStandardPaths::resourcesPath() + "stylesheet.macos.qss");
  macOSQssFile.open(QFile::ReadOnly);
  styleSheet += QString::fromUtf8(macOSQssFile.readAll());

#elif defined(__linux__)
  QFile linuxQssFile(WbStandardPaths::resourcesPath() + "stylesheet.linux.qss");
  linuxQssFile.open(QFile::ReadOnly);
  styleSheet += QString::fromUtf8(linuxQssFile.readAll());

#elif _WIN32
  QFile windowsQssFile(WbStandardPaths::resourcesPath() + "stylesheet.windows.qss");
  windowsQssFile.open(QFile::ReadOnly);
  styleSheet += QString::fromUtf8(windowsQssFile.readAll());
#endif

  qApp->setStyleSheet(styleSheet);
#ifdef _WIN32
  if (mThemeLoaded != "webots_classic.qss")
    windowsDarkMode = true;
#endif
}

void WbGuiApplication::setWindowsDarkMode(QWidget *window) {
#ifdef _WIN32
  if (windowsDarkMode)
    setDarkTitlebar(reinterpret_cast<HWND>(window->winId()));
#endif
}

void WbGuiApplication::timerEvent(QTimerEvent *event) {
  if (event->timerId() == mHeartbeat)
    cout << "." << endl;
}
