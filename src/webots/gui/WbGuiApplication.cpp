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

#include "WbGuiApplication.hpp"

#include "WbApplication.hpp"
#include "WbApplicationInfo.hpp"
#include "WbConsole.hpp"
#include "WbMainWindow.hpp"
#include "WbMessageBox.hpp"
#include "WbMultimediaStreamingServer.hpp"
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
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbX3dStreamingServer.hpp"

#include <QtCore/QCommandLineParser>
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
  mTask(NORMAL),
  mStreamingServer(NULL) {
  setApplicationName("Webots");
  setApplicationVersion(WbApplicationInfo::version().toString(true, false, true));
  setOrganizationName("Cyberbotics");
  setOrganizationDomain("cyberbotics.com");
#ifdef _WIN32
  QProcess process;
  process.start("cygpath", QStringList{QString("-w"), QString("/")});
  process.waitForFinished(-1);
  QString MSYS2_HOME = process.readAllStandardOutput().trimmed();
  MSYS2_HOME.chop(1);                          // remove final backslash
  qputenv("MSYS2_HOME", MSYS2_HOME.toUtf8());  // useful to Python 3.8 controllers
  const QString webotsQtPlugins = MSYS2_HOME.replace('\\', '/') + "/mingw64/share/qt5/plugins";
  QCoreApplication::setLibraryPaths(QStringList(webotsQtPlugins));
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
  udpateStyleSheet();

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

void WbGuiApplication::parseStreamArguments(const QString &streamArguments) {
  bool monitorActivity = false;
  bool disableTextStreams = false;
  bool ssl = false;
  bool controllerEdit = false;
  int port = 1234;
  QString mode = "x3d";

  const QStringList &options = streamArguments.split(';', Qt::SkipEmptyParts);
  foreach (QString option, options) {
    option = option.trimmed();
    const QRegExp rx("(\\w+)\\s*=\\s*([A-Za-z0-9:/.\\-,]+)?");
    rx.indexIn(option);
    const QStringList &capture = rx.capturedTexts();
    // "key" without value case
    if (option == "monitorActivity")
      monitorActivity = true;
    else if (option == "disableTextStreams")
      disableTextStreams = true;
    else if (option == "ssl")
      ssl = true;
    else if (option == "controllerEdit")
      controllerEdit = true;
    else if (capture.size() == 3) {
      const QString &key = capture[1];
      const QString &value = capture[2];
      if (key == "port") {
        bool ok;
        const int tmpPort = value.toInt(&ok);
        if (ok)
          port = tmpPort;
        else {
          cerr << tr("webots: invalid 'port' option: '%1' in --stream").arg(value).toUtf8().constData() << endl;
          cerr << tr("webots: stream port has to be integer").toUtf8().constData() << endl;
          mTask = FAILURE;
        }
      } else if (key == "mode") {
        if (value != "x3d" && value != "mjpeg") {
          cerr << tr("webots: invalid 'mode' option: '%1' in --stream").arg(value).toUtf8().constData() << endl;
          cerr << tr("webots: stream mode can only be x3d or mjpeg").toUtf8().constData() << endl;
          mTask = FAILURE;
        } else if (value == "mjpeg")
          mode = "mjpeg";
      } else {
        cerr << tr("webots: unknown option: '%1' in --stream").arg(option).toUtf8().constData() << endl;
        mTask = FAILURE;
      }
    } else {
      cerr << tr("webots: unknown option: '%1' in --stream").arg(option).toUtf8().constData() << endl;
      mTask = FAILURE;
    }
  }

  if (mode == "mjpeg") {
    mStreamingServer = new WbMultimediaStreamingServer(monitorActivity, disableTextStreams, ssl, controllerEdit);
    mStreamingServer->start(port);
    return;
  }
  mStreamingServer = new WbX3dStreamingServer(monitorActivity, disableTextStreams, ssl, controllerEdit);
  mStreamingServer->start(port);
  WbWorld::enableX3DStreaming();
}

void WbGuiApplication::parseArguments() {
  // faster when copied according to Qt's doc
  bool logPerformanceMode = false;
  bool batch = false, stream = false;
  QCommandLineParser parser;
  parser.addPositionalArgument("worldfile", tr("Start Webots in this world."));
  parser.setOptionsAfterPositionalArgumentsMode(QCommandLineParser::ParseAsOptions);
  QCommandLineOption helpOption("help", tr("Display this help message and exit."));
  parser.addOption(helpOption);
  QCommandLineOption versionOption("version", tr("Display version information and exit."));
  parser.addOption(versionOption);
  QCommandLineOption sysinfoOption("sysinfo", tr("Display information about the system and exit."));
  parser.addOption(sysinfoOption);
  QCommandLineOption modeOption("mode",
                                tr("Choose the startup mode, overriding application preferences. The <mode> argument must be "
                                   "either pause, realtime or fast."),
                                tr("mode"));
  parser.addOption(modeOption);
  QCommandLineOption noRenderingOption("no-rendering", tr("Disable rendering in the main 3D view."));
  parser.addOption(noRenderingOption);
  QCommandLineOption fullscreenOption("fullscreen", tr("Start Webots in fullscreen."));
  parser.addOption(fullscreenOption);
  QCommandLineOption minimizeOption("minimize", tr("Minimize the Webots window on startup."));
  parser.addOption(minimizeOption);
  QCommandLineOption batchOption("batch", tr("Prevent Webots from creating blocking pop-up windows."));
  parser.addOption(batchOption);
  QCommandLineOption stdoutOption("stdout", tr("Redirect the stdout of the controllers to the terminal."));
  parser.addOption(stdoutOption);
  QCommandLineOption stderrOption("stderr", tr("Redirect the stderr of the controllers to the terminal."));
  parser.addOption(stderrOption);
  QCommandLineOption streamOption("stream",
                                  tr("Start the Webots streaming server. Parameters may be given as an option:\n"
                                     "port=1234          - Start the streaming server on port 1234.\n"
                                     "mode=<x3d|mjpeg>   - Specify the streaming mode: x3d (default) or mjpeg.\n"
                                     "monitorActivity    - Print a dot '.' on stdout every 5 seconds.\n"
                                     "disableTextStreams - Disable the streaming of stdout and stderr."),
                                  tr("key[=value];..."));
  parser.addOption(streamOption);
  QCommandLineOption updateProtoCacheOption("update-proto-cache", tr("Update the PROTO cache"), tr("task"));
  parser.addOption(updateProtoCacheOption);
  QCommandLineOption logPerfomanceOption(
    "log-performance",
    tr("Measure the performance of Webots and log it in the file specified in the <file> argument. The optional <steps> "
       "argument is an integer value that specifies how many steps are logged. If the --sysinfo option is used, the system "
       "information is prepended into the log file"),
    tr("file"));
  QCommandLineOption convertOption("convert", tr("Convert a PROTO file to a URDF, WBO, or WRL file."), tr("file"));
  parser.addOption(convertOption);
  QCommandLineOption updateWorldOption("update-world", tr("Update world"));
  parser.addOption(updateWorldOption);
  QCommandLineOption x3DMetaFileExportOption("enable-x3d-meta-file-export", tr("Enable x3d meta file export"));
  parser.addOption(x3DMetaFileExportOption);
  parser.addOption(logPerfomanceOption);
#ifndef _WIN32

  parser.addOptions({
    {"disable-logging", tr("Disable logging")},
    {"enable-logging", tr("Enable logging")},
    {"log-level", tr("Log level")},
    {"no-sandbox", tr("No sandbox")},
    {"single-process", tr("Single Process")},
    {"remote-debugging-port", tr("Remote Debugging Port")},
  });
#endif

#ifdef _WIN32
  if (qgetenv("WEBOTS_TERMINAL") == "1" && GetConsoleWindow() == 0)
    AttachConsole(ATTACH_PARENT_PROCESS);
#endif

  parser.parse(QCoreApplication::arguments());
  const QStringList unknownOptions = parser.unknownOptionNames();
  QStringList args = parser.optionNames();
  const QStringList path = parser.positionalArguments();
  if (!unknownOptions.isEmpty()) {
    cout << tr("webots: invalid option: '--%1'").arg(unknownOptions[0]).toUtf8().constData() << endl;
    mTask = FAILURE;
  }
  while (!args.isEmpty()) {
    const QString subCommand = args.first();
    if (subCommand == "help" && parser.isSet(helpOption)) {
      mTask = HELP;
    } else if (subCommand == "version" && parser.isSet(versionOption)) {
      mTask = VERSION;
    } else if (subCommand == "minimize" && parser.isSet(minimizeOption)) {
      mShouldMinimize = true;
    } else if (subCommand == "fullscreen" && parser.isSet(fullscreenOption)) {
      mShouldStartFullscreen = true;
    } else if (subCommand == "mode" && parser.isSet("mode")) {
      const QStringList mode = parser.values(modeOption);
      if (mode.size() != 1 || mode[0] == "=") {
        cerr << tr("Error: Must specify one mode argument.").toUtf8().constData() << endl;
        mTask = FAILURE;
      }
      if (mode[0] == "stop") {
        cerr << tr("The '--mode=stop' option is deprecated. Please use '--mode=pause' instead.").toUtf8().constData() << endl;
        cout << tr("Try 'webots --help' for more information.").toUtf8().constData() << endl;
        mStartupMode = WbSimulationState::PAUSE;
      } else if (mode[0] == "pause")
        mStartupMode = WbSimulationState::PAUSE;
      else if (mode[0] == "realtime")
        mStartupMode = WbSimulationState::REALTIME;
      else if (mode[0] == "fast")
        mStartupMode = WbSimulationState::FAST;
      else if (mode[0] == "run") {
        cerr << tr("Warning: `run` mode is deprecated, falling back to `fast` mode").toUtf8().constData() << endl;
        cout << tr("Try 'webots --help' for more information.").toUtf8().constData() << endl;
        mStartupMode = WbSimulationState::FAST;
      } else {
        cout << tr("webots: invalid option: '%1' in --mode").arg(mode[0]).toUtf8().constData() << endl;
        mTask = FAILURE;
      }
    } else if (subCommand == "no-rendering" && parser.isSet(noRenderingOption)) {
      mShouldDoRendering = false;
    } else if (subCommand == "convert" && parser.isSet("convert")) {
      const QStringList fileToConvert = parser.values(convertOption);
      mTaskArguments = fileToConvert;
      mTask = CONVERT;
    } else if (subCommand == "sysinfo" && parser.isSet(sysinfoOption)) {
      mTask = SYSINFO;
    } else if (subCommand == "batch" && parser.isSet(batchOption)) {
      batch = true;
      WbMessageBox::disable();
    } else if (subCommand == "update-proto-cache" && parser.isSet("update-proto-cache")) {
      const QStringList protoCacheUpdate = parser.values(updateProtoCacheOption);
      if (protoCacheUpdate.size() == 1)
        mTaskArguments.append(protoCacheUpdate[0]);
      else
        mTaskArguments.clear();
      mTask = UPDATE_PROTO_CACHE;
    } else if (subCommand == "update-world" && parser.isSet(updateWorldOption)) {
      mTask = UPDATE_WORLD;
    } else if (subCommand == "enable-x3d-meta-file-export" && parser.isSet(x3DMetaFileExportOption)) {
      WbWorld::enableX3DMetaFileExport();
    } else if (subCommand == "stream" && parser.isSet("stream")) {
      const QStringList streamArgument = parser.values(streamOption);
      stream = true;
      parseStreamArguments(streamArgument[0]);
    } else if (subCommand == "stdout" && parser.isSet(stdoutOption)) {
      WbLog::enableStdOutRedirectToTerminal();
    } else if (subCommand == "stderr" && parser.isSet(stderrOption)) {
      WbLog::enableStdErrRedirectToTerminal();
    } else if (subCommand == "log-performance" && parser.isSet("log-performance")) {
      const QStringList logArgument = parser.values(logPerfomanceOption);
      if (logArgument.isEmpty()) {
        cerr << tr("webots: log file path is missing in '--log-performance'.\n").toUtf8().constData();
        mTask = FAILURE;
      } else if (logArgument[0].contains(",")) {
        const QStringList argumentsList = logArgument[0].split(",");
        WbPerformanceLog::createInstance(argumentsList[0], argumentsList[1].trimmed().toInt());
      } else {
        WbPerformanceLog::createInstance(logArgument[0]);
        logPerformanceMode = true;
      }
    }
#ifndef _WIN32
    else if (subCommand == "disable-gpu" || subCommand == "disable-logging" || subCommand == "enable-logging" ||
             subCommand == "log-level" || subCommand == "no-sandbox" || subCommand == "single-process" ||
             subCommand == "remote-debugging-port") {
      // Silently ignore the awesome QWebEngine debugging tools:
      // cf. https://doc.qt.io/qt-5/qtwebengine-debugging.html
    }
#endif
    else {
      cout << tr("webots: invalid option: '--%1'").arg(subCommand).toUtf8().constData() << endl;
      mTask = FAILURE;
    }
    args.pop_front();
  }
  if (!path.isEmpty()) {
    if (path.size() > 1) {
      cerr << tr("webots: too many arguments.").toUtf8().constData() << endl;
      parser.clearPositionalArguments();
      mTask = FAILURE;

    } else if ((mStartWorldName).isEmpty() && (path.at(0)).isEmpty()) {
      cerr << tr("Argument 'name' missing.").toUtf8().constData() << endl;
      parser.clearPositionalArguments();
      mTask = FAILURE;
    }
    if (!(path.at(0)).isEmpty()) {
      mStartWorldName = path.at(0);
      parser.clearPositionalArguments();
    }
  }
  if (stream && !batch) {
    cerr << tr("Warning: you should also use --batch (in addition to --stream) for production.").toUtf8().constData() << endl;
    cout << tr("Try 'webots --help' for more information.").toUtf8().constData() << endl;
  }

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
  if (mTask == FAILURE) {
    cerr << tr("Try 'webots --help' for more information.").toUtf8().constData() << endl;
    return;
  }
}

int WbGuiApplication::exec() {
  if (mTask == NORMAL || mTask == UPDATE_WORLD) {
    if (setup()) {
      QApplication::processEvents();
      loadInitialWorld();
    }
  }

  WbSingleTaskApplication *task = NULL;
  if (mTask != NORMAL) {
    task = new WbSingleTaskApplication(mTask, mTaskArguments, this, mApplication->startupPath());
    if (mMainWindow)
      connect(task, &WbSingleTaskApplication::finished, mMainWindow, &WbMainWindow::close);
    else
      connect(task, &WbSingleTaskApplication::finished, this, &QApplication::exit);
    // run the task from the application event loop
    QTimer::singleShot(0, task, SLOT(run()));
  }

  int ret = QApplication::exec();
  delete task;
  return ret;
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
    if (!info.isReadable()) {
      cerr << tr("Could not open file: '%1'.").arg(mStartWorldName).toUtf8().constData() << endl;
      cout << tr("Try 'webots --help' for more information.").toUtf8().constData() << endl;
      mTask = FAILURE;
    }
  }

  if (WbMessageBox::enabled() &&
      (!WbPreferences::instance()->contains("General/theme") || !WbPreferences::instance()->contains("General/telemetry"))) {
    if (WbNewVersionDialog::run() != QDialog::Accepted) {
      mTask = QUIT;
      return false;
    } else if (WbPreferences::instance()->value("General/theme").toString() != mThemeLoaded)
      udpateStyleSheet();
  }

  // Show guided tour if first ever launch and no command line world argument is given
  bool showGuidedTour =
    prefs->value("Internal/firstLaunch", true).toBool() && mStartWorldName.isEmpty() && WbMessageBox::enabled();

#ifndef _WIN32
  // create main window on Linux and macOS before the splash screen otherwise, the
  // image in the splash screen is empty...
  // Doing the same on Windows slows down the popup of the SplashScreen, therefore
  // the main window is created later on Windows.
  mMainWindow = new WbMainWindow(mShouldMinimize, mStreamingServer);
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
  if (mStartWorldName.isEmpty() || showGuidedTour) {
    const QString defaultFileName = WbStandardPaths::emptyProjectPath() + "worlds/" + WbProject::newWorldFileName();
    mStartWorldName = prefs->value("RecentFiles/file0", defaultFileName).toString();
  }
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

  /**
   * Hopefully improved the icon resolution
   * http://blog.qt.digia.com/blog/2013/04/25/retina-display-support-for-mac-os-ios-and-x11/
   **/
  setAttribute(Qt::AA_UseHighDpiPixmaps);

#ifdef _WIN32
  // create main window
  mMainWindow = new WbMainWindow(mShouldMinimize, mStreamingServer);
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
  if (WbSysInfo::isRootUser())
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
  if (!mMainWindow->loadWorld(mStartWorldName))
    // this file should always exists
    mMainWindow->loadWorld(WbStandardPaths::emptyProjectPath() + "worlds/" + WbProject::newWorldFileName());

  if (!mShouldMinimize && mShouldStartFullscreen)
    mMainWindow->setFullScreen(true, false, false, true);
}

#ifdef _WIN32
#include <Windows.h>
#include <dwmapi.h>
#include <QtGui/QWindow>

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

void WbGuiApplication::udpateStyleSheet() {
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
