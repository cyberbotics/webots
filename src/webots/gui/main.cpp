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

#include "WbApplication.hpp"
#include "WbGuiApplication.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLocale>
#include <QtCore/QRegularExpression>
#include <QtCore/QTextCodec>
#include <QtCore/QTextStream>
#include <QtCore/QVector>
#include <QtGui/QSurfaceFormat>
#include <QtWidgets/QApplication>

#include <csignal>

#ifdef _WIN32
#include <windows.h>
extern "C" {
// defaults to nVidia instead of Intel graphics on Optimus architectures (commonly found on laptops)
// unfortunately, the AMD equivalent doesn't seem to exist.
__declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
}
#ifdef NDEBUG
#include <stdio.h>
#include <wincon.h>
static void RedirectIOToConsole() {
  long int file_type = GetFileType(GetStdHandle(STD_OUTPUT_HANDLE));
  if (file_type != FILE_TYPE_PIPE) {
    if (!AttachConsole(ATTACH_PARENT_PROCESS))
      return;  // attempt to use the parent's console
    (void)freopen("CONOUT$", "w", stdout);
    (void)freopen("CONOUT$", "w", stderr);
    (void)freopen("CONIN$", "r", stdin);
  }
}
#endif
#else
#include <locale.h>
#endif

static QVector<QRegularExpression *> gQtMessageFilters;

// http://doc.qt.io/qt-5/qtglobal.html#qInstallMessageHandler
static void catchMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
  if (!gQtMessageFilters.isEmpty()) {
    QRegularExpressionMatch match;
    foreach (QRegularExpression *re, gQtMessageFilters) {
      match = re->match(msg);
      if (match.hasMatch())
        // filter out message
        return;
    }
  }

  QString message = msg;
  if (context.file != NULL)
    message += QString("(%s:%u, %s)").arg(context.file).arg(context.line).arg(context.function);
  switch (type) {
    case QtInfoMsg:
      fprintf(stderr, "Info: %s\n", message.toUtf8().constData());
      break;
    case QtDebugMsg:
      fprintf(stderr, "Debug: %s\n", message.toUtf8().constData());
      break;
    case QtWarningMsg:
      fprintf(stderr, "Warning: %s\n", message.toUtf8().constData());
      break;
    case QtCriticalMsg:
      fprintf(stderr, "Critical: %s\n", message.toUtf8().constData());
      break;
    case QtFatalMsg:
      fprintf(stderr, "Fatal: %s\n", message.toUtf8().constData());
      abort();
  }
}

static void quitApplication(int sig) {
  WbApplication::instance()->simulationQuit(EXIT_SUCCESS);
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
#ifdef NDEBUG
  const char *MSYSCON = getenv("MSYSCON");
  if (MSYSCON && strncmp("mintty.exe", MSYSCON, 10) == 0)
    // if webots was started from a MINGW mintty console
    // we need to unbuffer the stderr as _IOLBF is not working in the msys console
    setvbuf(stderr, NULL, _IONBF, 0);
  else                      // started from a DOS console or from Windows (double click on Webots icon)
    RedirectIOToConsole();  // the release version is built with the -mwindows flag
                            // which drops stdout/stderr, so we need to redirect
                            // them to the parent console in case Webots was started
                            // from a DOS
#else
  // we need to unbuffer the stderr as _IOLBF is not working in the msys console
  setvbuf(stderr, NULL, _IONBF, 0);
#endif
#endif
  QLocale::setDefault(QLocale::c());
  QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));  // so that all QTextStream use UTF-8 encoding by default

#ifdef __linux__
  // on Linux, the webots binary is located in $WEBOTS_HOME/bin/webots-bin
  const QString webotsDirPath = QDir(QFileInfo(argv[0]).absolutePath() + "/..").canonicalPath();
#elif defined(__APPLE__)
  // on macOS, the webots binary is located in $WEBOTS_HOME/Contents/MacOS/webots-bin
  const QString webotsDirPath = QDir(QFileInfo(argv[0]).absolutePath() + "/../..").canonicalPath();
#else
  // on Windows, the webots binary is located in $WEBOTS_HOME/msys64/mingw64/bin/webots
  // we need to use GetModuleFileName as argv[0] doesn't always provide an absolute path
  const int BUFFER_SIZE = 4096;
  char *modulePath = new char[BUFFER_SIZE];
  GetModuleFileName(NULL, modulePath, BUFFER_SIZE);
  const QString webotsDirPath = QDir(QFileInfo(modulePath).absolutePath() + "/../../..").canonicalPath();
  delete[] modulePath;
#endif

  const QString QT_QPA_PLATFORM_PLUGIN_PATH = qEnvironmentVariable("QT_QPA_PLATFORM_PLUGIN_PATH");
  if (QT_QPA_PLATFORM_PLUGIN_PATH.isEmpty()) {
    const QString platformPluginPath =
#ifdef _WIN32
      webotsDirPath + "/msys64/mingw64/share/qt5/plugins";
#else
      webotsDirPath + "/lib/webots/qt/plugins";
#endif
    qputenv("QT_QPA_PLATFORM_PLUGIN_PATH", platformPluginPath.toUtf8());
  }

  // load qt warning filters from file
  QString qtFiltersFilePath = QDir::fromNativeSeparators(webotsDirPath + "/resources/qt_warning_filters.conf");
  QFile qtFiltersFile(qtFiltersFilePath);
  if (qtFiltersFile.open(QIODevice::ReadOnly)) {
    QTextStream in(&qtFiltersFile);
    QString line;
    while (!in.atEnd()) {
      line = in.readLine();
      line = line.trimmed();
      if (line.startsWith("#") || line.isEmpty())
        continue;
      QRegularExpression *re = new QRegularExpression(
        line, QRegularExpression::ExtendedPatternSyntaxOption | QRegularExpression::UseUnicodePropertiesOption);
      if (re->isValid())
        gQtMessageFilters.append(re);
      else {
        QString message = QString("regular expression '%1' in file '%2' is invalid: %3")
                            .arg(line)
                            .arg(qtFiltersFilePath)
                            .arg(re->errorString());
        fprintf(stderr, "%s\n", message.toUtf8().constData());
      }
    }
  } else {
    QString message = QString("File not found: '%1'.").arg(qtFiltersFilePath);
    fprintf(stderr, "%s\n", message.toUtf8().constData());
  }

  // Putting break points in the catchMessageOutput and getting the stack allows to determine
  // efficiently what Webots statement is responsible to generate some Qt output
  qInstallMessageHandler(catchMessageOutput);

  QApplication::setAttribute(Qt::AA_Use96Dpi);

  WbGuiApplication app(argc, argv);

  // Quit the application correctly when receiving POSIX signals.
  signal(SIGINT, quitApplication);  // this signal is working on Windows when Ctrl+C from cmd.exe.
#ifndef _WIN32
  signal(SIGTERM, quitApplication);
  signal(SIGQUIT, quitApplication);
  signal(SIGHUP, quitApplication);
#endif

#ifndef _WIN32
  // Symptom: locale is wrong in dynamic libraries (i.e. physics plugin)
  // From http://qt-project.org/doc/qt-4.8/qcoreapplication.html :
  //   On Unix/Linux Qt is configured to use the system locale settings by default.
  //   This can cause a conflict when using POSIX functions, for instance, when
  //   converting between data types such as floats and strings, since the
  //   notation may differ between locales. To get around this problem, call
  //   the POSIX function setlocale(LC_NUMERIC,"C") right after initializing
  //   QApplication or QCoreApplication to reset the locale that is used for
  //   number formatting to "C"-locale.
  setlocale(LC_NUMERIC, "C");
#endif

#ifdef __APPLE__
  // 'LANG' can be set in the terminal.
  // - "fr_CH.UTF-8" may cause issues in procedural PROTO nodes.
  // - If "UTF-8" is not set, UTF-8 characters are not handled properly in some libraries, such as FreeType.
  setenv("LANG", "UTF-8", true);
#endif

  return app.exec();
}
