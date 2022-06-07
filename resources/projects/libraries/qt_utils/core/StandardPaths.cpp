#include "StandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QProcessEnvironment>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <webots/robot.h>

#include <QtCore/QDir>

#include <cassert>

using namespace webotsQtUtils;

#ifdef _WIN32
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#else
// dummy function used to know the full path
// of the current library
extern "C" {
static void foo() {
}
}
#endif

const QString &StandardPaths::getWebotsHomePath() {
  static QString path(QProcessEnvironment::systemEnvironment().value("WEBOTS_HOME") + "/");
  return path;
};

const QString &StandardPaths::getCurrentLibraryPath() {
  static bool defined = false;
  static QString path;

  if (!defined) {
#ifdef _WIN32
    WCHAR buffer[MAX_PATH] = {0};
    GetModuleFileNameW((HINSTANCE)&__ImageBase, buffer, sizeof(buffer));
    path = QString::fromWCharArray(buffer);
    path = path.replace('\\', '/');
#else
    Dl_info dl_info;
    dladdr(reinterpret_cast<void *>(foo), &dl_info);
    path = dl_info.dli_fname;
#endif
    path = path.mid(0, path.lastIndexOf('/') + 1);
    assert(!path.isEmpty());
    defined = true;
  }

  return path;
}

const QString &StandardPaths::getControllerPath() {
  static bool defined = false;
  static QString path;

  if (!defined) {
    path = QCoreApplication::applicationFilePath();
    path = path.mid(0, path.lastIndexOf('/') + 1);
    defined = true;
  }

  return path;
}

const QString &StandardPaths::getProjectPath() {
  static bool defined = false;
  static QString path;

  if (!defined) {
    path = QDir(wb_robot_get_project_path()).path();
    if (!path.endsWith('/'))
      path.append('/');
    defined = true;
  }

  return path;
}
