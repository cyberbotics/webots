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

#include "WbPlugin.hpp"

#include "WbBinaryIncubator.hpp"
#include "WbFileUtil.hpp"
#include "WbLog.hpp"
#include "WbProject.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"
#include "WbWorld.hpp"

#include <QtCore/QFile>
#include <QtCore/QFileInfo>

#ifdef _WIN32
static HMODULE loadLibrary(const QString &library) {
  int l = library.length();
  wchar_t *wlibrary = new wchar_t[l + 1];
  library.toWCharArray(wlibrary);
  wlibrary[l] = 0;
  HMODULE h = LoadLibraryW(wlibrary);
  delete[] wlibrary;
  return h;
}
#define DLOPEN(a, b) loadLibrary(a)
#define DLSYM(lib, name) static_cast<FARPROC>(GetProcAddress(lib, name))
#else
#include <dlfcn.h>

#ifdef __linux__
#define DLOPEN(a, b) dlopen(a.toUtf8(), b)
#define DLSYM(lib, name) dlsym(lib, name)

#elif defined(__APPLE__)
#define DLOPEN(a, b) dlopen(a.toUtf8(), b)
#define DLSYM(lib, name) dlsym(lib, name)
#endif

#endif

WbPlugin::WbPlugin(const QString &name) : mFunctions(NULL), mName(name), mLib(NULL) {
}

QString WbPlugin::openLibrary(const QString &fullName) {
  // check if plugins folder exists
  const QFileInfo pluginLib(fullName);
  if (!pluginLib.absoluteDir().exists())
    return QString();

  WbBinaryIncubator::copyBinaryAndDependencies(fullName);

  if (!QFile::exists(fullName))
    return tr("'%1': file does not exist.").arg(fullName);

  mLib = DLOPEN(fullName, RTLD_NOW);
  if (mLib == NULL)
#ifdef _WIN32
    return tr("'%1': cannot open DLL.").arg(fullName);
#else
    return QString(dlerror());
#endif

  return "";
}

bool WbPlugin::load() {
  if (mName == "<none>")
    return false;

  const QString pluginName("plugins/" + type() + "/" + mName + "/");
  const QString fileName(WbStandardPaths::dynamicLibraryPrefix() + mName + WbStandardPaths::dynamicLibraryExtension());

  QStringList possibleDirPaths;
  possibleDirPaths << WbProject::current()->path() + pluginName;
  foreach (WbProtoModel *model, WbProtoManager::instance()->models())
    possibleDirPaths << QDir(model->path() + "../" + pluginName).absolutePath() + "/";
  possibleDirPaths << WbStandardPaths::projectsPath() + "default/" + pluginName;
  possibleDirPaths << WbStandardPaths::resourcesProjectsPath() + pluginName;
  possibleDirPaths.removeDuplicates();  // case: several PROTOs in the same directory
  // qDebug() << possibleDirPaths;

  bool found = false;

  foreach (const QString &possibleDirPath, possibleDirPaths) {
    QString possiblePath = possibleDirPath + fileName;
    // try to open the library as soon the directory is found
    // case: the plugin is not built
    if (QFile::exists(possibleDirPath)) {
      const QString error(openLibrary(possiblePath));

      if (mLib == NULL) {
        WbLog::warning(tr("Error while loading plugin: '%1':").arg(possiblePath));
        WbLog::warning(error);
        return false;
      }

      mDirPath = possibleDirPath;
      mFilePath = possiblePath;

      found = true;
      break;
    }
  }

  if (!found) {
    WbLog::warning(tr("Plugin '%1' not found. Search in:").arg(mName));
    foreach (const QString &possibleDirPath, possibleDirPaths)
      WbLog::warning(QString("'%1'").arg(possibleDirPath));
    return false;
  }

  const int size = functionCount();
  mFunctions = new void *[size];
  for (int i = 0; i < size; ++i) {
    const char *const fname = functionName(i);
    mFunctions[i] = reinterpret_cast<void *>(DLSYM(mLib, fname + 1));

    if (!mFunctions[i] && fname[0] == '!') {
      WbLog::warning(tr("Function %1() missing in '%2'.").arg(QString(fname + 1), mFilePath));
      WbLog::warning(tr("You need to implement this function to activate the '%1' plugin.").arg(mName));
      unload();
      return false;
    }
  }

  return true;
}

bool WbPlugin::unload() {
  if (mLib) {
#ifdef _WIN32
    if (FreeLibrary(mLib) == 0)
#else
    if (dlclose(mLib))
#endif
      WbLog::warning(tr("Could not close dynamic library: '%1'").arg(mFilePath));

    mLib = NULL;
    delete[] mFunctions;
    mFunctions = NULL;
    return true;
  }

  return false;
}

WbPlugin::~WbPlugin() {
  unload();
}
