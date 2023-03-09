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

#ifndef WB_PLUGIN_HPP
#define WB_PLUGIN_HPP

#ifdef _WIN32
#include <windows.h>  // for HMODULE
#endif

#include <QtCore/QObject>

// abstract base class for dynamically loaded Libraries (plugins)

class WbPlugin : public QObject {
  Q_OBJECT

public:
  // plugin name, e.g. "salamander_physics"
  WbPlugin(const QString &name);

  // unloads the shared library and frees the memory associate to this plugin
  virtual ~WbPlugin();

  // returns the name passed as argument to the constructor, e.g. "blimp_physics"
  const QString &name() const { return mName; }

  // loads/unloads the shared library
  // returns true on success
  virtual bool load();
  bool unload();

  // returns true if the shared library is currentl loaded
  bool isLoaded() const { return mFunctions != NULL; }

  // returns the directory where the shared library file is located, including a trailing "/"
  // e.g. "/usr/local/webots/projects/samples/demos/plugins/physics/xyz_physics/"
  const QString &dirPath() const { return mDirPath; }

  // return the absolute path of the shared library file
  // e.g. "/usr/local/webots/projects/samples/demos/plugins/physics/xyz_physics/xyz_physics.dll"
  const QString &filePath() const { return mFilePath; }

protected:
  // must return a type string for example: "radio" or "physics"
  // this type is used to sort the plugins into subdirectories
  virtual const QString &type() const = 0;

  // must return the number of functions to be resolved in the plugin
  virtual int functionCount() const = 0;

  // The convention is that a function whose name starts with a '!' MUST be present
  // in the plugin. If any of the '!' mFunctions is missing, load will return: false
  virtual const char *functionName(int index) const = 0;

  // used on windows only: should copy dll to a new name before loading
  virtual bool shouldCopyBeforeLoad() const { return false; }

  void **mFunctions;  // array of plugin functions

private:
  QString mName;
  QString mDirPath;
  QString mFilePath;

#ifdef _WIN32
  HMODULE mLib;
  HMODULE copyAndOpenDll(const QString &name);
#else
  void *mLib;
#endif

  QString openLibrary(const QString &fullName);
};

#endif
