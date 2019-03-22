// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "OSMImportWindow.hpp"

#include <QtCore/QDir>
#include <QtWidgets/QApplication>
#include <string.h>

int main(int argc, char *argv[]) {
  QDir libDir(qgetenv("WEBOTS_HOME") + "/lib/qt/plugins");
  QApplication::addLibraryPath(libDir.absolutePath()); // required for for loading libqxcb
  QApplication app(argc, argv);
  char scriptPath[512] = "";
  if (argc > 1)
    strcpy(scriptPath, argv[1]);

  OSMImportWindow window(scriptPath);
  window.show();
  return app.exec();
}
