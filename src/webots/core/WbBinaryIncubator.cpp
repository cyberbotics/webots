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

#include "WbBinaryIncubator.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include <cassert>

WbBinaryIncubator::Error WbBinaryIncubator::copyBinaryAndDependencies(const QString &filename) {
  QFileInfo fi(filename);

  // copy the main binary
  Error error = copyBinaryPath(fi.absolutePath());

  // copy the project libraries sub folders
  QDir projectLibraryDir(WbProject::current()->path() + "libraries");
  projectLibraryDir.setFilter(QDir::Dirs);
  foreach (const QString &subDir, projectLibraryDir.entryList()) {
    if (!subDir.startsWith("."))
      copyBinaryPath(projectLibraryDir.path() + '/' + subDir);
  }

  return error;
}

WbBinaryIncubator::Error WbBinaryIncubator::copyBinaryPath(const QString &path) {
  QFileInfo fi(path);
  assert(fi.isDir());

  int executableLastIndex = path.lastIndexOf("controllers");
  int libraryLastIndex = qMax(path.lastIndexOf("libraries"), path.lastIndexOf("plugins"));

  if (executableLastIndex == -1 && libraryLastIndex == -1)
    return INVALID_BINARY_PATH;

  bool isExecutablePath = executableLastIndex > libraryLastIndex;
  bool isLibraryPath = executableLastIndex < libraryLastIndex;

  QString binaryFullName = fi.baseName();
  if (isExecutablePath)
    binaryFullName += WbStandardPaths::executableExtension();
  else if (isLibraryPath)
    binaryFullName = WbStandardPaths::dynamicLibraryPrefix() + binaryFullName + WbStandardPaths::dynamicLibraryExtension();
  else
    return INVALID_BINARY_TYPE;

  QString copySource = path + "/build/";
  if (!QFileInfo(copySource).isDir())
    return UNEXISTING_BUILD_FOLDER;

  QString copyTarget = path + '/' + binaryFullName;
  QDateTime newerDate = QFileInfo(copyTarget).lastModified();
  QString newerBinary = copyTarget;
  QDir dir(copySource);
  dir.setFilter(QDir::Dirs);
  bool found = false;
  foreach (const QString &subDir, dir.entryList()) {
    if (!subDir.startsWith(".")) {
      QString possibleSource = path + "/build/" + subDir + "/" + binaryFullName;
      if (QFile::exists(possibleSource)) {
        QDateTime possibleSourceDate = QFileInfo(possibleSource).lastModified();
        if (possibleSourceDate > newerDate) {
          newerDate = possibleSourceDate;
          newerBinary = possibleSource;
          found = true;
        }
      }
    }
  }
  if (!found)
    return NONE;  // no need to perform a copy as the current file is already the newest

  // Qt API to copy file only allow the copy if the target
  // file is not existing
  // Note: This seems to be really true only on Windows, but
  //       the doc says to do so
  if (QFile::exists(copyTarget))
    if (!QFile::remove(copyTarget))
      return FILE_REMOVE_ERROR;

  bool success = QFile::copy(newerBinary, copyTarget);
  if (success)
    return NONE;
  else
    return FILE_COPY_ERROR;
}
