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

#include "ZIP.hpp"

#include <zip.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

#include <QtCore/QDir>
#include <QtCore/QString>
#include <QtCore/QStringList>

ZIP::ZIP() {
}

ZIP::~ZIP() {
}

bool ZIP::AddFile(struct zip *archive, const QString &file, const QString &name) {
  struct zip_source *s;
  if ((s = zip_source_file(archive, (const char *)file.toStdString().c_str(), 0, 0)) == NULL ||
      zip_add(archive, (const char *)name.toStdString().c_str(), s) < 0) {
    zip_source_free(s);
    return false;
  }
  return true;
}

bool ZIP::AddFolder(struct zip *archive, const QString &folder, const QString &name, bool recursive) {
  QStringList fileList;
  QStringList directoryList;

  // Add all files in this directory
  fileList = QDir(folder).entryList(QDir::Files);
  for (int i = 0; i < fileList.size(); i++) {
    if (!(AddFile(archive, (folder + "/" + fileList.at(i)), (name + "/" + fileList.at(i))))) {
      cerr << "Error while adding file " << fileList.at(i).toStdString() << " to archive" << endl;
      return false;
    }
  }

  // Add all sub-directories
  if (recursive) {
    directoryList = QDir(folder).entryList(QDir::AllDirs);
    for (int i = 2; i < directoryList.size(); i++) {
      if ((directoryList.at(i) != ".") && (directoryList.at(i) != "..")) {
        if (!(AddFolder(archive, (folder + "/" + directoryList.at(i)), (name + "/" + directoryList.at(i))))) {
          cerr << "Error while directory " << directoryList.at(i).toStdString() << " to archive" << endl;
          return false;
        }
      }
    }
  }

  return true;
}

bool ZIP::CompressFolder(const QString &archiveName, const QString &folder, bool recursive, const char *firstFolder) {
  int archiveError = 0;
  struct zip *archive = zip_open((const char *)archiveName.toStdString().c_str(), ZIP_CREATE, &archiveError);

  if (archiveError != 0) {
    cerr << "Error while creating archive" << endl;
    return false;
  } else {
    if (!(AddFolder(archive, folder, firstFolder, recursive)))
      return false;
    if (zip_close(archive) == -1) {
      cerr << "CompressFolder: Error while closing " << archiveName.toStdString() << " archive: " << zip_strerror(archive)
           << endl;
      return false;
    }
    return true;
  }
}

bool ZIP::AddFolderToArchive(const QString &archiveName, const QString &folder, bool recursive, const char *firstFolder) {
  int archiveError = 0;
  struct zip *archive = zip_open((const char *)archiveName.toStdString().c_str(), ZIP_CREATE, &archiveError);

  if (archiveError != 0) {
    cerr << "Error while opening archive" << endl;
    return false;
  } else {
    if (!(AddFolder(archive, folder, firstFolder, recursive)))
      return false;
    if (zip_close(archive) == -1) {
      cerr << "Error while closing archive" << endl;
      return false;
    }
    return true;
  }
}

bool ZIP::AddFileToArchive(const QString &archiveName, const QString &file, const char *nameInArchive) {
  int archiveError = 0;
  struct zip *archive = zip_open((const char *)archiveName.toStdString().c_str(), ZIP_CREATE, &archiveError);

  if (archiveError != 0) {
    cerr << "Error while openning archive" << endl;
    return false;
  } else {
    if (!(AddFile(archive, file, QString(nameInArchive))))
      return false;
    if (zip_close(archive) == -1) {
      cerr << "Error while closing archive" << endl;
      return false;
    }
    return true;
  }
}
