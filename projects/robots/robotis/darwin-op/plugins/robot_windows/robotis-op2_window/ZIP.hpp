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

#ifndef ZIP_HPP
#define ZIP_HPP

class QString;
struct zip;

class ZIP {
public:
  ZIP();
  virtual ~ZIP();

  // Compress a directory and all the sub-directory
  // archiveName : Name of the desired archive
  // folder      : folder to compress
  // firstFolder : All the file and directory are located in a directory in the archive
  // recursive   : Include sub-directory
  static bool CompressFolder(const QString &archiveName, const QString &folder, bool recursive = true,
                             const char *firstFolder = "");

  // Add a directory and all the sub-directory to an archive
  // archiveName : Name of the desired archive
  // folder      : folder to compress
  // firstFolder : All the file and directory are located in a directory in the archive
  // recursive   : Include sub-directory
  static bool AddFolderToArchive(const QString &archiveName, const QString &folder, bool recursive = true,
                                 const char *firstFolder = "");

  static bool AddFileToArchive(const QString &archiveName, const QString &file, const char *nameInArchive = "");

private:
  static bool AddFolder(struct zip *archive, const QString &folder, const QString &name, bool recursive = true);
  static bool AddFile(struct zip *archive, const QString &file, const QString &name);
};

#endif
