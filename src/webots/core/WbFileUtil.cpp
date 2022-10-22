// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbFileUtil.hpp"

#include "WbDesktopServices.hpp"
#include "WbLog.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QCryptographicHash>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QProcess>
#include <QtCore/QTextStream>
#include <QtCore/QUrl>

#include <cassert>

bool WbFileUtil::copyAndReplaceString(const QString &sourcePath, const QString &destinationPath, const QString &before,
                                      const QString &after) {
  QList<QPair<QString, QString>> values;
  values << QPair<QString, QString>(before, after);

  return copyAndReplaceString(sourcePath, destinationPath, values);
}

bool WbFileUtil::copyAndReplaceString(const QString &sourcePath, const QString &destinationPath,
                                      QList<QPair<QString, QString>> values) {
  QFile sourceFile(sourcePath);
  if (!sourceFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }

  QFile destinationFile(destinationPath);
  if (!destinationFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  // read source file in memory
  QTextStream in(&sourceFile);
  QString content = in.readAll();

  // replace strings
  for (int i = 0; i < values.size(); i++)
    content.replace(values[i].first, values[i].second);

  // write to destination file
  QTextStream out(&destinationFile);
  out << content;

  return true;
}

bool WbFileUtil::forceCopy(const QString &source, const QString &target) {
  QFile targetFile(target);
  if (targetFile.exists()) {
    bool success = targetFile.remove();
    if (!success)
      return false;
  }
  return QFile::copy(source, target);
}

int WbFileUtil::copyDir(const QString &sourcePath, const QString &destPath, bool recurse, bool merge, bool setWritable) {
  // check if source exists
  QDir sourceDir(sourcePath);
  if (!sourceDir.exists())
    return 0;

  // create destination directory path if necessary
  QDir destDir(destPath);
  if (!destDir.exists())
    if (!destDir.mkpath(destPath))
      return 0;

  // copy files
  int count = 0;
  QStringList files = sourceDir.entryList(QDir::Files | QDir::Hidden);
  foreach (QString file, files) {
    QString srcName = sourcePath + "/" + file;
    QString destName = destPath + "/" + file;
    if (QFile::exists(destName)) {
      WbLog::warning(
        QString("The file '%1' was not copied because a file with the same name already exists in the destination directory.")
          .arg(file));
      continue;
    } else if (!QFile::copy(srcName, destName))
      continue;
    if (setWritable) {
      QFile::Permissions permissions = QFile::permissions(destName);
      QFile::setPermissions(destName, permissions | QFile::WriteOwner | QFile::WriteUser);
    }
    count++;
  }

  // recurse in subdirectories
  if (recurse) {
    QStringList dirs = sourceDir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
    foreach (QString dir, dirs) {
      QString srcName = sourcePath + "/" + dir;
      QString destName = destPath + "/" + dir;
      if (!QDir(destName).exists() || merge)
        count += copyDir(srcName, destName, true, merge, setWritable);
    }
  }

  return count;
}

bool WbFileUtil::areIdenticalFiles(const QString &fileAPath, const QString &fileBPath) {
  // Test: check that the sha1 signatures of the files are matching or not
  // cf: http://stackoverflow.com/questions/8517024/check-if-a-file-equals-to-other

  if (!QFileInfo(fileAPath).exists() || !QFileInfo(fileBPath).exists())
    return false;

  QString files[2] = {fileAPath, fileBPath};
  QByteArray signatures[2];

  for (int i = 0; i < 2; ++i) {
    QCryptographicHash hash(QCryptographicHash::Sha1);
    QFile file(files[i]);

    bool success = file.open(QIODevice::ReadOnly);
    if (!success)
      return false;

    while (!file.atEnd())
      hash.addData(file.read(8192));

    signatures[i] = hash.result();
  }

  return signatures[0] == signatures[1];
}

bool WbFileUtil::isLocatedInDirectory(const QString &file, const QString &directory) {
#ifdef _WIN32
  return file.startsWith(directory, Qt::CaseInsensitive);
#else
  return file.startsWith(directory, Qt::CaseSensitive);
#endif
}

bool WbFileUtil::isLocatedInInstallationDirectory(const QString &file, bool ignoreAllowModify) {
  const bool inWebots = WbFileUtil::isLocatedInDirectory(file, WbStandardPaths::webotsHomePath());
  return inWebots && (ignoreAllowModify || !WbPreferences::booleanEnvironmentVariable("WEBOTS_ALLOW_MODIFY_INSTALLATION"));
}

void WbFileUtil::searchDirectoryNameRecursively(QStringList &results, const QString &directoryName, const QString &root) {
  QDir dir(root);
  assert(dir.exists());
  QStringList subDirs = dir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
  bool projectFolder = false;
  foreach (const QString &subDir, subDirs) {
    // we don't want to search below a Webots project folder to avoid very long and useless exploration
    if (subDir == "controllers" || subDir == "worlds" || subDir == "protos" || subDir == "plugins" || subDir == "libraries") {
      projectFolder = true;
      break;
    }
  }
  foreach (const QString &subDir, subDirs) {
    QString subDirAbsolutePath = dir.absolutePath() + '/' + subDir;
    if (subDir == directoryName)
      results << subDirAbsolutePath + '/';
    else if (!projectFolder)
      searchDirectoryNameRecursively(results, directoryName, subDirAbsolutePath);
  }
}

bool WbFileUtil::isDirectoryWritable(const QString &path) {
  QDir dir(path);
  // find existing parent directory
  if (!dir.exists()) {
    QString relativePath = "..";
    while (!dir.cd(relativePath))
      relativePath += "/..";
  }

  QString fileName = dir.absolutePath() + "/webots_dummy.txt";

  // remove dummy file if exists
  if (QFile::exists(fileName)) {
    if (!QFile::remove(fileName))
      return false;
  }

  // check if file can be created (and removed)
  QFile dummyFile(fileName);
  bool success = dummyFile.open(QFile::WriteOnly);
  if (success)
    return dummyFile.remove();
  return false;
}

void WbFileUtil::revealInFileManager(const QString &file) {
#ifdef _WIN32
  QProcess::startDetached("explorer", QStringList("/select," + QDir::toNativeSeparators(file)));
#elif defined(__APPLE__)
  QStringList args;
  args << "-e"
       << "tell application \"Finder\""
       << "-e"
       << "activate"
       << "-e"
       << "select POSIX file \"" + file + "\""
       << "-e"
       << "end tell";
  QProcess::startDetached("osascript", args);
#else  // __linux__
  QProcess process;
  process.start("xdg-mime", QStringList() << "query"
                                          << "default"
                                          << "inode/directory");
  process.waitForFinished();
  QString output = process.readLine().simplified();
  if (output == "dolphin.desktop" || output == "org.kde.dolphin.desktop")
    process.startDetached("dolphin", QStringList() << "--select" << file);
  else if (output == "nautilus.desktop" || output == "org.gnome.Nautilus.desktop" ||
           output == "nautilus-folder-handler.desktop")
    process.startDetached("nautilus", QStringList() << "--no-desktop" << file);
  else if (output == "caja-folder-handler.desktop")
    process.startDetached("caja", QStringList() << "--no-desktop" << file);
  else if (output == "nemo.desktop")
    process.startDetached("nemo", QStringList() << "--no-desktop" << file);
  else if (output == "kfmclient_dir.desktop")
    process.startDetached("konqueror", QStringList() << "--select" << file);
  else
    WbDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(file).path()).toString());
#endif
}
