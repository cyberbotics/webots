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

#include "WbUrl.hpp"

#include "WbApplicationInfo.hpp"
#include "WbFileUtil.hpp"
#include "WbLog.hpp"
#include "WbMFString.hpp"
#include "WbNode.hpp"
#include "WbNodeUtilities.hpp"
#include "WbProject.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>

namespace {
  QString checkIsFile(const WbNode *node, const QString &field, const QString &path) {
    if (QFileInfo(path).isFile())
      return path;
    if (node)
      node->parsingWarn(QObject::tr("First item of '%1' field expected to be a file but is a directory.").arg(field));
    else
      WbLog::warning(QObject::tr("'%1' expected to be a file but is a directory.").arg(field), false, WbLog::PARSING);
    return "";
  }
}  // namespace

QStringList WbUrl::orderedSearchPaths(const WbNode *node) {
  // retrieve PROTOs search paths
  // - if project PROTO add search path before world path
  // - if Webots PROTO add search path after world path
  QStringList projectPROTOSearchPath;
  QStringList webotsPROTOSearchPath;
  WbNode *currentNode = const_cast<WbNode *>(node);
  while (currentNode) {
    WbProtoModel *proto = WbNodeUtilities::findContainingProto(currentNode);
    while (proto) {
      if (!proto->path().isEmpty()) {
        if (proto->path().startsWith(WbProject::current()->worldsPath())) {
          if (projectPROTOSearchPath.contains(proto->path()))
            projectPROTOSearchPath.append(proto->path());
        } else if (!webotsPROTOSearchPath.contains(proto->path()))
          webotsPROTOSearchPath.append(proto->path());
      }
      if (!proto->projectPath().isEmpty() && !projectPROTOSearchPath.contains(proto->projectPath() + "/protos"))
        projectPROTOSearchPath.append(proto->projectPath() + "/protos");
      proto = WbProtoList::current()->findModel(proto->ancestorProtoName(), "");
    }
    currentNode = currentNode->parentNode();
  }

  QStringList searchPaths;
  searchPaths << projectPROTOSearchPath;
  searchPaths.append(WbProject::current()->worldsPath());
  if (WbProject::extraDefaultProject())
    searchPaths.append(WbProject::extraDefaultProject()->worldsPath());
  searchPaths << webotsPROTOSearchPath;
  searchPaths.append(WbStandardPaths::projectsPath() + "default/worlds");
  return searchPaths;
}

const QString WbUrl::missingTexture() {
  return WbStandardPaths::resourcesPath() + "images/missing_texture.png";
}

const QString WbUrl::missing(const QString &url) {
  const QString suffix = QFileInfo(url).suffix();
  const QStringList textureSuffixes = {"png", "jpg", "jpeg"};
  if (textureSuffixes.contains(suffix, Qt::CaseInsensitive))
    return missingTexture();
  return "";
}

QString WbUrl::computePath(const WbNode *node, const QString &field, const WbMFString *urlField, int index, bool warn) {
  // check if mUrl is empty
  if (urlField->size() < 1)
    return "";

  // get the url at specified index
  const QString &url = urlField->item(index);

  return computePath(node, field, url, warn);
}

QString WbUrl::computePath(const WbNode *node, const QString &field, const QString &url, bool warn) {
  // check if the first url is empty
  if (url.isEmpty()) {
    if (node)
      node->parsingWarn(QObject::tr("First item of '%1' field is empty.").arg(field));
    else
      WbLog::warning(QObject::tr("Missing '%1' value.").arg(field), false, WbLog::PARSING);
    return missing(url);
  }

  if (isWeb(url))
    return url;

  QString path;
  if (isLocalUrl(url))
    path = QDir::cleanPath(WbStandardPaths::webotsHomePath() + url.mid(9));
  else if (QDir::isAbsolutePath(url))  // check if the url is an absolute path
    path = QDir::cleanPath(url);

  if (!path.isEmpty()) {
    if (QFileInfo(path).exists())
      return checkIsFile(node, field, path);

    if (isLocalUrl(url)) {
      QString newUrl(url);
      const WbVersion &version = WbApplicationInfo::version();
      // if it's an official release, use the tag (for example R2022b), if it's a nightly use the commit
      const QString reference = version.commit().isEmpty() ? version.toString() : version.commit();
      newUrl.replace("webots://", "https://raw.githubusercontent.com/cyberbotics/webots/" + reference + "/");
      return newUrl;
    }

    const QString error = QObject::tr("'%1' not found.").arg(url);
    if (node)
      node->parsingWarn(error);
    else
      WbLog::warning(error, false, WbLog::PARSING);
    return missing(url);
  }

  // check if the url is defined relatively

  QStringList searchPaths = orderedSearchPaths(node);
  foreach (const QString &path, searchPaths) {
    QDir dir(path);
    if (dir.exists(url))
      return checkIsFile(node, field, QDir::cleanPath(dir.absoluteFilePath(url)));
  }

  if (warn) {
    const QString warning =
      QObject::tr("'%1' not found.").arg(url) + "\n" +
      QObject::tr(
        "A resource file can be defined relatively to the worlds directory of the current project, relatively to the worlds "
        "directory of the default project, relatively to its protos directory (if defined in a PROTO), or absolutely.");
    if (node)
      node->parsingWarn(warning);
    else
      WbLog::warning(warning, false, WbLog::PARSING);
  }

  return missing(url);
}

QString WbUrl::exportTexture(const WbNode *node, const QString &url, const QString &sourcePath,
                             const QString &relativeTexturesPath, const WbVrmlWriter &writer) {
  const QFileInfo urlFileInfo(url);
  const QString fileName = urlFileInfo.fileName();
  const QString expectedRelativePath = relativeTexturesPath + fileName;
  const QString expectedPath = writer.path() + "/" + expectedRelativePath;

  if (expectedPath == sourcePath)  // everything is fine, the file is where we expect it
    return expectedPath;

  // otherwise, we need to copy it
  // but first, we need to check that folders exists and create them if they don't exist
  const QFileInfo fi(expectedPath);
  if (!QDir(fi.path()).exists())
    QDir(writer.path()).mkpath(fi.path());

  if (QFile::exists(expectedPath)) {
    if (WbFileUtil::areIdenticalFiles(sourcePath, expectedPath))
      return expectedRelativePath;
    else {
      const QString baseName = urlFileInfo.completeBaseName();
      const QString extension = urlFileInfo.suffix();

      for (int i = 1; i < 100; ++i) {  // number of trials before failure
        const QString newRelativePath = writer.relativeTexturesPath() + baseName + '.' + QString::number(i) + '.' + extension;
        const QString newAbsolutePath = writer.path() + "/" + newRelativePath;
        if (QFileInfo(newAbsolutePath).exists()) {
          if (WbFileUtil::areIdenticalFiles(sourcePath, newAbsolutePath))
            return newRelativePath;
        } else {
          QFile::copy(sourcePath, newAbsolutePath);
          return newRelativePath;
        }
      }

      node->warn(QObject::tr("Texture export fails, because too much textures are sharing the same name: %1.").arg(url));
      return "";
    }
  } else {  // simple case
    QFile::copy(sourcePath, expectedPath);
    return expectedRelativePath;
  }
}

QString WbUrl::exportTexture(const WbNode *node, const WbMFString *urlField, int index, const WbVrmlWriter &writer) {
  // in addition to writing the node, we want to ensure that the texture file exists
  // at the expected location. If not, we should copy it, possibly creating the expected
  // directory structure.
  return exportTexture(node, QDir::fromNativeSeparators(urlField->item(index)), computePath(node, "url", urlField, index),
                       writer.relativeTexturesPath(), writer);
}

bool WbUrl::isWeb(const QString &url) {
  return url.startsWith("https://") || url.startsWith("http://");
}

bool WbUrl::isLocalUrl(const QString &url) {
  return url.startsWith("webots://");
}
