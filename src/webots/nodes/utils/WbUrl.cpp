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

#include "WbUrl.hpp"

#include "WbApplicationInfo.hpp"
#include "WbFileUtil.hpp"
#include "WbLog.hpp"
#include "WbMFString.hpp"
#include "WbNode.hpp"
#include "WbNodeUtilities.hpp"
#include "WbProject.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QUrl>

QStringList WbUrl::orderedSearchPaths(const WbNode *node) {  // TODO: remove?
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
      proto = WbProtoManager::instance()->findModel(proto->ancestorProtoName(), "");
    }
    currentNode = currentNode->parentNode();
  }

  QStringList searchPaths;
  searchPaths << projectPROTOSearchPath;
  searchPaths.append(WbProject::current()->worldsPath());
  foreach (const WbProject *extraProject, *WbProject::extraProjects())
    searchPaths.append(extraProject->worldsPath());
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

/*
QString WbUrl::computePath(const WbNode *node, const QString &field, const WbMFString *urlField, int index, bool warn) {
  // check if mUrl is empty
  if (urlField->size() < 1)
    return "";

  // get the url at specified index
  const QString &url = urlField->item(index);

  return computePath(node, field, url, warn);
}
*/

QString WbUrl::computePath(const WbNode *node, const QString &field, const WbMFString *urlField, int index, bool warn) {
  // check if mUrl is empty
  if (urlField->size() < 1)
    return "";

  // get the url at specified index
  const QString &url = urlField->item(index);

  return computePath(node, field, url, warn);
}

QString WbUrl::computePath(const WbNode *node, const QString &field, const QString &rawUrl, bool warn) {
  // TODO: cleanse and make to see if there's warnings

  // use cross-platform forward slashes
  QString url = rawUrl;
  url = url.replace("\\", "/");

  // cases where no url manipulation is necessary
  if (isWeb(url))
    return url;

  if (QDir::isAbsolutePath(url))
    return QDir::cleanPath(url);

  // cases where url manipulation is necessary
  QString externPath;
  const WbNode *protoAncestor = node->isProtoInstance() ? node : node->protoAncestor();
  if (protoAncestor && protoAncestor->proto())
    externPath = protoAncestor->proto()->externPath();
  else {
    // the asset is not referenced with respect to a PROTO file (only have base nodes as ancestors)
    if (isLocalUrl(url))
      return QDir::cleanPath(WbStandardPaths::webotsHomePath() + url.mid(9));

    if (QDir::isRelativePath(url)) {  // then it must be relative to the world itself
      QString worldsPath = WbProject::current()->worldsPath();
      worldsPath.chop(1);  // remove trailing slash

      // consume directories accordingly
      while (url.startsWith("../")) {
        worldsPath = worldsPath.left(worldsPath.lastIndexOf("/"));
        url.remove(0, 3);
      }

      return worldsPath + "/" + url;
    }

    return missing(url);
  }

  // the asset has a PROTO ancestor
  return generateExternProtoPath(url, externPath);
}

QString WbUrl::generateExternProtoPath(const QString &rawUrl, const QString &rawParentUrl) {
  // TODO: if there is typo, ex: webots::/ instead of webots:// ?

  // use cross-platform forward slashes
  QString url = rawUrl;
  url = url.replace("\\", "/");
  QString parentUrl = rawParentUrl;
  parentUrl = parentUrl.replace("\\", "/");

  // cases where no url manipulation is necessary
  if (isWeb(url))
    return url;

  if (QDir::isAbsolutePath(url))
    return QDir::cleanPath(url);

  // the asset has a PROTO ancestor
  if (isLocalUrl(url)) {
    // infer url based on PROTO ancestor's url
    if (isWeb(parentUrl)) {
      QRegularExpression re("(https://raw.githubusercontent.com/cyberbotics/webots/[a-zA-Z0-9\\_\\-\\+]+/)");
      QRegularExpressionMatch match = re.match(parentUrl);
      if (!match.hasMatch()) {
        WbLog::error(QObject::tr("The cascaded url inferring mechanism is supported only for official webots assets."));
        return missing(url);
      }
      return url.replace("webots://", match.captured(0));
    }

    if (isLocalUrl(parentUrl) || parentUrl.isEmpty() || QDir::isAbsolutePath(parentUrl))
      return QDir::cleanPath(WbStandardPaths::webotsHomePath() + url.mid(9));

    return missing(url);
  }

  if (QDir::isRelativePath(url)) {
    // for relative urls, begin by searching relative to the world and protos folders
    QStringList searchPaths = QStringList() << WbProject::current()->worldsPath() << WbProject::current()->protosPath();
    foreach (const QString &path, searchPaths) {
      QDir dir(path);
      if (dir.exists(url))
        return QDir::cleanPath(dir.absoluteFilePath(url));
    }

    // if it is not available in those folders, infer the url based on the parent's url
    if (isWeb(parentUrl) || QDir::isAbsolutePath(parentUrl) || isLocalUrl(parentUrl)) {
      // remove filename and trailing slash from parent url
      parentUrl = QUrl(parentUrl).adjusted(QUrl::RemoveFilename | QUrl::StripTrailingSlash).toString();

      if (url.startsWith("./"))
        url.remove(0, 2);

      // consume directories in both urls accordingly
      while (url.startsWith("../")) {
        parentUrl = parentUrl.left(parentUrl.lastIndexOf("/"));
        url.remove(0, 3);
      }

      const QString newUrl = parentUrl + "/" + url;
      if (isWeb(parentUrl) || QDir::isAbsolutePath(parentUrl))
        return newUrl;

      if (isLocalUrl(parentUrl))
        return QDir::cleanPath(WbStandardPaths::webotsHomePath() + newUrl.mid(9));
    }
  }

  return missing(url);
}

QString WbUrl::exportResource(const WbNode *node, const QString &url, const QString &sourcePath,
                              const QString &relativeResourcePath, const WbWriter &writer, const bool isTexture) {
  const QFileInfo urlFileInfo(url);
  const QString fileName = urlFileInfo.fileName();
  const QString expectedRelativePath = relativeResourcePath + fileName;
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
        QString newRelativePath;
        if (isTexture)
          newRelativePath = writer.relativeTexturesPath() + baseName + '.' + QString::number(i) + '.' + extension;
        else
          newRelativePath = writer.relativeMeshesPath() + baseName + '.' + QString::number(i) + '.' + extension;

        const QString newAbsolutePath = writer.path() + "/" + newRelativePath;
        if (QFileInfo(newAbsolutePath).exists()) {
          if (WbFileUtil::areIdenticalFiles(sourcePath, newAbsolutePath))
            return newRelativePath;
        } else {
          QFile::copy(sourcePath, newAbsolutePath);
          return newRelativePath;
        }
      }
      if (isTexture)
        node->warn(QObject::tr("Failure exporting texture, too many textures share the same name: %1.").arg(url));
      else
        node->warn(QObject::tr("Failure exporting mesh, too many meshes share the same name: %1.").arg(url));

      return "";
    }
  } else {  // simple case
    QFile::copy(sourcePath, expectedPath);
    return expectedRelativePath;
  }
}

QString WbUrl::exportTexture(const WbNode *node, const WbMFString *urlField, int index, const WbWriter &writer) {
  // in addition to writing the node, we want to ensure that the texture file exists
  // at the expected location. If not, we should copy it, possibly creating the expected
  // directory structure.
  return exportResource(node, QDir::fromNativeSeparators(urlField->item(index)), computePath(node, "url", urlField, index),
                        writer.relativeTexturesPath(), writer);
}

QString WbUrl::exportMesh(const WbNode *node, const WbMFString *urlField, int index, const WbWriter &writer) {
  // in addition to writing the node, we want to ensure that the mesh file exists
  // at the expected location. If not, we should copy it, possibly creating the expected
  // directory structure.
  return exportResource(node, QDir::fromNativeSeparators(urlField->item(index)), computePath(node, "url", urlField, index),
                        writer.relativeMeshesPath(), writer, false);
}

bool WbUrl::isWeb(const QString &url) {
  return url.startsWith("https://") || url.startsWith("http://");
}

bool WbUrl::isLocalUrl(const QString &url) {
  return url.startsWith("webots://");
}