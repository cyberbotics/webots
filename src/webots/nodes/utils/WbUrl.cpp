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
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QUrl>

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
      proto = WbProtoList::instance()->customFindModel(proto->ancestorProtoName(), "");
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

QString WbUrl::computePath(const WbNode *node, const QString &field, const WbMFString *urlField, int index, bool warn) {
  // check if mUrl is empty
  if (urlField->size() < 1)
    return "";

  // get the url at specified index
  const QString &url = urlField->item(index);

  return computePath(node, field, url, warn);
}

QString WbUrl::computePathV2(const WbNode *node, const QString &field, const WbMFString *urlField, int index, bool warn) {
  // check if mUrl is empty
  if (urlField->size() < 1)
    return "";

  // get the url at specified index
  const QString &url = urlField->item(index);

  return computePathV2(node, field, url, warn);
}

QString WbUrl::computePathV2(const WbNode *node, const QString &field, const QString &url, bool warn) {
  printf("NEW computePath:\n");

  printf("  URL  is %s\n", url.toUtf8().constData());
  // cases where no url manipulation is necessary
  if (isWeb(url))
    return url;

  if (QDir::isAbsolutePath(url)) {
    const QString newUrl = QDir::cleanPath(url);
    return newUrl;
  }

  // cases where url manipulation is necessary
  QString externPath;
  const WbNode *protoAncestor = node->protoAncestor();
  if (protoAncestor && protoAncestor->proto())
    externPath = protoAncestor->proto()->externPath();
  else
    externPath = "";
  // assert(0);  // parent is normal node?

  if (isLocalUrl(url)) {
    // manipulate url based on whether it's a descendant of a PROTO
    if (isWeb(externPath)) {
      QRegularExpression re("(https://raw.githubusercontent.com/cyberbotics/webots/[a-zA-Z0-9\\_\\-\\+]+/)");
      QRegularExpressionMatch match = re.match(externPath);
      // TODO: need to match? can't replace?
      assert(match.hasMatch());  // ancestor remote url should match the template
      QString newUrl = url;
      newUrl = newUrl.replace("webots://", match.captured(0));
      printf("  GEN_ParentWeb_ChildLocal is %s\n", newUrl.toUtf8().constData());
      return newUrl;
    }

    if (isLocalUrl(externPath)) {
      const QString newUrl = QDir::cleanPath(WbStandardPaths::webotsHomePath() + url.mid(9));
      printf("  GEN_ParentLocal_ChildLocal is %s\n", newUrl.toUtf8().constData());
      return newUrl;
    }

    // TODO: externPath is local? is abs? possible?
    return missing(url);
  }

  if (QDir::isRelativePath(url)) {
    // TODO: if "./asd/ewer/"?
    if (isWeb(externPath) || isLocalUrl(externPath)) {
      // consume directories in both urls accordingly
      QString assetUrl = url;
      externPath = QUrl(externPath)
                     .adjusted(QUrl::RemoveFilename | QUrl::StripTrailingSlash)
                     .toString();  // remove filename and trailing slash
      while (assetUrl.startsWith("../")) {
        externPath = externPath.left(externPath.lastIndexOf("/"));
        assetUrl.remove(0, 3);
      }
      QString newUrl = externPath + "/" + assetUrl;
      if (isLocalUrl(externPath)) {
        newUrl = QDir::cleanPath(WbStandardPaths::webotsHomePath() + newUrl.mid(9));
        printf("  GEN_ParentLocal_ChildRelative is %s\n", newUrl.toUtf8().constData());
        return newUrl;
      }

      printf("  GEN_ParentWeb_ChildRelative is %s\n", newUrl.toUtf8().constData());
      return newUrl;
    }

    // assert(0);  // dunno
  }

  return missing(url);
}

QString WbUrl::computePath(const WbNode *node, const QString &field, const QString &url, bool warn) {
  printf("OLD computePath: %s\n", url.toUtf8().constData());
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
      const QString &reference = version.commit().isEmpty() ? version.toString() : version.commit();
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

QString WbUrl::generateExternProtoPath(const QString &url, const QString &parentUrl) {
  // TODO: if backward slashes instead of forward?
  // TODO: simplify all this function when it works
  // TODO: if there is typo, ex: webots::/ instead of webots:// ?

  QString path;
  // handle case if the parent proto references a subproto that is itself a remote asset
  // ex: EXTERNPROTO "https://raw.github.com/.../SubNode.proto"
  if (isWeb(url))
    return url;

  // handle situation: EXTERNPROTO "/absolute/path/to/proto/SubNode.proto"
  if (QDir::isAbsolutePath(url))
    return url;  // nothing to do, url is accessible as-is
  else {
    // either the sub-proto is defined relatively or is a local url (webots://)
    QString parentProtoUrl = parentUrl.left(parentUrl.lastIndexOf("/"));
    QString subProtoUrl = url;

    if (isLocalUrl(subProtoUrl)) {
      // if like the sub-proto the parent proto is also local (i.e starts with webots://) then the sub-proto should be
      // available locally. This is the case in the webots development environment.
      if (isLocalUrl(parentProtoUrl) || QDir::isAbsolutePath(parentProtoUrl))
        return QDir::cleanPath(WbStandardPaths::webotsHomePath() + url.mid(9));
      else {
        // handle case where a remote proto references a local (webots://) subproto
        // ex: EXTERNPROTO "webots://.../SubNode.proto"
        QRegularExpression re("(https://raw.githubusercontent.com/cyberbotics/webots/[a-zA-Z0-9\\_\\-\\+]+/)");
        QRegularExpressionMatch match = re.match(parentProtoUrl);
        // TODO: need to match? can't replace?

        assert(match.hasMatch());  // parent remote url should match the template
        return match.captured(0) + subProtoUrl.replace("webots://", "");
      }
    } else if (QDir::isRelativePath(subProtoUrl)) {
      // handle case if the parent proto references a subproto relatively to itself
      // ex: EXTERNPROTO "../../../SubNode.proto"

      // consume directories in both urls accordingly
      while (subProtoUrl.startsWith("../")) {
        parentProtoUrl = parentProtoUrl.left(parentProtoUrl.lastIndexOf("/"));
        subProtoUrl.remove(0, 3);
      }

      // printf(">>%s|||%s]][[%s|||%s]]\n", parentProtoUrl.toUtf8().constData(), subProtoUrl.toUtf8().constData(),
      //       url.toUtf8().constData(), parentUrl.toUtf8().constData());
      // manufacture subproto url based on parent's url
      return parentProtoUrl + "/" + subProtoUrl;
    }
  }

  assert(0);  // unhandled case
  return "";
}

QString WbUrl::exportTexture(const WbNode *node, const QString &url, const QString &sourcePath,
                             const QString &relativeTexturesPath, const WbWriter &writer) {
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

QString WbUrl::exportTexture(const WbNode *node, const WbMFString *urlField, int index, const WbWriter &writer) {
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