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

#include "WbProtoManager.hpp"

#include "WbApplicationInfo.hpp"
#include "WbFieldModel.hpp"
#include "WbFileUtil.hpp"
#include "WbLog.hpp"
#include "WbMultipleValue.hpp"
#include "WbNetwork.hpp"
#include "WbNode.hpp"
#include "WbParser.hpp"
#include "WbProject.hpp"
#include "WbProtoModel.hpp"
#include "WbProtoTreeItem.hpp"
#include "WbSFNode.hpp"
#include "WbStandardPaths.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"
#include "WbUrl.hpp"
#include "WbVrmlNodeUtilities.hpp"

#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QRegularExpression>
#include <QtCore/QUrl>
#include <QtCore/QXmlStreamReader>

static WbProtoManager *gInstance = NULL;

WbProtoManager *WbProtoManager::instance() {
  if (!gInstance)
    gInstance = new WbProtoManager();
  return gInstance;
}

WbProtoManager::WbProtoManager() {
  mTreeRoot = NULL;

  mImportedFromSupervisor = false;

  loadWebotsProtoMap();

  // set 1/1/1970 by default to force a generation of the WbProtoInfos the first time
  mProtoInfoGenerationTime.insert(PROTO_WORLD, QDateTime::fromSecsSinceEpoch(0));
  mProtoInfoGenerationTime.insert(PROTO_PROJECT, QDateTime::fromSecsSinceEpoch(0));
  mProtoInfoGenerationTime.insert(PROTO_EXTRA, QDateTime::fromSecsSinceEpoch(0));
}

// we do not delete the PROTO models here: each PROTO model is automatically deleted when its last PROTO instance is deleted
WbProtoManager::~WbProtoManager() {
  cleanup();

  if (gInstance == this)
    gInstance = NULL;
}

WbProtoModel *WbProtoManager::readModel(const QString &url, const QString &worldPath, const QString &prefix,
                                        const QStringList &baseTypeList) const {
  WbTokenizer tokenizer;
  const QString path = WbUrl::isWeb(url) ? WbNetwork::instance()->get(url) : url;
  int errors = tokenizer.tokenize(path, prefix);
  if (errors > 0)
    return NULL;

  WbParser parser(&tokenizer);
  if (!parser.parseProtoInterface(worldPath))
    return NULL;

  tokenizer.rewind();
  while (tokenizer.peekWord() == "EXTERNPROTO" || tokenizer.peekWord() == "IMPORTABLE")  // consume EXTERNPROTO declarations
    parser.skipExternProto();

  const bool prevInstantiateMode = WbNode::instantiateMode();
  try {
    WbNode::setInstantiateMode(false);
    WbProtoModel *model = new WbProtoModel(&tokenizer, worldPath, url, prefix, baseTypeList);
    WbNode::setInstantiateMode(prevInstantiateMode);
    return model;
  } catch (...) {
    WbNode::setInstantiateMode(prevInstantiateMode);
    return NULL;
  }
}

WbProtoModel *WbProtoManager::findModel(const QString &modelName, const QString &worldPath, const QString &parentFilePath,
                                        const QStringList &baseTypeList) {
  if (modelName.isEmpty())
    return NULL;

  assert(!parentFilePath.isEmpty());  // cannot find a model unless we know where to look

  QString protoDeclaration;

  // nodes imported from a supervisor should only check the IMPORTABLE list
  if (!mImportedFromSupervisor) {
    // check the clipboard buffer
    if (protoDeclaration.isEmpty() && !mExternProtoClipboardBuffer.isEmpty()) {
      foreach (const WbExternProto *item, mExternProtoClipboardBuffer) {
        if (item->name() == modelName)
          protoDeclaration = item->url();
      }
    }

    // determine the location of the PROTO based on the EXTERNPROTO declaration in the parent file
    if (protoDeclaration.isEmpty())
      protoDeclaration = findExternProtoDeclarationInFile(parentFilePath, modelName);
  }

  // for IMPORTABLE proto nodes the declaration is in the EXTERNPROTO list, nodes added with add-node follow a different pipe
  if (protoDeclaration.isEmpty()) {
    foreach (const WbExternProto *proto, mExternProto) {
      if (proto->name() == modelName && proto->isImportable())
        protoDeclaration = proto->url();
    }
    // for supervisor imported nodes, only the first level should be exclusively checked in the IMPORTABLE list
    mImportedFromSupervisor = false;
  }

  // based on the declaration found in the file or in the mExternProto list, check if it's a known model
  if (!protoDeclaration.isEmpty()) {
    foreach (WbProtoModel *model, mModels) {
      // if the resolved url is one among the known ones, return the model
      if (WbUrl::resolveUrl(model->url()) == WbUrl::combinePaths(protoDeclaration, parentFilePath))
        return model;
    }
  }

  // if the protoDeclaration is still empty then none was given so find a valid one using the backwards compatibility mechanism
  if (protoDeclaration.isEmpty()) {
    protoDeclaration = injectDeclarationByBackwardsCompatibility(modelName);
    bool foundProtoVersion = false;
    const WbVersion protoVersion = checkProtoVersion(parentFilePath, &foundProtoVersion);
    if (foundProtoVersion && protoVersion < WbVersion(2022, 1, 0)) {
      const QString backwardsCompatibilityMessage = tr("Please adapt your project to R2023b following these instructions: "
                                                       "https://cyberbotics.com/doc/guide/upgrading-webots");
      const QString outdatedProtoMessage =
        tr("'%1' must be converted because EXTERNPROTO declarations are missing.").arg(parentFilePath);
      displayMissingDeclarations(backwardsCompatibilityMessage);
      displayMissingDeclarations(outdatedProtoMessage);
    } else {
      QString url;
      if (protoDeclaration.isEmpty() && isProtoInCategory(modelName, PROTO_WEBOTS))
        url = mWebotsProtoList.value(modelName)->url();
      else if (WbUrl::isWeb(protoDeclaration))
        url = protoDeclaration;
      else
        url = QDir(QFileInfo(mCurrentWorld).absolutePath()).relativeFilePath(protoDeclaration);
      const QString errorMessage =
        (!protoDeclaration.isEmpty() || isProtoInCategory(modelName, PROTO_WEBOTS)) ?
          tr("Missing declaration for '%1', add: 'EXTERNPROTO \"%2\"' to '%3'.").arg(modelName).arg(url).arg(parentFilePath) :
          tr("Missing declaration for '%1', unknown node.").arg(modelName);

      displayMissingDeclarations(errorMessage);
    }
    if (protoDeclaration.isEmpty())
      return NULL;
  }

  // a PROTO declaration is provided, enforce it
  QString modelPath;  // how the PROTO is referenced
  if (WbUrl::isWeb(protoDeclaration) && WbNetwork::instance()->isCachedWithMapUpdate(modelPath))
    modelPath = protoDeclaration;
  else if (WbUrl::isLocalUrl(protoDeclaration) || QDir::isRelativePath(protoDeclaration)) {
    // two possibitilies arise if the declaration is local (webots://)
    // 1. the parent PROTO is in the cache (all its references are always 'webots://'): it may happen if a PROTO references
    // another PROTO (both being cached)
    // 2. the PROTO is actually locally available
    // option (1) needs to be checked first, otherwise in the webots development environment the declarations aren't
    // respected (since a local version of the PROTO exists virtually every time)
    QString parentFile;
    if (WbFileUtil::isLocatedInDirectory(parentFilePath, WbStandardPaths::cachedAssetsPath()))
      // reverse lookup the file in order to establish its original remote path
      parentFile = WbNetwork::instance()->getUrlFromEphemeralCache(parentFilePath);
    else
      parentFile = parentFilePath;

    // extract the prefix from the parent so that we can build the child's path accordingly
    if (WbUrl::isWeb(parentFile)) {
      const QRegularExpression re(WbUrl::remoteWebotsAssetRegex(true));
      const QRegularExpressionMatch match = re.match(parentFile);
      if (match.hasMatch()) {
        if (WbUrl::isLocalUrl(protoDeclaration))  // replace the prefix (webots://) based on the parent's prefix
          modelPath = protoDeclaration.replace("webots://", match.captured(0));
        else  // if it's a relative url, then manufacture a remote url based on the relative path and the parent's path
          modelPath = WbUrl::combinePaths(protoDeclaration, parentFile);
        // if the PROTO tree was built correctly, by definition the child must be cached already too
        assert(WbNetwork::instance()->isCachedNoMapUpdate(modelPath));
      } else {
        WbLog::error(tr("The cascaded URL inferring mechanism is supported only for official Webots assets."));
        return NULL;
      }
    }
  }

  if (modelPath.isEmpty()) {
    assert(QFileInfo(parentFilePath).exists());
    modelPath = WbUrl::combinePaths(protoDeclaration, parentFilePath);
  }
  // determine prefix and disk location from modelPath
  const QString modelDiskPath = WbUrl::isWeb(modelPath) && WbNetwork::instance()->isCachedWithMapUpdate(modelPath) ?
                                  WbNetwork::instance()->get(modelPath) :
                                  modelPath;
  const QString prefix = WbUrl::computePrefix(modelPath);  // used to retrieve remote assets (replaces webots:// in the body)

  if (!modelPath.isEmpty() && QFileInfo(modelDiskPath).exists()) {
    WbProtoModel *model = readModel(modelPath, worldPath, prefix, baseTypeList);
    if (model == NULL)  // can occur if the PROTO contains errors
      return NULL;
    mModels << model;
    model->ref();
    return model;
  }

  return NULL;
}

QString WbProtoManager::findExternProtoDeclarationInFile(const QString &url, const QString &modelName) {
  if (url.isEmpty())
    return QString();

  QFile file(WbUrl::isWeb(url) ? WbNetwork::instance()->get(url) : url);
  if (!file.open(QIODevice::ReadOnly)) {
    WbLog::error(tr("Could not search for EXTERNPROTO declarations in '%1' because the file is not readable.").arg(url));
    return QString();
  }
  QString identifier = modelName;
  identifier.replace("+", "\\+").replace("-", "\\-").replace("_", "\\_");
  const QString regex = QString("^\\s*(?:IMPORTABLE\\s+)?EXTERNPROTO\\s+\"(.*(?:/|\\\\|(?<=\"))%1\\.proto)\"").arg(identifier);
  const QRegularExpression re(regex, QRegularExpression::MultilineOption);
  QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

  while (it.hasNext()) {
    const QRegularExpressionMatch match = it.next();
    if (match.hasMatch())
      return match.captured(1);
  }

  return QString();
}

QMap<QString, QString> WbProtoManager::undeclaredProtoNodes(const QString &filename) {
  QMap<QString, QString> protoNodeList;

  if (!filename.endsWith(".wbt", Qt::CaseInsensitive))
    return protoNodeList;

  WbTokenizer tokenizer;
  tokenizer.tokenize(filename);
  WbParser parser(&tokenizer);

  // only apply mechanism to worlds prior to R2022b
  if (tokenizer.fileVersion() >= WbVersion(2022, 1, 0))
    return protoNodeList;
  // fill queue with nodes referenced by the world file
  QStringList queue;
  queue << parser.protoNodeList();

  displayMissingDeclarations(tr("Please adapt your project to R2023b following these instructions: "
                                "https://cyberbotics.com/doc/guide/upgrading-webots"));

  // list all PROTO nodes which are known
  QMap<QString, QString> localProto;
  foreach (QString path, listProtoInCategory(PROTO_PROJECT))
    localProto.insert(QFileInfo(path).baseName(), path);
  QMap<QString, QString> extraProto;
  foreach (QString path, listProtoInCategory(PROTO_EXTRA))
    extraProto.insert(QFileInfo(path).baseName(), path);
  QMap<QString, QString> webotsProto;
  foreach (QString path, listProtoInCategory(PROTO_WEBOTS))
    webotsProto.insert(QUrl(path).fileName().replace(".proto", "", Qt::CaseInsensitive), path);

  QStringList knownProto;
  knownProto << localProto.keys() << extraProto.keys() << webotsProto.keys();
  knownProto.removeDuplicates();

  while (!queue.empty()) {
    const QString proto = queue.front();
    queue.pop_front();

    // determine PROTO location
    QString url;
    if (localProto.contains(proto))  // check if it's a PROTO local to the project
      url = localProto.value(proto);
    else if (extraProto.contains(proto))  // check if it's a PROTO local to the extra projects
      url = extraProto.value(proto);
    else if (webotsProto.contains(proto))  // if all else fails, use the official webots proto
      url = webotsProto.value(proto);
    else
      continue;

    url = WbUrl::resolveUrl(url);
    assert(url.endsWith(".proto", Qt::CaseInsensitive));

    if (!protoNodeList.contains(proto))
      protoNodeList.insert(proto, url);

    if (!WbUrl::isWeb(url)) {
      // open the PROTO file and extract all PROTO nodes it refrerences by brute force (comparing against the known nodes)
      QFile file(url);
      if (file.open(QIODevice::ReadOnly)) {
        const QByteArray &contents = file.readAll();
        foreach (const QString &item, knownProto) {
          QString identifier = item;
          identifier.replace("+", "\\+").replace("-", "\\-").replace("_", "\\_");
          const QRegularExpression re(identifier + "\\s*\\{");
          const QRegularExpressionMatch match = re.match(contents);
          if (match.hasMatch() && !protoNodeList.contains(item))
            queue << item;  // these nodes need to further be analyzed to see what they depend on
        }
      }
    }
  }

  return protoNodeList;
}

void WbProtoManager::retrieveExternProto(const QString &filename, bool reloading) {
  // clear current project related variables
  cleanup();
  mCurrentWorld = filename;
  mReloading = reloading;

  // set the world file as the root of the tree
  mTreeRoot = new WbProtoTreeItem(filename, NULL, false);
  connect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoManager::loadWorld);

  // backwards compatibility mechanism: populate the tree with urls which are not declared by EXTERNPROTO (worlds < R2022b)
  QMapIterator<QString, QString> it(undeclaredProtoNodes(filename));
  while (it.hasNext())
    mTreeRoot->insert(it.next().value());

  // root node of the tree is fully populated, trigger cascaded download
  mTreeRoot->download();
}

void WbProtoManager::retrieveExternProto(const QString &filename) {
  // set the proto file as the root of the tree
  mTreeRoot = new WbProtoTreeItem(filename, NULL, false);
  connect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoManager::protoRetrievalCompleted);
  // trigger download
  mTreeRoot->download();
}

void WbProtoManager::retrieveLocalProtoDependencies() {
  QStringList dependencies;
  // current project
  QDirIterator it(WbProject::current()->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext())
    dependencies << it.next();
  // extra projects
  foreach (const WbProject *project, *WbProject::extraProjects()) {
    QDirIterator i(project->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
    while (i.hasNext())
      dependencies << i.next();
  }

  // create an empty root and populate its children with the dependencies to be downloaded
  mTreeRoot = new WbProtoTreeItem("", NULL, false);
  foreach (const QString &proto, dependencies)
    mTreeRoot->insert(proto);

  connect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoManager::dependenciesAvailable);
  // trigger parallel download
  mTreeRoot->download();
}

void WbProtoManager::protoRetrievalCompleted() {
  disconnect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoManager::protoRetrievalCompleted);

  mTreeRoot->generateSessionProtoList(mSessionProto);
  mTreeRoot->deleteLater();

  emit retrievalCompleted();
}

void WbProtoManager::loadWorld() {
  disconnect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoManager::loadWorld);
  if (!mTreeRoot->error().isEmpty()) {
    foreach (const QString &error, mTreeRoot->error())
      WbLog::error(error);
  }

  // generate mSessionProto based on the resulting tree
  mTreeRoot->generateSessionProtoList(mSessionProto);

  // determine what changed from the previous session (and therefore what is no longer needed)
  QSet<QString> difference = QSet<QString>(mPreviousSessionProto.begin(), mPreviousSessionProto.end())
                               .subtract(QSet<QString>(mSessionProto.begin(), mSessionProto.end()));

  QList<WbProtoModel *>::iterator modelIt = mModels.begin();
  while (modelIt != mModels.end()) {
    // remove all local models as they may have changed and all items of the previous session which are no longer needed
    if (!WbUrl::isWeb((*modelIt)->url()) || difference.contains((*modelIt)->url()))
      modelIt = mModels.erase(modelIt);
    else
      ++modelIt;
  }

  // declare all root PROTO defined at the world level, and inferred by backwards compatibility, to the list of EXTERNPROTO
  foreach (const WbProtoTreeItem *const child, mTreeRoot->children())
    declareExternProto(child->name(), child->url(), child->isImportable());

  // cleanup and load world at last
  mTreeRoot->deleteLater();
  emit worldLoadCompleted(mCurrentWorld, mReloading, true);
}

void WbProtoManager::loadWebotsProtoMap() {
  if (!mWebotsProtoList.empty())
    return;  // Webots proto list already generated

  const QString filename(WbStandardPaths::resourcesPath() + QString("proto-list.xml"));
  QFile file(filename);
  if (!file.open(QFile::ReadOnly | QFile::Text)) {
    WbLog::error(tr("Cannot read file '%1'.").arg(filename));
    return;
  }

  QXmlStreamReader reader(&file);
  if (reader.readNextStartElement()) {
    if (reader.name().toString() == "proto-list") {
      while (reader.readNextStartElement()) {
        if (reader.name().toString() == "proto") {
          bool needsRobotAncestor = false;
          QString name, url, baseType, license, licenseUrl, documentationUrl, description, slotType;
          QStringList tags, parameters;
          while (reader.readNextStartElement()) {
            if (reader.name().toString() == "name") {
              name = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "base-type") {
              baseType = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "url") {
              url = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "license") {
              license = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "license-url") {
              licenseUrl = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "documentation-url") {
              documentationUrl = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "description") {
              description = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "slot-type") {
              slotType = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "tags") {
              tags = reader.readElementText().split(',', Qt::SkipEmptyParts);
              reader.readNext();
            }
            if (reader.name().toString() == "parameters") {
              parameters = reader.readElementText().split("\\n", Qt::SkipEmptyParts);
              reader.readNext();
            }
            if (reader.name().toString() == "needs-robot-ancestor") {
              needsRobotAncestor = reader.readElementText() == "true";
              reader.readNext();
            }
          }
          description = description.replace("\\n", "\n");
          WbProtoInfo *const info = new WbProtoInfo(url, baseType, license, licenseUrl, documentationUrl, description, slotType,
                                                    tags, parameters, needsRobotAncestor);
          mWebotsProtoList.insert(name, info);
        } else
          reader.raiseError(tr("Expected 'proto' element."));
      }
    } else
      reader.raiseError(tr("Expected 'proto-list' element."));
  } else
    reader.raiseError(tr("The format of 'proto-list.xml' is invalid."));
}

void WbProtoManager::generateProtoInfoMap(int category, bool regenerate) {
  if (!regenerate)
    return;

  QMap<QString, WbProtoInfo *> *map;
  switch (category) {
    case PROTO_WORLD:
      map = &mWorldFileProtoList;
      break;
    case PROTO_PROJECT:
      map = &mProjectProtoList;
      break;
    case PROTO_EXTRA:
      map = &mExtraProtoList;
      break;
    case PROTO_WEBOTS:
      return;  // note: mWebotsProtoList is loaded, not generated
    default:
      WbLog::error(tr("Cannot select proto list, unknown category '%1'.").arg(category));
      return;
  }

  // flag all as dirty
  QMapIterator<QString, WbProtoInfo *> it(*map);
  while (it.hasNext())
    it.next().value()->setDirty(true);

  // find all proto and instantiate the nodes to build WbProtoInfo (if necessary)
  const QStringList protos = listProtoInCategory(category);
  const QDateTime lastGenerationTime = mProtoInfoGenerationTime.value(category);
  foreach (const QString &protoPath, protos) {
    if (!QFileInfo(protoPath).exists())
      continue;  // PROTO was deleted

    QString protoInferredPath;
    QString protoName;
    const bool isCachedWithMapUpdateProto = WbFileUtil::isLocatedInDirectory(protoPath, WbStandardPaths::cachedAssetsPath());
    if (isCachedWithMapUpdateProto) {  // cached file, infer name from reverse lookup
      protoInferredPath = WbNetwork::instance()->getUrlFromEphemeralCache(protoPath);
      protoName = QUrl(protoInferredPath).fileName().replace(".proto", "", Qt::CaseInsensitive);
    } else {
      protoInferredPath = protoPath;
      protoName = QFileInfo(protoPath).baseName();
    }

    if (!map->contains(protoName) || (QFileInfo(protoPath).lastModified() > lastGenerationTime)) {
      // if it exists but is just out of date, remove previous information
      if (map->contains(protoName)) {
        const WbProtoInfo *info = map->take(protoName);  // remove element from map
        delete info;
      }

      WbProtoInfo *info;
      const bool isWebotsProto = isProtoInCategory(protoName, PROTO_WEBOTS) &&
                                 (WbUrl::resolveUrl(protoUrl(protoName, PROTO_WEBOTS)) == WbUrl::resolveUrl(protoInferredPath));
      // for distributions, the official PROTO can be used only if it is in the cache, which is not the case in the development
      // environment
      if (isWebotsProto && (isCachedWithMapUpdateProto || WbUrl::isLocalUrl(protoPath)))
        // the proto is an official one, both in name and url, so copy the info from the one provided in proto-list.xml
        // note: a copy is necessary because other categories can be deleted, but the webots one can't and shouldn't
        info = new WbProtoInfo(*protoInfo(protoName, PROTO_WEBOTS));
      else
        // generate from file and insert it
        info = generateInfoFromProtoFile(protoPath);

      if (info) {
        info->setDirty(false);
        map->insert(protoName, info);
      }
    } else  // no info change necessary
      map->value(protoName)->setDirty(false);
  }

  // delete anything that is still flagged as dirty (it might happen if the PROTO no longer exists in the folders)
  it.toFront();
  while (it.hasNext()) {
    if (it.next().value()->isDirty()) {
      const WbProtoInfo *info = map->take(it.key());  // remove element
      delete info;
    }
  }

  mProtoInfoGenerationTime.remove(category);
  mProtoInfoGenerationTime.insert(category, QDateTime::currentDateTime());
}

QStringList WbProtoManager::listProtoInCategory(int category) const {
  QStringList protos;

  switch (category) {
    case PROTO_WORLD: {
      for (int i = 0; i < mExternProto.size(); ++i) {
        QString protoPath(mExternProto[i]->url());
        // mExternProto contains raw paths, retrieve corresponding disk file
        if (WbUrl::isWeb(protoPath) && WbNetwork::instance()->isCachedWithMapUpdate(protoPath))
          protoPath = WbNetwork::instance()->get(protoPath);

        protos << protoPath;
      }
      break;
    }
    case PROTO_PROJECT: {
      QDirIterator it(WbProject::current()->protosPath(), QStringList() << "*.proto", QDir::Files,
                      QDirIterator::Subdirectories);

      while (it.hasNext())
        protos << it.next();

      break;
    }
    case PROTO_EXTRA: {
      foreach (const WbProject *project, *WbProject::extraProjects()) {
        QDirIterator it(project->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext())
          protos << it.next();
      }
      break;
    }
    case PROTO_WEBOTS: {
      QMapIterator<QString, WbProtoInfo *> it(mWebotsProtoList);
      while (it.hasNext())
        protos << it.next().value()->url();
      break;
    }
    default:
      WbLog::error(tr("Cannot list protos, unknown category '%1'.").arg(category));
  }

  return protos;
}

const QMap<QString, WbProtoInfo *> &WbProtoManager::protoInfoMap(int category) const {
  static QMap<QString, WbProtoInfo *> empty;

  switch (category) {
    case PROTO_WORLD:
      return mWorldFileProtoList;
    case PROTO_PROJECT:
      return mProjectProtoList;
    case PROTO_EXTRA:
      return mExtraProtoList;
    case PROTO_WEBOTS:
      return mWebotsProtoList;
    default:
      WbLog::error(tr("Cannot retrieve proto info list, unknown category '%1'.").arg(category));
      return empty;
  }
}

const WbProtoInfo *WbProtoManager::protoInfo(const QString &protoName, int category) {
  const QMap<QString, WbProtoInfo *> &map = protoInfoMap(category);
  if (!map.contains(protoName)) {
    WbLog::error(tr("PROTO '%1' does not belong to category '%2'.").arg(protoName).arg(category));
    return NULL;
  }

  return map.value(protoName);
}

bool WbProtoManager::isProtoInCategory(const QString &protoName, int category) const {
  // note: this function should only be called if the category is known to be up to date, otherwise the update will loop
  // forever
  switch (category) {
    case PROTO_WORLD:
      return mWorldFileProtoList.contains(protoName);
    case PROTO_PROJECT:
      return mProjectProtoList.contains(protoName);
    case PROTO_EXTRA:
      return mExtraProtoList.contains(protoName);
    case PROTO_WEBOTS:
      return mWebotsProtoList.contains(protoName);
    default:
      WbLog::error(tr("Cannot check if '%1' exists because category '%2' is unknown.").arg(protoName).arg(category));
  }

  return false;
}

QString WbProtoManager::protoUrl(const QString &protoName, int category) const {
  const QMap<QString, WbProtoInfo *> &map = protoInfoMap(category);
  if (map.contains(protoName))
    return map.value(protoName)->url();

  return QString();
}

WbProtoInfo *WbProtoManager::generateInfoFromProtoFile(const QString &protoFileName) {
  assert(QFileInfo(protoFileName).exists());
  WbTokenizer tokenizer;
  const int errors = tokenizer.tokenize(protoFileName);
  if (errors > 0)
    return NULL;  // invalid PROTO file

  WbParser parser(&tokenizer);
  if (!parser.parseProtoInterface(mCurrentWorld))
    return NULL;  // invalid PROTO file

  const QString url = WbFileUtil::isLocatedInDirectory(protoFileName, WbStandardPaths::cachedAssetsPath()) ?
                        WbNetwork::instance()->getUrlFromEphemeralCache(protoFileName) :
                        protoFileName;

  tokenizer.rewind();
  WbProtoModel *protoModel = NULL;
  const bool previousInstantiateMode = WbNode::instantiateMode();
  WbNode *previousParent = WbNode::globalParentNode();
  try {
    WbNode::setGlobalParentNode(NULL);
    WbNode::setInstantiateMode(false);
    protoModel = new WbProtoModel(&tokenizer, mCurrentWorld, url);
    WbNode::setInstantiateMode(previousInstantiateMode);
    WbNode::setGlobalParentNode(previousParent);
  } catch (...) {
    WbNode::setInstantiateMode(previousInstantiateMode);
    WbNode::setGlobalParentNode(previousParent);
    return NULL;
  }

  tokenizer.rewind();

  bool needsRobotAncestor = false;
  // establish if it requires a Robot ancestor by checking if it contains devices
  while (tokenizer.hasMoreTokens()) {
    WbToken *token = tokenizer.nextToken();
    if (token->isIdentifier() && mNeedsRobotAncestorCallback(token->word())) {
      needsRobotAncestor = true;
      break;
    }
  }

  // generate field string (needed by PROTO wizard)
  QStringList parameters;
  foreach (const WbFieldModel *model, protoModel->fieldModels()) {
    const WbValue *defaultValue = model->defaultValue();
    QString vrmlDefaultValue;

    if (defaultValue->type() == WB_SF_NODE) {
      const WbSFNode *sfn = dynamic_cast<const WbSFNode *>(defaultValue);
      if (sfn->value()) {
        QString nodeContent = WbVrmlNodeUtilities::exportNodeToString(sfn->value());
        vrmlDefaultValue = nodeContent.replace(QRegularExpression("[\\s\\n]+"), " ");
      }
    } else
      vrmlDefaultValue = defaultValue->toString();

    const WbMultipleValue *mv = dynamic_cast<const WbMultipleValue *>(defaultValue);
    if (defaultValue->type() == WB_MF_NODE && mv) {
      vrmlDefaultValue = "[ ";
      for (int j = 0; j < mv->size(); ++j)
        vrmlDefaultValue += mv->itemToString(j) + "{} ";
      vrmlDefaultValue += "]";
    }

    QString field;
    field += model->isVrml() ? "vrmlField " : "field ";
    field += defaultValue->vrmlTypeName() + " ";
    field += model->name() + " ";
    field += vrmlDefaultValue;
    parameters << field;
  }

  WbProtoInfo *info = new WbProtoInfo(url, protoModel->baseType(), protoModel->license(), protoModel->licenseUrl(),
                                      protoModel->documentationUrl(), protoModel->info(), protoModel->slotType(),
                                      protoModel->tags(), parameters, needsRobotAncestor);

  protoModel->destroy();
  return info;
}

QString WbProtoManager::declareExternProto(const QString &protoName, const QString &protoPath, bool importable,
                                           bool forceUpdate) {
  QString previousUrl;
  const QString expandedProtoPath(WbUrl::resolveUrl(protoPath));
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName) {
      mExternProto[i]->setImportable(mExternProto[i]->isImportable() || importable);
      if (mExternProto[i]->url() != expandedProtoPath) {
        previousUrl = mExternProto[i]->url();
        if (forceUpdate)
          mExternProto[i]->setUrl(expandedProtoPath);
      }
      return previousUrl;
    }
  }

  mExternProto.push_back(new WbExternProto(protoName, expandedProtoPath, importable));
  return previousUrl;
}

void WbProtoManager::purgeUnusedExternProtoDeclarations(const QSet<QString> &protoNamesInUse) {
  for (int i = mExternProto.size() - 1; i >= 0; --i) {
    if (!protoNamesInUse.contains(mExternProto[i]->name()) && !mExternProto[i]->isImportable()) {
      // delete non-importable nodes that have no remaining visible instances
      delete mExternProto[i];
      mExternProto.remove(i);
    }
  }
}

QString WbProtoManager::externProtoUrl(const WbNode *node, bool formatted) const {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == node->modelName()) {
      if (formatted)
        return formatExternProtoPath(mExternProto[i]->url());
      return mExternProto[i]->url();
    }
  }

  // PROTO might be declared in PROTO file instead of world file
  // for example for default PROTO parameter nodes
  if (node->proto()) {
    if (formatted)
      return formatExternProtoPath(node->proto()->url());
    return node->proto()->url();
  }

  assert(false);
  return QString();
}

void WbProtoManager::saveToExternProtoClipboardBuffer(const QString &url) {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->url() == url) {
      mExternProtoClipboardBuffer << new WbExternProto(*mExternProto[i]);
      return;
    }
  }
}

void WbProtoManager::saveToExternProtoClipboardBuffer(const QList<const WbNode *> &nodes) {
  foreach (const WbNode *node, nodes) {
    if (!node->proto())
      continue;

    saveToExternProtoClipboardBuffer(node->proto()->url());
  }
}

void WbProtoManager::clearExternProtoClipboardBuffer() {
  qDeleteAll(mExternProtoClipboardBuffer);
  mExternProtoClipboardBuffer.clear();
}

QList<QString> WbProtoManager::externProtoClipboardBufferUrls() const {
  QList<QString> list;
  foreach (WbExternProto *proto, mExternProtoClipboardBuffer)
    list << proto->url();
  return list;
}

void WbProtoManager::resetExternProtoClipboardBuffer(const QList<QString> &bufferUrls) {
  clearExternProtoClipboardBuffer();
  foreach (QString url, bufferUrls)
    saveToExternProtoClipboardBuffer(url);
}

void WbProtoManager::removeImportableExternProto(const QString &protoName, WbNode *root) {
  for (int i = mExternProto.size() - 1; i >= 0; --i) {
    if (mExternProto[i]->name() == protoName) {
      assert(mExternProto[i]->isImportable());
      // only IMPORTABLE nodes should be removed using this function, instantiated nodes are removed when deleting the node
      mExternProto[i]->setImportable(false);
      if (!WbVrmlNodeUtilities::existsVisibleProtoNodeNamed(protoName, root)) {
        delete mExternProto[i];
        mExternProto.remove(i);
      }
      return;  // we can stop since the list is supposed to contain unique elements, and a match was found
    }
  }
}

void WbProtoManager::updateExternProto(const QString &protoName, const QString &url) {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName) {
      mExternProto[i]->setUrl(WbUrl::resolveUrl(url));
      // loaded model still refers to previous file, it will be updated on world reload
      return;  // we can stop since the list is supposed to contain unique elements, and a match was found
    }
  }

  assert(false);  // should not be requesting to change something that doesn't exist
}

QString WbProtoManager::formatExternProtoPath(const QString &url) const {
  QString path = url;
  if (WbFileUtil::isLocatedInInstallationDirectory(path, true))
    path.replace(WbStandardPaths::webotsHomePath(), "webots://");
  if (path.startsWith(WbProject::current()->protosPath()))
    path = QDir(QFileInfo(mCurrentWorld).absolutePath()).relativeFilePath(path);

  return path;
}

bool WbProtoManager::isImportableExternProtoDeclared(const QString &protoName) {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName && mExternProto[i]->isImportable())
      return true;
  }

  return false;
}

void WbProtoManager::cleanup() {
  qDeleteAll(mWorldFileProtoList);
  qDeleteAll(mProjectProtoList);
  qDeleteAll(mExtraProtoList);
  qDeleteAll(mExternProto);

  mUniqueErrorMessages.clear();
  mWorldFileProtoList.clear();
  mProjectProtoList.clear();
  mExtraProtoList.clear();
  mExternProto.clear();

  mPreviousSessionProto = mSessionProto;
  mSessionProto.clear();
}

QString WbProtoManager::injectDeclarationByBackwardsCompatibility(const QString &modelName) {
  // check if it's the current project
  const QStringList projectProto = listProtoInCategory(PROTO_PROJECT);
  foreach (const QString &proto, projectProto) {
    if (proto.contains(modelName + ".proto", Qt::CaseInsensitive))
      return QFileInfo(proto).absoluteFilePath();
  }
  // check if it's in the EXTRA projects
  const QStringList extraProto = listProtoInCategory(PROTO_EXTRA);
  foreach (const QString &proto, extraProto) {
    if (proto.contains(modelName + ".proto", Qt::CaseInsensitive))
      return QFileInfo(proto).absoluteFilePath();
  }
  // check among the  official ones
  if (isProtoInCategory(modelName, PROTO_WEBOTS)) {
    QString url = mWebotsProtoList.value(modelName)->url();
    if (WbUrl::isWeb(url)) {
      if (WbNetwork::instance()->isCachedWithMapUpdate(url))
        return url;
    }

    if (WbUrl::isLocalUrl(url)) {
      url = QDir::cleanPath(url.replace("webots://", WbStandardPaths::webotsHomePath()));
      if (QFileInfo(url).exists())
        return url;
    }
  }

  return QString();
}

void WbProtoManager::displayMissingDeclarations(const QString &message) {
  if (!mUniqueErrorMessages.contains(message)) {
    mUniqueErrorMessages << message;
    WbLog::error(message);
  }
}

WbVersion WbProtoManager::checkProtoVersion(const QString &protoUrl, bool *foundProtoVersion) {
  QFile protoFile(protoUrl);
  WbVersion protoVersion;
  if (protoFile.open(QIODevice::ReadOnly)) {
    const QByteArray &contents = protoFile.readAll();
    *foundProtoVersion = protoVersion.fromString(contents, "^VRML_SIM", " utf8$");
  }
  return protoVersion;
}
