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

#include "WbProtoManager.hpp"

#include "WbApplication.hpp"
#include "WbDownloader.hpp"
#include "WbLog.hpp"
#include "WbNetwork.hpp"
#include "WbNode.hpp"
#include "WbNodeUtilities.hpp"
#include "WbParser.hpp"
#include "WbProject.hpp"
#include "WbProtoModel.hpp"
#include "WbProtoTreeItem.hpp"
#include "WbStandardPaths.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"
#include "WbUrl.hpp"

#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QRegularExpression>
#include <QtCore/QXmlStreamReader>

static WbProtoManager *gInstance = NULL;

WbProtoManager *WbProtoManager::instance() {
  if (!gInstance)
    gInstance = new WbProtoManager();
  return gInstance;
}

WbProtoManager::WbProtoManager() {
  mTreeRoot = NULL;
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

WbProtoModel *WbProtoManager::readModel(const QString &fileName, const QString &worldPath, const QString &externUrl,
                                        QStringList baseTypeList) const {
  WbTokenizer tokenizer;
  int errors = tokenizer.tokenize(fileName);
  if (errors > 0)
    return NULL;

  WbParser parser(&tokenizer);
  if (!parser.parseProtoInterface(worldPath))
    return NULL;

  tokenizer.rewind();
  while (tokenizer.peekWord() == "EXTERNPROTO")  // consume all EXTERNPROTO tokens, if any
    parser.skipExternProto();

  bool prevInstantiateMode = WbNode::instantiateMode();
  try {
    WbNode::setInstantiateMode(false);
    WbProtoModel *model = new WbProtoModel(&tokenizer, worldPath, fileName, externUrl, baseTypeList);
    WbNode::setInstantiateMode(prevInstantiateMode);
    return model;
  } catch (...) {
    WbNode::setInstantiateMode(prevInstantiateMode);
    return NULL;
  }
}

void WbProtoManager::readModel(WbTokenizer *tokenizer, const QString &worldPath) {
  WbProtoModel *model = NULL;
  bool prevInstantiateMode = WbNode::instantiateMode();
  try {
    WbNode::setInstantiateMode(false);
    model = new WbProtoModel(tokenizer, worldPath);
    WbNode::setInstantiateMode(prevInstantiateMode);
  } catch (...) {
    WbNode::setInstantiateMode(prevInstantiateMode);
    return;
  }
  mModels.prepend(model);
  model->ref();
}

// TODO: remove
void WbProtoManager::printCurrentWorldProtoList() {
  QMapIterator<QString, QString> it(mSessionProto);
  printf("-- mSessionProto ------\n");
  while (it.hasNext()) {
    it.next();
    printf("%35s -> [%d] %s\n", it.key().toUtf8().constData(), WbNetwork::instance()->isCached(it.value()),
           it.value().toUtf8().constData());
  }
  printf("----------------\n");
}

WbProtoModel *WbProtoManager::findModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList) {
  foreach (WbProtoModel *model, mModels) {
    if (model->name() == modelName)
      return model;
  }

  if (mSessionProto.contains(modelName)) {
    // TODO: old
    // QString url = WbUrl::computePath(NULL, "EXTERNPROTO", mSessionProto.value(modelName), false);
    QString url = WbUrl::generateExternProtoPath(mSessionProto.value(modelName));
    printf("%35s is a SESSION proto, url is: %s\n", modelName.toUtf8().constData(), url.toUtf8().constData());
    if (WbUrl::isWeb(url))
      url = WbNetwork::instance()->get(url);

    WbProtoModel *model = readModel(url, worldPath, mSessionProto.value(modelName), baseTypeList);
    if (model == NULL)  // can occur if the PROTO contains errors
      return NULL;
    mModels << model;
    model->ref();
    return model;
  } else if (isWebotsProto(modelName)) {  // TODO: add version check as well?
    QString url = mWebotsProtoList.value(modelName)->url();
    if (WbUrl::isWeb(url) && WbNetwork::instance()->isCached(url))
      url = WbNetwork::instance()->get(url);
    else if (WbUrl::isLocalUrl(url))
      url = QDir::cleanPath(WbStandardPaths::webotsHomePath() + url.mid(9));

    printf("%35s is an OFFICIAL proto, url is: %s\n", modelName.toUtf8().constData(), url.toUtf8().constData());
    WbProtoModel *model = readModel(QFileInfo(url).absoluteFilePath(), worldPath, url, baseTypeList);
    if (model == NULL)  // can occur if the PROTO contains errors
      return NULL;
    mModels << model;
    model->ref();
    return model;
  } else {  // TODO: cleanup
    if (!modelName.isEmpty())
      printf("proto %s not found in mSessionProto ?\n", modelName.toUtf8().constData());
  }

  return NULL;
}

QString WbProtoManager::findModelPath(const QString &modelName) const {
  // TODO: restore this
  /*
  QFileInfoList availableProtoFiles;
  availableProtoFiles << mPrimaryProtoCache << gExtraProtoCache << gProjectsProtoCache << gResourcesProtoCache;

  foreach (const QFileInfo &fi, availableProtoFiles) {
    if (fi.baseName() == modelName)
      return fi.absoluteFilePath();
  }
  */
  return QString();  // not found
}

void WbProtoManager::retrieveExternProto(const QString &filename, bool reloading, const QStringList &unreferencedProtos) {
  // clear current project related variables
  cleanup();
  mCurrentWorld = filename;
  mReloading = reloading;

  // populate the tree with urls expressed by EXTERNPROTO
  mTreeRoot = new WbProtoTreeItem(filename, NULL, NULL, false);  // download is triggered manually
  connect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoManager::loadWorld);

  // populate the tree with urls not referenced by EXTERNPROTO (worlds prior to R2022b)
  foreach (const QString proto, unreferencedProtos) {
    if (isWebotsProto(proto))
      mTreeRoot->insert(protoUrl(PROTO_WEBOTS, proto));
    else
      WbLog::error(tr("PROTO '%1' is not a known Webots PROTO. The backwards compatibility mechanism may fail.").arg(proto));
  }

  // status pre-firing
  printf("---------- mTreeRoot--------\n");
  mTreeRoot->print();
  printf("-------------------------\n");
  // root node is now fully populated, trigger download
  // printf("starting download from %p %lld\n", mTreeRoot, mTreeRoot->children().size());
  mTreeRoot->download();
}

void WbProtoManager::retrieveExternProto(const QString &filename) {
  printf("REQUESTING PROTO DOWNLOAD FOR: %s\n", filename.toUtf8().constData());
  // populate the tree with urls expressed by EXTERNPROTO
  mTreeRoot = new WbProtoTreeItem(filename, NULL, NULL, false);  // download is triggered manually
  connect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoManager::singleProtoRetrievalCompleted);
  // trigger download
  printf("---------- mTreeRoot--------\n");
  mTreeRoot->print();
  printf("-------------------------\n");
  mTreeRoot->download();
}

void WbProtoManager::singleProtoRetrievalCompleted() {
  printf("--- hierarchy ----\n");
  mTreeRoot->print();
  printf("------------------\n");
  mTreeRoot->generateSessionProtoMap(mSessionProto);
  delete mTreeRoot;

  emit retrievalCompleted();
}

void WbProtoManager::loadWorld() {
  printf("TRY WORLD LOAD\n");

  if (mTreeRoot && !mTreeRoot->error().isEmpty()) {
    WbLog::error(mTreeRoot->error());
    printf("Loading empty world!\n");
    delete mTreeRoot;
    mTreeRoot = NULL;
    WbApplication::instance()->cancelWorldLoading(true, true);
    return;
  }

  mTreeRoot->generateSessionProtoMap(mSessionProto);  // generate mSessionProto based on the resulting tree
  // declare all root PROTO (i.e. defined at the world level and inferred by backwards compatibility) to the list of EXTERNPROTO
  foreach (const WbProtoTreeItem *const child, mTreeRoot->children()) {
    QString url = child->url();
    declareExternProto(child->name(), url.replace(WbStandardPaths::webotsHomePath(), "webots://"), false);
  }

  // cleanup and load world at last
  delete mTreeRoot;
  mTreeRoot = NULL;
  WbApplication::instance()->loadWorld(mCurrentWorld, mReloading, true);
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
          QStringList tags;
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
            if (reader.name().toString() == "needs-robot-ancestor") {
              needsRobotAncestor = reader.readElementText() == "true";
              reader.readNext();
            }
          }
          description = description.replace("\\n", "\n");
          WbProtoInfo *const info = new WbProtoInfo(url, baseType, license, licenseUrl, documentationUrl, description, slotType,
                                                    tags, needsRobotAncestor);
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
  printf("generating for category %d\n", category);
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
    it.next().value()->dirty(true);

  // find all proto and instantiate the nodes to build WbProtoInfo (if necessary)
  const QStringList protos = listProtoInDirectory(category);
  QDateTime lastGenerationTime = mProtoInfoGenerationTime.value(category);
  foreach (const QString &protoPath, protos) {
    const QString protoName = QFileInfo(protoPath).baseName();
    printf("checking %s\n", protoName.toUtf8().constData());

    if (!map->contains(protoName) || (QFileInfo(protoPath).lastModified() > lastGenerationTime)) {
      // if it exists but is just out of date, remove previous information
      if (map->contains(protoName)) {
        printf("  existed\n");
        delete map->value(protoName);
        map->remove(protoName);
      }
      printf("  gen new\n");
      // generate new and insert it
      WbProtoInfo *info = generateInfoFromProtoFile(protoPath);
      if (info)
        map->insert(protoName, info);
    } else {
      printf("  no change\n");
      // no change necessary
      map->value(protoName)->dirty(false);
    }
  }

  // delete anything that is still flagged as dirty (it might happen if the PROTO no longer exists in the folders)
  it.toFront();
  while (it.hasNext()) {
    if (it.next().value()->isDirty()) {
      delete map->value(it.key());
      map->remove(it.key());
    }
  }

  mProtoInfoGenerationTime.remove(category);
  mProtoInfoGenerationTime.insert(category, QDateTime::currentDateTime());
}

QStringList WbProtoManager::listProtoInDirectory(int category) {
  QStringList protos;

  switch (category) {
    case PROTO_WORLD: {
      for (int i = 0; i < mExternProto.size(); ++i) {
        QString protoPath = WbUrl::generateExternProtoPath(mExternProto[i]->url());
        // mExternProto contains raw paths, retrieve corresponding disk file
        if (WbUrl::isWeb(protoPath) && WbNetwork::instance()->isCached(protoPath))
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
    default:
      WbLog::error(tr("Cannot list protos, unknown category '%1'.").arg(category));
  }

  return protos;
}

const QMap<QString, WbProtoInfo *> &WbProtoManager::protoInfoMap(int category) {
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

bool WbProtoManager::isWebotsProto(const QString &protoName) {
  assert(mWebotsProtoList.size() > 0);
  return mWebotsProtoList.contains(protoName);
}

const QString &WbProtoManager::protoUrl(int category, const QString &protoName) {
  const QMap<QString, WbProtoInfo *> &map = protoInfoMap(category);
  assert(map.contains(protoName));
  return map.value(protoName)->url();
}

WbProtoInfo *WbProtoManager::generateInfoFromProtoFile(const QString &protoFileName) {
  assert(QFileInfo(protoFileName).exists());
  WbTokenizer tokenizer;
  int errors = tokenizer.tokenize(protoFileName);
  if (errors > 0)
    return NULL;  // invalid PROTO file

  WbParser parser(&tokenizer);
  if (!parser.parseProtoInterface(mCurrentWorld))
    return NULL;  // invalid PROTO file

  tokenizer.rewind();
  WbProtoModel *protoModel = NULL;
  bool previousInstantiateMode = WbNode::instantiateMode();
  WbNode *previousParent = WbNode::globalParentNode();
  try {
    WbNode::setGlobalParentNode(NULL);
    WbNode::setInstantiateMode(false);
    protoModel = new WbProtoModel(&tokenizer, mCurrentWorld, protoFileName);
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
    if (token->isIdentifier() && WbNodeUtilities::isDeviceTypeName(token->word()) && token->word() != "Connector") {
      needsRobotAncestor = true;
      break;
    }
  }

  WbProtoInfo *info = new WbProtoInfo(protoFileName, protoModel->baseType(), protoModel->license(), protoModel->licenseUrl(),
                                      protoModel->documentationUrl(), protoModel->info(), protoModel->slotType(),
                                      protoModel->tags(), needsRobotAncestor);

  protoModel->destroy();
  return info;
}

void WbProtoManager::exportProto(const QString &path, int category) {
  printf("EXPORTPROTO CALLED WITH %s\n", path.toUtf8().constData());

  QString url = path;
  if (WbUrl::isWeb(url)) {
    if (WbNetwork::instance()->isCached(url))
      url = WbNetwork::instance()->get(url);
    else {
      WbLog::error(tr("Cannot export '%1', file not locally available.").arg(url));
      return;
    }
  }

  // if web url, build it from remote url not local file (which is an hash)
  const QString &protoName = WbUrl::isWeb(path) ? QUrl(path).fileName() : QFileInfo(url).fileName();

  QFile input(WbUrl::generateExternProtoPath(url));
  if (input.open(QIODevice::ReadOnly)) {
    QString contents = QString(input.readAll());
    input.close();

    // find all sub-proto references
    QRegularExpression re("EXTERNPROTO\\s+\"([^\\s]+)\"");
    QRegularExpressionMatchIterator it = re.globalMatch(contents);
    QStringList subProto;
    while (it.hasNext()) {
      QRegularExpressionMatch match = it.next();
      if (match.hasMatch())
        subProto << match.captured(1);
    }

    // manufacture url and replace it in the contents
    foreach (const QString &proto, subProto) {
      QString newUrl = WbUrl::generateExternProtoPath(proto, path);  // if web url, build it from remote url not local file
      newUrl.replace(WbStandardPaths::webotsHomePath(), "webots://");
      printf("WAS %s\n IS %s\n", proto.toUtf8().constData(), newUrl.toUtf8().constData());
      contents = contents.replace(proto, newUrl);
    }

    // create destination directory if it does not exist yet
    const QString &destination = WbProject::current()->protosPath();
    if (!QDir(destination).exists())
      QDir().mkdir(destination);

    // save to file
    const QString fileName = destination + protoName;
    QFile output(fileName);
    if (output.open(QIODevice::WriteOnly)) {
      output.write(contents.toUtf8());
      output.close();
      printf("PROTO WILL BE EXPORTED TO %s\n", fileName.toUtf8().constData());
    } else
      WbLog::error(tr("Impossible to export PROTO to '%1' as this location cannot be written to.").arg(fileName));
  } else
    WbLog::error(tr("Impossible to export PROTO '%1' as the source file cannot be read.").arg(protoName));
}

void WbProtoManager::declareExternProto(const QString &protoName, const QString &protoPath, bool ephemeral) {
  printf("DECLARING EXTERN [%d]: '%s' => '%s'\n", ephemeral, protoName.toUtf8().constData(), protoPath.toUtf8().constData());

  // check there are no ambiguities with the existing mExternProto
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName) {
      if (mExternProto[i]->url() != protoPath)
        WbLog::error(tr("'%1' cannot be declared as EXTERNPROTO because it is ambiguous.\nThe previous reference was: "
                        "'%2'\nThe current reference is: '%3'.")
                       .arg(protoName)
                       .arg(mExternProto[i]->url())
                       .arg(protoPath));
      else
        WbLog::warning(tr("'%1' is already declared as EXTERNPROTO.").arg(protoName));

      return;  // exists already
    }
  }

  mExternProto.push_back(new WbExternProtoInfo(protoName, protoPath, ephemeral));
}

void WbProtoManager::removeExternProto(const QString &protoName) {
  printf("REMOVING EXTERN: '%s'\n", protoName.toUtf8().constData());
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName) {
      mExternProto.remove(i);
      return;  // we can stop since the list is supposed to contain unique elements
    }
  }
}

bool WbProtoManager::isDeclaredExternProto(const QString &protoName) {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName)
      return true;
  }

  return false;
}

void WbProtoManager::refreshExternProtoList() {
  // only when the node tree is complete it can be established which among the declared EXTERNPROTO are actually ephemeral
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (!WbNodeUtilities::existsVisibleNodeNamed(mExternProto[i]->name()))
      mExternProto[i]->ephemeral(true);
  }
}

void WbProtoManager::cleanup() {
  qDeleteAll(mExternProto);
  qDeleteAll(mWorldFileProtoList);
  qDeleteAll(mProjectProtoList);
  qDeleteAll(mExtraProtoList);

  mWorldFileProtoList.clear();
  mProjectProtoList.clear();
  mExtraProtoList.clear();
  mSessionProto.clear();
  mExternProto.clear();
}
