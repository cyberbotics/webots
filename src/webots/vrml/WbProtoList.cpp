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

#include "WbProtoList.hpp"

#include "WbApplication.hpp"
#include "WbDownloader.hpp"
#include "WbLog.hpp"
#include "WbNetwork.hpp"
#include "WbNode.hpp"
#include "WbNodeUtilities.hpp"
#include "WbParser.hpp"
#include "WbPreferences.hpp"
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

static WbProtoList *gInstance = NULL;

WbProtoList *WbProtoList::instance() {
  if (!gInstance)
    gInstance = new WbProtoList();
  return gInstance;
}

WbProtoList::WbProtoList() {
  mTreeRoot = NULL;
  generateWebotsProtoList();
}

// we do not delete the PROTO models here: each PROTO model is automatically deleted when its last PROTO instance is deleted
WbProtoList::~WbProtoList() {
  // TODO: test proto insertion from supervisor

  if (gInstance == this)
    gInstance = NULL;

  foreach (WbProtoModel *model, mModels)
    model->unref();
}

void WbProtoList::findProtosRecursively(const QString &dirPath, QFileInfoList &protoList, bool inProtos) {
  QDir dir(dirPath);
  if (!dir.exists() || !dir.isReadable())
    // no PROTO nodes
    return;

  // search in folder
  if (inProtos) {
    QStringList filter("*.proto");
    protoList.append(dir.entryInfoList(filter, QDir::Files, QDir::Name));
  }
  // search in subfolders
  QFileInfoList subfolderInfoList = dir.entryInfoList(QDir::AllDirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

  if (!inProtos) {
    // try to identify a project root folder
    foreach (QFileInfo subfolder, subfolderInfoList) {
      const QString &fileName = subfolder.fileName();
      if (fileName == "controllers" || fileName == "worlds" || fileName == "protos" || fileName == "plugins") {
        const QString protosPath = dirPath + "/protos";
        if (QFile::exists(protosPath))
          findProtosRecursively(protosPath, protoList, true);
        return;
      }
    }
  }
  foreach (QFileInfo subfolder, subfolderInfoList) {
    if (inProtos &&
        (subfolder.fileName() == "textures" || subfolder.fileName() == "icons" || subfolder.fileName() == "meshes")) {
      // skip any textures or icons subfolder inside a protos folder
      continue;
    }
    findProtosRecursively(subfolder.absoluteFilePath(), protoList, inProtos);
  }
}

WbProtoModel *WbProtoList::readModel(const QString &fileName, const QString &worldPath, const QString &externUrl,
                                     QStringList baseTypeList) const {
  WbTokenizer tokenizer;
  int errors = tokenizer.tokenize(fileName);
  if (errors > 0)
    return NULL;

  // TODO: should be moved elsewhere (WbParser), as this point might be reached while parsing a world too
  WbParser parser(&tokenizer);

  while (tokenizer.peekWord() == "EXTERNPROTO")  // consume all EXTERNPROTO tokens, if any
    parser.skipExternProto();

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

void WbProtoList::readModel(WbTokenizer *tokenizer, const QString &worldPath) {  // TODO: this constructor, still needed?
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

void WbProtoList::printCurrentWorldProtoList() {
  QMapIterator<QString, QPair<QString, bool>> it(mCurrentWorldProto);
  printf("-- mCurrentWorldProto ------\n");
  while (it.hasNext()) {
    it.next();
    QPair<QString, bool> value = it.value();
    printf("%35s -> [%d] %s\n", it.key().toUtf8().constData(), WbNetwork::instance()->isCached(value.first),
           value.first.toUtf8().constData());
  }
  printf("----------------\n");
}

WbProtoModel *WbProtoList::customFindModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList) {
  // printf("WbProtoList::customFindModel\n");
  // return NULL;

  foreach (WbProtoModel *model, mModels) {  // TODO: ensure mModels is cleared between loads
    if (model->name() == modelName) {
      // printf("No need to search for model %s, it's loaded already.\n", modelName.toUtf8().constData());
      return model;
    }
  }

  if (mCurrentWorldProto.contains(modelName)) {
    QString url =
      WbUrl::computePath(NULL, "EXTERNPROTO", mCurrentWorldProto.value(modelName).first, false);  // TODO: change this
    if (WbUrl::isWeb(url)) {
      // printf(">>>%s\n", url.toUtf8().constData());
      assert(WbNetwork::instance()->isCached(url));
      url = WbNetwork::instance()->get(url);
    }
    printf("%35s is a PROJECT proto, url is: %s\n", modelName.toUtf8().constData(), url.toUtf8().constData());
    WbProtoModel *model = readModel(url, worldPath, mCurrentWorldProto.value(modelName).first, baseTypeList);
    if (model == NULL)  // can occur if the PROTO contains errors
      return NULL;
    mModels << model;
    model->ref();
    return model;
  } else if (mWebotsProtoList.contains(modelName)) {  // TODO: add version check as well?
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
  } else {
    if (!modelName.isEmpty())
      printf("proto %s not found in mCurrentWorldProto ?\n", modelName.toUtf8().constData());
  }
  return NULL;
}

/*
WbProtoModel *WbProtoList::findModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList) {
  // see if model is already loaded
  foreach (WbProtoModel *model, mModels)
    if (model->name() == modelName)
      return model;

  // TODO: remove this function?
  QFileInfoList tmpProto;  // protos in Webots temporary folder (i.e added by EXTERNPROTO reference)
  foreach (const QString &path, WbStandardPaths::webotsTmpProtoPath())
    findProtosRecursively(path, tmpProto);  // TODO: works because folder in tmp is called "protos". No need to have list of
                                            // searchable paths for each primary proto if this is good enough
  printf("> done searching\n");

  foreach (const QFileInfo &fi, tmpProto) {
    if (fi.baseName() == modelName) {
      WbProtoModel *model = readModel(fi.absoluteFilePath(), worldPath, baseTypeList);
      if (model == NULL)  // can occur if the PROTO contains errors
        return NULL;
      mModels << model;
      model->ref();
      return model;
    }
  }

  return NULL;  // not found
}
*/

QString WbProtoList::findModelPath(const QString &modelName) const {
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

void WbProtoList::retrieveExternProto(const QString &filename, bool reloading, const QStringList &unreferencedProtos) {
  // clear current project related variables
  mCurrentWorld = filename;
  mReloading = reloading;
  mCurrentWorldProto.clear();
  delete mTreeRoot;
  foreach (WbProtoModel *model, mModels) {
    if (mCurrentWorldProto.contains(model->name()))
      model->unref();
  }

  // populate the tree with urls expressed by EXTERNPROTO
  QFile rootFile(filename);
  if (rootFile.open(QIODevice::ReadOnly)) {  // TODO: isn't readability checked in prototreeitem?
    QFile rootFile(filename);
    mTreeRoot = new WbProtoTreeItem(filename, NULL, NULL);  //
    connect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoList::tryWorldLoad);
  } else {
    WbLog::error(tr("File '%1' is not readable.").arg(filename));
    return;
  }

  // populate the tree with urls not referenced by EXTERNPROTO (worlds prior to R2022b)
  foreach (const QString proto, unreferencedProtos) {
    if (isWebotsProto(proto))
      mTreeRoot->insert(WbProtoList::instance()->getWebotsProtoUrl(proto));
    else
      WbLog::error(tr("PROTO '%1' is not a known Webots PROTO. The backwards compatibility mechanism may fail.").arg(proto));
  }

  // status pre-firing
  mTreeRoot->print();
  // root node is now fully populated, trigger download
  printf("starting download\n");
  mTreeRoot->downloadAssets();
}

void WbProtoList::retrieveExternProto(const QString &filename) {
  printf("REQUESTING PROTO DOWNLOAD FOR: %s\n", filename.toUtf8().constData());
  // populate the tree with urls expressed by EXTERNPROTO
  delete mTreeRoot;
  mTreeRoot = new WbProtoTreeItem(filename, NULL, NULL);
  connect(mTreeRoot, &WbProtoTreeItem::finished, this, &WbProtoList::singleProtoRetrievalCompleted);
  // trigger download
  mTreeRoot->downloadAssets();
}

void WbProtoList::singleProtoRetrievalCompleted() {
  printf("PROTO DOWNLOAD ENDED, resulting hierarchy:\n");
  // status pre-firing
  printf("--- hierarchy ----\n");
  mTreeRoot->print();
  printf("------------------\n");
  mTreeRoot->generateProtoMap(mCurrentWorldProto);
  emit retrievalCompleted();
}

void WbProtoList::tryWorldLoad() {
  printf("RETRY WORLD LOAD\n");

  if (mTreeRoot && !mTreeRoot->error().isEmpty())
    WbLog::error(mTreeRoot->error());

  // note: although it might have failed, generate the map for the nodes that didn't so that they can be loaded
  mTreeRoot->generateProtoMap(mCurrentWorldProto);  // generate mCurrentWorldProto

  // cleanup and load world at last
  delete mTreeRoot;
  mTreeRoot = NULL;
  WbApplication::instance()->loadWorld(mCurrentWorld, mReloading, true);  // load the world again
  // WbApplication::instance()->cancelWorldLoading(true, true);
}

void WbProtoList::generateWebotsProtoList() {
  if (!mWebotsProtoList.empty())
    return;  // webots proto list already generated

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
          // printf("inserting: [%s][%s][%s][%s][%s][%s][%s]\n", name.toUtf8().constData(), url.toUtf8().constData(),
          //       baseType.toUtf8().constData(), license.toUtf8().constData(), licenseUrl.toUtf8().constData(),
          //       description.toUtf8().constData(), tags.join(",").toUtf8().constData());
          description = description.replace("\\n", "\n");
          WbProtoInfo *info = new WbProtoInfo(url, baseType, license, licenseUrl, documentationUrl, description, slotType, tags,
                                              needsRobotAncestor);
          mWebotsProtoList.insert(name, info);
        } else
          reader.raiseError(tr("Expected 'proto' element."));
      }
    } else
      reader.raiseError(tr("Expected 'proto-list' element."));
  } else
    reader.raiseError(tr("The format of 'proto-list.xml' is invalid."));

  // printf("-- known proto %lld --\n", mWebotsProtoList.size());
  // QMapIterator<QString, WbProtoInfo> it(mWebotsProtoList);
  // while (it.hasNext()) {
  //  it.next();
  //  WbProtoInfo info = it.value();
  //  printf("  %35s %s\n", it.key().toUtf8().constData(), info.url().toUtf8().constData());
  //}
  // printf("-- end mWebotsProtoList --\n");
}

void WbProtoList::generateWorldFileProtoList() {
  qDeleteAll(mWorldFileProtoList);
  mWorldFileProtoList.clear();

  QMapIterator<QString, QPair<QString, bool>> it(mCurrentWorldProto);
  while (it.hasNext()) {
    it.next();
    QPair<QString, int> item = it.value();  // item.second == flag that indicates if it's a root proto in a world file
    if (!item.second)
      continue;

    QString protoPath = WbUrl::generateExternProtoPath(it.value().first, mCurrentWorld);
    if (WbUrl::isWeb(protoPath) && WbNetwork::instance()->isCached(protoPath))
      protoPath = WbNetwork::instance()->get(protoPath);

    const QString protoName = QUrl(it.value().first).fileName().replace(".proto", "");
    WbProtoInfo *info = generateInfoFromProtoFile(protoPath);
    if (info && !mWorldFileProtoList.contains(protoName))
      mWorldFileProtoList.insert(protoName, info);
  }
}

void WbProtoList::generateProjectProtoList() {
  qDeleteAll(mProjectProtoList);
  mProjectProtoList.clear();

  // find all proto and instantiate the nodes to build WbProtoInfo
  QDirIterator it(WbProject::current()->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    const QString path = it.next();
    const QString protoName = QFileInfo(path).baseName();
    WbProtoInfo *info = generateInfoFromProtoFile(path);
    if (info && !mProjectProtoList.contains(protoName))
      mProjectProtoList.insert(protoName, info);
  }
}

void WbProtoList::generateExtraProtoList() {
  qDeleteAll(mExtraProtoList);
  mExtraProtoList.clear();

  // find all proto and instantiate the nodes to build WbProtoInfo
  foreach (const WbProject *project, *WbProject::extraProjects()) {
    QDirIterator it(project->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
      const QString path = it.next();
      const QString protoName = QFileInfo(path).baseName();
      WbProtoInfo *info = generateInfoFromProtoFile(path);
      if (info && !mExtraProtoList.contains(protoName))
        mExtraProtoList.insert(protoName, info);
    }
  }
}

bool WbProtoList::isWebotsProto(const QString &protoName) {
  assert(mWebotsProtoList.size() > 0);
  return mWebotsProtoList.contains(protoName);
}

const QString WbProtoList::getWebotsProtoUrl(const QString &protoName) {
  assert(mWebotsProtoList.size() > 0);
  return mWebotsProtoList.value(protoName)->url();
}

WbProtoInfo *WbProtoList::generateInfoFromProtoFile(const QString &protoFileName) {
  assert(QFileInfo(protoFileName).exists());
  WbTokenizer tokenizer;
  int errors = tokenizer.tokenize(protoFileName);
  if (errors > 0)
    return NULL;  // invalid PROTO file

  WbParser parser(&tokenizer);
  if (!parser.parseProtoInterface(mCurrentWorld)) {  // TODO: ensure mCurrentWorld is always correct or ""
    return NULL;                                     // invalid PROTO file
  }

  tokenizer.rewind();
  WbProtoModel *protoModel = NULL;
  bool prevInstantiateMode = WbNode::instantiateMode();
  WbNode *previousParent = WbNode::globalParentNode();
  try {
    WbNode::setGlobalParentNode(NULL);
    WbNode::setInstantiateMode(false);
    protoModel = new WbProtoModel(&tokenizer, mCurrentWorld, protoFileName);
    WbNode::setInstantiateMode(prevInstantiateMode);
    WbNode::setGlobalParentNode(previousParent);
  } catch (...) {
    WbNode::setInstantiateMode(prevInstantiateMode);
    WbNode::setGlobalParentNode(previousParent);
    return NULL;
  }

  tokenizer.rewind();
  bool needsRobotAncestor = false;
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

void WbProtoList::exportProto(const QString &proto) {
  printf("EXPORTPROTO CALLED WITH %s\n", proto.toUtf8().constData());
  QString path = proto;
  if (WbUrl::isWeb(proto) && WbNetwork::instance()->isCached(path))
    path = WbNetwork::instance()->get(path);
  else {
    delete mTreeRoot;
    mTreeRoot = new WbProtoTreeItem(proto, NULL, NULL);
    mTreeRoot->setRecursion(false);  // stop download at the first level
    connect(mTreeRoot, &WbProtoTreeItem::downloadComplete, this, &WbProtoList::exportProto);
    mTreeRoot->downloadAssets();  // trigger download
    return;
  }

  path = WbUrl::generateExternProtoPath(path, "");

  QString destination = WbProject::current()->protosPath();
  if (!QDir(destination).exists())
    QDir().mkdir(destination);

  QFile input(path);
  if (input.open(QIODevice::ReadOnly)) {
    QString contents = QString(input.readAll());
    input.close();

    QRegularExpression re("EXTERNPROTO\\s+\n*\"(webots://).*\\.proto\"");  // TODO: test it more
    contents.replace(re, "TEST://");
    QString filename = destination + "tmp.proto";
    QFile output(filename);  // TODO: to fix
    if (output.open(QIODevice::WriteOnly)) {
      output.write(contents.toUtf8());
      output.close();
    } else
      WbLog::error(tr("Impossible to export PROTO to '%1' as this location cannot be written to.").arg(filename));
  } else
    WbLog::error(tr("Impossible to export PROTO '%1' as the source file cannot be read.").arg(proto));

  printf("PROTO %s WILL BE EXPORTED TO %s\n", path.toUtf8().constData(), destination.toUtf8().constData());
}

QStringList WbProtoList::nameList(int category) {
  QStringList names;

  switch (category) {
    case PROTO_WORLD: {
      QMapIterator<QString, QPair<QString, bool>> it(mCurrentWorldProto);
      while (it.hasNext()) {
        QPair<QString, int> item = it.next().value();
        if (!item.second)  // item.second == flag that indicates if it's a root proto in a world file
          continue;

        names << it.key();
      }
      break;
    }
    case PROTO_PROJECT: {
      QDirIterator it(WbProject::current()->protosPath(), QStringList() << "*.proto", QDir::Files,
                      QDirIterator::Subdirectories);
      while (it.hasNext())
        names << QFileInfo(it.next()).baseName();
      break;
    }
    case PROTO_EXTRA: {
      foreach (const WbProject *project, *WbProject::extraProjects()) {
        QDirIterator it(project->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext())
          names << QFileInfo(it.next()).baseName();
      }
      break;
    }
    case PROTO_WEBOTS: {
      generateWebotsProtoList();  // generate list if not done yet
      QMapIterator<QString, WbProtoInfo *> it(mWebotsProtoList);
      while (it.hasNext())
        names << it.next().key();
      break;
    }
    default:
      WbLog::error(tr("'%1' is an unknown PROTO list category.").arg(category));
  }

  return names;
}

void WbProtoList::declareAsExtern(const QString &protoName, const QString &protoUrl) {

}