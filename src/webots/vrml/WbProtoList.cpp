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
  setupKnownProtoList();
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
  QMapIterator<QString, QString> it(mCurrentWorldProto);
  printf("-- mCurrentWorldProto ------\n");
  while (it.hasNext()) {
    it.next();
    printf("%35s -> [%d] %s\n", it.key().toUtf8().constData(), WbNetwork::instance()->isCached(it.value()),
           it.value().toUtf8().constData());
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
    QString url = WbUrl::computePath(NULL, "EXTERNPROTO", mCurrentWorldProto.value(modelName), false);  // TODO: change this
    if (WbUrl::isWeb(url)) {
      // printf(">>>%s\n", url.toUtf8().constData());
      assert(WbNetwork::instance()->isCached(url));
      url = WbNetwork::instance()->get(url);
    }
    printf("%35s is a PROJECT proto, url is: %s\n", modelName.toUtf8().constData(), url.toUtf8().constData());
    WbProtoModel *model = readModel(url, worldPath, mCurrentWorldProto.value(modelName), baseTypeList);
    if (model == NULL)  // can occur if the PROTO contains errors
      return NULL;
    mModels << model;
    model->ref();
    return model;
  } else if (mOfficialProtoList.contains(modelName)) {  // TODO: add version check as well?
    QString url = mOfficialProtoList.value(modelName)->url();
    if (WbNetwork::instance()->isCached(url)) {
      url = WbNetwork::instance()->get(url);
      printf("%35s is an OFFICIAL proto, url is: %s\n", modelName.toUtf8().constData(), url.toUtf8().constData());
      WbProtoModel *model = readModel(QFileInfo(url).absoluteFilePath(), worldPath, url, baseTypeList);
      if (model == NULL)  // can occur if the PROTO contains errors
        return NULL;
      mModels << model;
      model->ref();
      return model;
    }
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

QStringList WbProtoList::fileList() {
  QStringList list;
  foreach (WbProtoModel *model, gInstance->mModels)
    list << model->fileName();
  return list;
}

void WbProtoList::setupKnownProtoList() {
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
          WbProtoInfo *info = new WbProtoInfo(url, baseType, license, licenseUrl, documentationUrl, description, slotType, tags,
                                              needsRobotAncestor);
          mOfficialProtoList.insert(name, info);
        } else
          reader.raiseError(tr("Expected 'proto' element."));
      }
    } else
      reader.raiseError(tr("Expected 'proto-list' element."));
  } else
    reader.raiseError(tr("The format of 'proto-list.xml' is invalid."));

  // printf("-- known proto %lld --\n", mProtoList.size());
  // QMapIterator<QString, WbProtoInfo> it(mProtoList);
  // while (it.hasNext()) {
  //  it.next();
  //  WbProtoInfo info = it.value();
  //  printf("  %35s %s\n", it.key().toUtf8().constData(), info.url().toUtf8().constData());
  //}
  // printf("-- end known proto --\n");
}

bool WbProtoList::isOfficialProto(const QString &protoName) {
  assert(mOfficialProtoList.size() > 0);
  return mOfficialProtoList.contains(protoName);
}

const QString WbProtoList::getOfficialProtoUrl(const QString &protoName) {
  assert(mOfficialProtoList.size() > 0);
  return mOfficialProtoList.value(protoName)->url();
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
  if (rootFile.open(QIODevice::ReadOnly)) {
    QFile rootFile(filename);
    mTreeRoot = new WbProtoTreeItem(filename, NULL);  //
    connect(mTreeRoot, &WbProtoTreeItem::readyToLoad, this, &WbProtoList::tryWorldLoad);
  } else {
    WbLog::error(tr("File '%1' is not readable.").arg(filename));
    return;
  }

  // populate the tree with urls not referenced by EXTERNPROTO (worlds prior to R2022b)
  foreach (const QString proto, unreferencedProtos) {
    if (isOfficialProto(proto))
      mTreeRoot->insert(WbProtoList::instance()->getOfficialProtoUrl(proto));
    else
      WbLog::error(tr("PROTO '%1' is not a known official PROTO. The backwards compatibility mechanism may fail.").arg(proto));
  }

  // status pre-firing
  mTreeRoot->print();
  // root node is now fully populated, trigger download
  mTreeRoot->downloadAssets();
}

void WbProtoList::tryWorldLoad() {
  printf("RETRY WORLD LOAD\n");
  // generate mCurrentWorldProto
  mTreeRoot->generateProtoMap(mCurrentWorldProto);
  // cleanup and load world at last
  delete mTreeRoot;
  mTreeRoot = NULL;
  WbApplication::instance()->loadWorld(mCurrentWorld, mReloading, true);  // load the world again
}

WbProtoInfo *WbProtoList::generateInfoFromProtoFile(const QString &protoFileName) {
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

  // compute and store proto info
  WbProtoInfo *info = new WbProtoInfo(protoFileName, protoModel->baseType(), protoModel->license(), protoModel->licenseUrl(),
                                      protoModel->documentationUrl(), protoModel->info(), protoModel->slotType(),
                                      protoModel->tags(), needsRobotAncestor);

  protoModel->destroy();
  return info;
}