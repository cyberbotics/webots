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

#include "WbNode.hpp"
#include "WbParser.hpp"
#include "WbPreferences.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"
#include "WbTokenizer.hpp"
#include "WbVrmlWriter.hpp"

#include <QtCore/QDir>

WbProtoList *gCurrent = NULL;
QFileInfoList WbProtoList::gResourcesProtoCache;
QFileInfoList WbProtoList::gProjectsProtoCache;
QFileInfoList WbProtoList::gExtraProtoCache;

WbProtoList *WbProtoList::current() {
  return gCurrent;
}

WbProtoList::WbProtoList(const QString &primarySearchPath) {
  gCurrent = this;
  mPrimarySearchPath = primarySearchPath;

  static bool firstCall = true;
  if (firstCall) {
    updateResourcesProtoCache();
    updateProjectsProtoCache();
    updateExtraProtoCache();
    firstCall = false;
  }

  updatePrimaryProtoCache();
}

// we do not delete the PROTO models here: each PROTO model is automatically deleted when its last PROTO instance is deleted
WbProtoList::~WbProtoList() {
  if (gCurrent == this)
    gCurrent = NULL;

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

void WbProtoList::updateResourcesProtoCache() {
  gResourcesProtoCache.clear();
  QFileInfoList protosInfo;
  findProtosRecursively(WbStandardPaths::resourcesProjectsPath(), protosInfo);
  gResourcesProtoCache << protosInfo;
}

void WbProtoList::updateProjectsProtoCache() {
  gProjectsProtoCache.clear();
  QFileInfoList protosInfo;
  findProtosRecursively(WbStandardPaths::projectsPath(), protosInfo);
  gProjectsProtoCache << protosInfo;
}

void WbProtoList::updateExtraProtoCache() {
  gExtraProtoCache.clear();
  QFileInfoList protosInfo;
  if (!WbPreferences::instance()->value("General/extraProjectsPath").toString().isEmpty())
    findProtosRecursively(WbPreferences::instance()->value("General/extraProjectsPath").toString(), protosInfo);
  gExtraProtoCache << protosInfo;
}

void WbProtoList::updatePrimaryProtoCache() {
  mPrimaryProtoCache.clear();

  if (mPrimarySearchPath.isEmpty())
    return;

  QFileInfoList protosInfo;
  findProtosRecursively(mPrimarySearchPath, protosInfo, mPrimarySearchPath.endsWith("protos"));
  mPrimaryProtoCache << protosInfo;
}

WbProtoModel *WbProtoList::readModel(const QString &fileName, const QString &worldPath, QStringList baseTypeList) const {
  WbTokenizer tokenizer;
  int errors = tokenizer.tokenize(fileName);
  if (errors > 0)
    return NULL;

  WbParser parser(&tokenizer);
  if (!parser.parseProtoInterface(worldPath))
    return NULL;

  tokenizer.rewind();
  bool prevInstantiateMode = WbNode::instantiateMode();
  try {
    WbNode::setInstantiateMode(false);
    WbProtoModel *model = new WbProtoModel(&tokenizer, worldPath, fileName, baseTypeList);
    WbNode::setInstantiateMode(prevInstantiateMode);
    return model;
  } catch (...) {
    WbNode::setInstantiateMode(prevInstantiateMode);
    return NULL;
  }
}

void WbProtoList::readModel(WbTokenizer *tokenizer, const QString &worldPath) {
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

WbProtoModel *WbProtoList::findModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList) {
  // see if model is already loaded
  foreach (WbProtoModel *model, mModels)
    if (model->name() == modelName)
      return model;

  QFileInfoList availableProtoFiles;
  availableProtoFiles << mPrimaryProtoCache << gExtraProtoCache << gProjectsProtoCache << gResourcesProtoCache;

  foreach (const QFileInfo &fi, availableProtoFiles) {
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

QString WbProtoList::findModelPath(const QString &modelName) const {
  QFileInfoList availableProtoFiles;
  availableProtoFiles << mPrimaryProtoCache << gExtraProtoCache << gProjectsProtoCache << gResourcesProtoCache;

  foreach (const QFileInfo &fi, availableProtoFiles) {
    if (fi.baseName() == modelName)
      return fi.absoluteFilePath();
  }

  return QString();  // not found
}

QStringList WbProtoList::fileList() {
  QStringList list;
  foreach (WbProtoModel *model, gCurrent->mModels)
    list << model->fileName();
  return list;
}

QStringList WbProtoList::fileList(int cache) {
  QStringList list;

  QFileInfoList availableProtoFiles;
  switch (cache) {
    case RESOURCES_PROTO_CACHE:
      availableProtoFiles << gResourcesProtoCache;
      break;
    case PROJECTS_PROTO_CACHE:
      availableProtoFiles << gProjectsProtoCache;
      break;
    case EXTRA_PROTO_CACHE:
      availableProtoFiles << gExtraProtoCache;
      break;
    default:
      return list;
  }

  foreach (const QFileInfo &fi, availableProtoFiles)
    list.append(fi.baseName());

  return list;
}
