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

#include "../app/WbApplication.hpp"
#include "../core/WbApplicationInfo.hpp"
#include "../core/WbNetwork.hpp"
#include "../nodes/utils/WbDownloader.hpp"
#include "../nodes/utils/WbUrl.hpp"
#include "WbLog.hpp"
#include "WbNode.hpp"
#include "WbParser.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"
#include "WbTokenizer.hpp"

#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QRegularExpression>

static WbProtoList *gCurrent = NULL;
QFileInfoList WbProtoList::gResourcesProtoCache;
QFileInfoList WbProtoList::gProjectsProtoCache;
QFileInfoList WbProtoList::gExtraProtoCache;

WbProtoList *WbProtoList::current() {
  if (!gCurrent)
    gCurrent = new WbProtoList();
  return gCurrent;
}

/*
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
*/

WbProtoList::WbProtoList() {
  setupKnownProtoList();
}

// we do not delete the PROTO models here: each PROTO model is automatically deleted when its last PROTO instance is deleted
WbProtoList::~WbProtoList() {
  if (gCurrent == this)
    gCurrent = NULL;

  foreach (WbProtoModel *model, mModels)
    model->unref();

  clearProtoSearchPaths();
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

/*
void WbProtoList::updateResourcesProtoCache() {
  gResourcesProtoCache.clear();
  QFileInfoList protosInfo;
  findProtosRecursively(WbStandardPaths::resourcesProjectsPath(), protosInfo);
  gResourcesProtoCache << protosInfo;
}
*/

void WbProtoList::updateProjectsProtoCache() {
  gProjectsProtoCache.clear();
  QFileInfoList protosInfo;
  findProtosRecursively(WbStandardPaths::projectsPath(), protosInfo);
  gProjectsProtoCache << protosInfo;
}

void WbProtoList::updateExtraProtoCache() {
  gExtraProtoCache.clear();
  QFileInfoList protosInfo;

  foreach (const WbProject *project, *WbProject::extraProjects())
    findProtosRecursively(project->path(), protosInfo);

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

void WbProtoList::printCurrentProjectProtoList() {
  QMapIterator<QString, QString> it(mCurrentProjectProtoList);
  printf("-- mCurrentProjectProtoList ------\n");
  while (it.hasNext()) {
    it.next();
    printf("%s -> %s\n", it.key().toUtf8().constData(), it.value().toUtf8().constData());
  }
  printf("----------------\n");
}

WbProtoModel *WbProtoList::customFindModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList) {
  // printf("WbProtoList::customFindModel\n");
  // return NULL;

  foreach (WbProtoModel *model, mModels)
    if (model->name() == modelName)
      return model;

  if (mCurrentProjectProtoList.contains(modelName)) {
    QString url = WbUrl::computePath(NULL, "EXTERNPROTO", mCurrentProjectProtoList.value(modelName), false);
    if (WbUrl::isWeb(url)) {
      assert(WbNetwork::instance()->isCached(url));
      url = WbNetwork::instance()->get(mCurrentProjectProtoList.value(modelName));
    }
    printf("found %s in mCurrentProjectProtoList, url is: %s\n", modelName.toUtf8().constData(), url.toUtf8().constData());
    WbProtoModel *model = readModel(QFileInfo(url).absoluteFilePath(), worldPath, baseTypeList);
    if (model == NULL)  // can occur if the PROTO contains errors
      return NULL;
    mModels << model;
    model->ref();
    return model;
  } else
    printf("proto %s not found in mCurrentProjectProtoList\n", modelName.toUtf8().constData());

  return NULL;
}

WbProtoModel *WbProtoList::findModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList) {
  // see if model is already loaded
  foreach (WbProtoModel *model, mModels)
    if (model->name() == modelName)
      return model;

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

void WbProtoList::clearProtoSearchPaths(void) {
  printf("> clearing proto search paths\n");
  mProtoSearchPaths.clear();
  // TODO: add current working project path by default (location of world file), others?
}

void WbProtoList::insertProtoSearchPath(const QString &path) {
  QDir dir(path);
  if (dir.exists() && !mProtoSearchPaths.contains(path))
    mProtoSearchPaths << path;

  printf("Searchable paths:\n");
  foreach (const QString &path, mProtoSearchPaths)
    printf("- %s\n", path.toUtf8().constData());
}

bool WbProtoList::areProtoAssetsAvailable(const QString &filename) {
  // printf("checking %s\n", filename.toUtf8().constData());
  // navigate through all proto and referenced external subproto to ensure they are all available
  QString url = filename;
  if (WbUrl::isWeb(url) && !WbNetwork::instance()->isCached(url))
    return false;

  if (WbUrl::isLocalUrl(url))
    url = QDir::cleanPath(WbStandardPaths::webotsHomePath() + filename.mid(9));

  if (!QFileInfo(url).exists())
    return false;

  QMap<QString, QString> externProtos = getExternProtoList(url);
  mCurrentProjectProtoList.insert(externProtos);

  QMapIterator<QString, QString> it(externProtos);
  bool isProtoAssetAvailable = true;
  while (it.hasNext()) {
    it.next();

    QString path = it.value();
    if (WbUrl::isWeb(path) && WbNetwork::instance()->isCached(path))
      path = WbNetwork::instance()->get(path);
    else if (WbUrl::isLocalUrl(path))
      path = QDir::cleanPath(WbStandardPaths::webotsHomePath() + path.mid(9));

    bool success = areProtoAssetsAvailable(path);
    if (success)
      printf("> AVAILABLE: %s\n", path.toUtf8().constData());
    else
      printf("> NOT AVAILABLE: %s\n", path.toUtf8().constData());
    isProtoAssetAvailable &= success;
  }

  return isProtoAssetAvailable;
}

bool WbProtoList::externProtoExists(const QString &filename) {
  if (filename.startsWith("https://"))
    return WbNetwork::instance()->isCached(filename);

  const QString &path = WbUrl::computePath(NULL, "EXTERNPROTO", filename, false);  // TODO: remplace with something better
  return QFileInfo(path).exists();
}

void WbProtoList::retrieveAllExternProto(QString filename, bool reloading) {
  // reset current project related variables to prepare for a load
  mCurrentProjectProtoList.clear();
  mCurrentWorld = filename;
  mReloading = reloading;
  // ensure all referenced assets are available (locally), if not download them
  connect(this, &WbProtoList::protoRetrieved, this, &WbProtoList::retrievalCompletionTracker);
  recursiveProtoRetrieval(filename);
}

QMap<QString, QString> WbProtoList::getExternProtoList(const QString &filename) {
  // TODO: for now, assume this functions gets a clean locally accessible path. Is it better if this function does the
  // cleaning?
  QMap<QString, QString> protoList;

  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly)) {
    printf("ERROR, %s is not a valid filename for getExternProtoList\n", filename.toUtf8().constData());
    return protoList;
  }

  QRegularExpression re("EXTERNPROTO\\s([a-zA-Z0-9-_+]+)\\s\"(.*\\.proto)\"");  // TODO: test it more
  QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());
  while (it.hasNext()) {
    QRegularExpressionMatch match = it.next();
    if (match.hasMatch()) {
      const QString identifier = match.captured(1);
      const QString url = match.captured(2);

      if (!url.endsWith(identifier + ".proto")) {
        WbLog::error(tr("Malformed extern proto url. The identifier and url do not coincide.\n"));
        return protoList;
      }

      protoList.insert(identifier, url);  // if same identifier, only last url is kept
    }
  }

  return protoList;
}

void WbProtoList::recursiveProtoRetrieval(const QString &filename) {
  printf("recursing: %s\n", filename.toUtf8().constData());
  QString protoPath = filename;
  if (WbUrl::isWeb(filename)) {
    if (WbNetwork::instance()->isCached(filename))
      protoPath = WbNetwork::instance()->get(filename);
    else {
      // referenced proto not yet available, download it
      printf(" > %s not locally available, downloading.\n", filename.toUtf8().constData());
      WbDownloader *downloader = new WbDownloader(this);
      connect(downloader, &WbDownloader::complete, this, &WbProtoList::recurser);
      mRetrievers.push_back(downloader);
      downloader->download(QUrl(filename));
      return;
    }
  } else {
    // if it is a local file, it should exist on disk
    if (!QFileInfo(filename).exists()) {
      mRetrievalError = tr("Local proto asset '%1' not available.\n").arg(filename);
      retrievalCompletionTracker();
      return;
    }
  }

  assert(QFileInfo(protoPath).exists());  // by this point, the file should be locally accessible to recurse through it
  QMap<QString, QString> externProtos = getExternProtoList(protoPath);
  if (externProtos.isEmpty()) {
    emit protoRetrieved();
    return;  // nothing else to recurse into
  }

  QMapIterator<QString, QString> it(externProtos);
  while (it.hasNext()) {
    it.next();
    // manufacture url of sub-proto based on url of the file that references it
    QString externProtoUrl = WbUrl::generateExternProtoPath(it.value(), protoPath);
    printf("---subproto >>%s<<\n---parent   >>%s<<\n   ====> will retrieve: %s\n", (it.value()).toUtf8().constData(),
           protoPath.toUtf8().constData(), externProtoUrl.toUtf8().constData());
    if (WbUrl::isWeb(externProtoUrl)) {
      // retrieve any sub-proto references
      WbDownloader *downloader = new WbDownloader(this);
      connect(downloader, &WbDownloader::complete, this, &WbProtoList::recurser);  // TODO: need intermediary recurser function?
      mRetrievers.push_back(downloader);
      downloader->download(QUrl(externProtoUrl));
      return;
    } else
      recursiveProtoRetrieval(externProtoUrl);
  }

  return;
}

void WbProtoList::recurser() {
  if (!mRetrievalError.isEmpty()) {  // TODO: check if downloader has error instead
    WbLog::error(tr("Proto retrieval error: %1").arg(mRetrievalError));
    disconnect(this, &WbProtoList::protoRetrieved, this, &WbProtoList::retrievalCompletionTracker);
    qDeleteAll(mRetrievers);
    mRetrievers.clear();
    return;
  }

  WbDownloader *retriever = dynamic_cast<WbDownloader *>(sender());
  if (retriever) {
    printf("> download complete for %s.\n", retriever->url().toString().toUtf8().constData());
    recursiveProtoRetrieval(retriever->url().toString());
  }
}

void WbProtoList::retrievalCompletionTracker() {
  // TODO: function is weird
  // TODO: if one file doesn't exist (webots://something/that/doesnt/exist), it gets stuck
  // TODO: finishes before it actually finishes coz mRetrievers pushing back not fast enough?

  bool finished = true;
  for (int i = 0; i < mRetrievers.size(); ++i)
    if (!mRetrievers[i]->hasFinished())
      finished = false;

  if (!mRetrievalError.isEmpty()) {
    disconnect(this, &WbProtoList::protoRetrieved, this, &WbProtoList::retrievalCompletionTracker);
    qDeleteAll(mRetrievers);
    mRetrievers.clear();
    WbLog::error(tr("Proto retrieval error: %1").arg(mRetrievalError));
    return;
  }

  if (mRetrievalError.isEmpty() && finished) {
    printf("FINISHED, files downloaded: %lld\n", mRetrievers.size());
    disconnect(this, &WbProtoList::protoRetrieved, this, &WbProtoList::retrievalCompletionTracker);
    qDeleteAll(mRetrievers);
    mRetrievers.clear();
    // load the world again
    WbApplication::instance()->loadWorld(mCurrentWorld, mReloading);
  }
}

void WbProtoList::setupKnownProtoList() {
  /*
  const QString &searchPath = WbStandardPaths::projectsPath();

  QFileInfoList worlds;
  QDirIterator dit(searchPath, QStringList() << "*.wbt", QDir::Files, QDirIterator::Subdirectories);
  while (dit.hasNext())
    worlds << QFileInfo(dit.next());

  foreach (const QFileInfo &world, worlds) {
    // printf("~~> %s\n", world.absoluteFilePath().toUtf8().constData());
    QMap<QString, QString> externProtos = getExternProtoList(world.absoluteFilePath());
    // note: if externProtos contains a key that already exists in mProtoList, the latter is overwritten
    mProtoList.insert(externProtos);
  }

  printf("-- known proto %lld --\n", mProtoList.size());

  //QMapIterator<QString, QString> it(mProtoList);
  //while (it.hasNext()) {
  //  it.next();
  //  printf("  %30s %s\n", it.key().toUtf8().constData(), it.value().toUtf8().constData());
  //}

  printf("-- end known proto --\n");
  */
  const WbVersion &version = WbApplicationInfo::version();
  // if it's an official release, use the tag (for example R2022b), if it's a nightly use the commit
  const QString &reference = version.commit().isEmpty() ? version.toString() : version.commit();
  const QString filename = WbStandardPaths::resourcesPath() + QString("proto-list-%1.txt").arg(reference);
  QFile protoList(filename);

  if (protoList.open(QIODevice::ReadOnly)) {
    const QStringList lines = QString(protoList.readAll()).split('\n', Qt::SkipEmptyParts);
    foreach (QString line, lines) {
      const QString filename = QUrl(line).fileName().replace(".proto", "");
      mProtoList.insert(filename, line);
    }
  } else
    WbLog::error(tr("%1 not found.").arg(filename));

  // printf("-- known proto %lld --\n", mProtoList.size());
  // QMapIterator<QString, QString> it(mProtoList);
  // while (it.hasNext()) {
  //  it.next();
  //  printf("  %35s %s\n", it.key().toUtf8().constData(), it.value().toUtf8().constData());
  //}
  // printf("-- end known proto --\n");
}

void WbProtoList::resetCurrentProjectProtoList(void) {
  mCurrentProjectProtoList.clear();
  mRetrievalError = QString();
}