// Copyright 1996-2024 Cyberbotics Ltd.
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

#include "WbWorld.hpp"

#include "WbApplication.hpp"
#include "WbBackground.hpp"
#include "WbBallJointParameters.hpp"
#include "WbBasicJoint.hpp"
#include "WbFileUtil.hpp"
#include "WbGeometry.hpp"
#include "WbGroup.hpp"
#include "WbHingeJointParameters.hpp"
#include "WbImageTexture.hpp"
#include "WbJoint.hpp"
#include "WbJointDevice.hpp"
#include "WbJointParameters.hpp"
#include "WbLed.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbMFString.hpp"
#include "WbMotor.hpp"
#include "WbNetwork.hpp"
#include "WbNodeOperations.hpp"
#include "WbNodeReader.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeContact.hpp"
#include "WbPbrAppearance.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPerspective.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbPropeller.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbRenderingDevice.hpp"
#include "WbRobot.hpp"
#include "WbSimulationState.hpp"
#include "WbSlot.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbTemplateManager.hpp"
#include "WbTokenizer.hpp"
#include "WbUrl.hpp"
#include "WbViewpoint.hpp"
#include "WbWorldInfo.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenVertexArrayFrameListener.hpp"
#include "WbWriter.hpp"

#include <wren/scene.h>

#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QTextStream>

#include <ode/fluid_dynamics/ode_fluid_dynamics.h>

#include <cassert>

static WbWorld *gInstance = NULL;
bool WbWorld::cW3dStreaming = false;
bool WbWorld::cPrintExternUrls = false;

WbWorld *WbWorld::instance() {
  return gInstance;
}

WbWorld::WbWorld(WbTokenizer *tokenizer) :
  mWorldLoadingCanceled(false),
  mResetRequested(false),
  mRestartControllers(false),
  mIsModified(false),
  mIsModifiedFromSceneTree(false),
  mWorldInfo(NULL),
  mViewpoint(NULL),
  mPerspective(NULL),
  mLastAwakeningTime(0.0),
  mIsLoading(true),
  mIsCleaning(false),
  mIsVideoRecording(false) {
  WbWrenVertexArrayFrameListener::resetLastUpdateTime();
  gInstance = this;
  WbNode::setInstantiateMode(true);
  WbNode::setGlobalParentNode(NULL);
  mRoot = new WbGroup();
  mRoot->setUniqueId(0);
  WbNode::setGlobalParentNode(mRoot);
  mRadarTargets.clear();
  mCameraRecognitionObjects.clear();

  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::LOADING);

  if (tokenizer) {
    mFileName = tokenizer->fileName();
    mPerspective = new WbPerspective(mFileName);
    mPerspective->load();

    // read/create nodes
    WbNodeReader reader;
    WbApplication::instance()->setWorldLoadingStatus(tr("Parsing nodes"));
    connect(&reader, &WbNodeReader::readNodesHasProgressed, WbApplication::instance(), &WbApplication::setWorldLoadingProgress);
    connect(WbApplication::instance(), &WbApplication::worldLoadingWasCanceled, &reader, &WbNodeReader::cancelReadNodes);
    QList<WbNode *> nodes = reader.readNodes(tokenizer, mFileName);
    disconnect(WbApplication::instance(), &WbApplication::worldLoadingWasCanceled, &reader, &WbNodeReader::cancelReadNodes);
    disconnect(&reader, &WbNodeReader::readNodesHasProgressed, WbApplication::instance(),
               &WbApplication::setWorldLoadingProgress);
    if (WbApplication::instance()->wasWorldLoadingCanceled()) {
      mWorldLoadingCanceled = true;
      return;
    }
    WbTemplateManager::instance()->blockRegeneration(true);
    const WbField *childrenField = mRoot->findField("children");
    int index = 0;
    WbApplication::instance()->setWorldLoadingStatus(tr("Creating nodes"));
    foreach (WbNode *node, nodes) {
      index++;
      WbApplication::instance()->setWorldLoadingProgress(index * 100 / nodes.size());
      if (WbApplication::instance()->wasWorldLoadingCanceled()) {
        mWorldLoadingCanceled = true;
        return;
      }
      QString errorMessage;
      if (WbNodeUtilities::isAllowedToInsert(childrenField, mRoot, errorMessage, WbNode::STRUCTURE_USE,
                                             WbNodeUtilities::slotType(node), node)) {
        node->validate();
        mRoot->addChild(node);
      } else
        mRoot->parsingWarn(errorMessage);
    }
    WbTemplateManager::instance()->blockRegeneration(false);

    // ensure a minimal set of nodes for a functional world
    checkPresenceOfMandatoryNodes();
  } else {
    mFileName = WbProject::newWorldPath();
    mPerspective = new WbPerspective(mFileName);
    mPerspective->load();

    // create default nodes
    mWorldInfo = new WbWorldInfo();
    mViewpoint = new WbViewpoint();
    mRoot->addChild(mWorldInfo);
    mRoot->addChild(mViewpoint);
  }

  WbUrl::setWorldFileName(mFileName);

  WbNode::setGlobalParentNode(NULL);
  updateTopLevelLists();

  // world loading stuff
  connect(root(), &WbGroup::childFinalizationHasProgressed, WbApplication::instance(), &WbApplication::setWorldLoadingProgress);
  connect(root(), &WbGroup::worldLoadingStatusHasChanged, WbApplication::instance(),
          &WbApplication::worldLoadingStatusHasChanged);
  connect(this, &WbWorld::worldLoadingStatusHasChanged, WbApplication::instance(), &WbApplication::setWorldLoadingStatus);
  connect(this, &WbWorld::worldLoadingHasProgressed, WbApplication::instance(), &WbApplication::setWorldLoadingProgress);
  connect(WbApplication::instance(), &WbApplication::worldLoadingWasCanceled, root(), &WbGroup::cancelFinalization);

  WbProtoManager::instance()->setNeedsRobotAncestorCallback(
    [](const QString &nodeType) { return WbNodeUtilities::isDeviceTypeName(nodeType) && nodeType != "Connector"; });
}

void WbWorld::finalize() {
  disconnect(WbApplication::instance(), &WbApplication::worldLoadingWasCanceled, root(), &WbGroup::cancelFinalization);
  disconnect(root(), &WbGroup::worldLoadingStatusHasChanged, WbApplication::instance(),
             &WbApplication::worldLoadingStatusHasChanged);
  disconnect(this, &WbWorld::worldLoadingStatusHasChanged, WbApplication::instance(), &WbApplication::setWorldLoadingStatus);
  disconnect(this, &WbWorld::worldLoadingHasProgressed, WbApplication::instance(), &WbApplication::setWorldLoadingProgress);
  disconnect(root(), &WbGroup::childFinalizationHasProgressed, WbApplication::instance(),
             &WbApplication::setWorldLoadingProgress);
  if (WbApplication::instance()->wasWorldLoadingCanceled())
    mWorldLoadingCanceled = true;

  connect(mRoot, &WbGroup::topLevelListsUpdateRequested, this, &WbWorld::updateTopLevelLists);
  connect(mWorldInfo, &WbWorldInfo::globalPhysicsPropertiesChanged, this, &WbWorld::awake);
  connect(WbNodeOperations::instance(), &WbNodeOperations::nodeAdded, this, &WbWorld::storeAddedNodeIfNeeded);

  if (WbProject::current())
    connect(WbProject::current(), &WbProject::pathChanged, this, &WbWorld::updateProjectPath);

  // check for Solid name clash
  QSet<const QString> topSolidNameSet;
  foreach (WbSolid *s, mTopSolids)
    s->resolveNameClashIfNeeded(false, true, mTopSolids, &topSolidNameSet);
}

WbWorld::~WbWorld() {
  delete mRoot;
  WbNode::cleanup();
  gInstance = NULL;

  delete mPerspective;

  WbWrenOpenGlContext::makeWrenCurrent();
  // Sanity-check: make sure only the root wren::Transform remains
  assert(wr_scene_compute_node_count(wr_scene_get_instance()) == 1);
  wr_scene_reset(wr_scene_get_instance());
  WbWrenOpenGlContext::doneWren();
}

bool WbWorld::needSaving() const {
  if (mIsModifiedFromSceneTree)
    return true;
  if (WbPreferences::instance()->value("General/disableSaveWarning").toBool())
    return false;
  else
    return mIsModified;
}

void WbWorld::setModifiedFromSceneTree() {
  if (!mIsModifiedFromSceneTree) {
    mIsModifiedFromSceneTree = true;
    setModified();
  }
}

void WbWorld::setModified(bool isModified) {
  if (mIsModified != isModified) {
    mIsModified = isModified;
    emit modificationChanged(isModified);
  }
}

bool WbWorld::saveAs(const QString &fileName) {
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
    return false;

  WbWriter writer(&file, fileName);
  writer.writeHeader(fileName);

  writer << "\n";  // leave one space between header and body regardless of whether there are EXTERNPROTO or not

  // prior to saving the EXTERNPROTO entries to file, purge the unused entries
  WbNodeOperations::instance()->purgeUnusedExternProtoDeclarations();
  const QVector<WbExternProto *> &externProto = WbProtoManager::instance()->externProto();
  for (int i = 0; i < externProto.size(); ++i) {
    const QString &url = WbProtoManager::instance()->formatExternProtoPath(externProto[i]->url());
    writer << QString("%1EXTERNPROTO \"%2\"\n").arg(externProto[i]->isImportable() ? "IMPORTABLE " : "").arg(url);
    if (i == externProto.size() - 1)
      writer << "\n";  // add additional empty line after the last EXTERNPROTO entry
  }

  for (int i = 0; i < mRoot->childCount(); ++i) {
    mRoot->child(i)->write(writer);
    writer << "\n";
  }

  writer.writeFooter();

  mFileName = fileName;
  WbUrl::setWorldFileName(mFileName);

  mIsModified = false;
  mIsModifiedFromSceneTree = false;
  emit modificationChanged(false);

  storeLastSaveTime();

  mRoot->save("__init__");
  return true;
}

bool WbWorld::save() {
  return saveAs(mFileName);
}

bool WbWorld::exportAsHtml(const QString &fileName, bool animation) const {
  assert(fileName.endsWith(".html", Qt::CaseInsensitive));

  WbSimulationState *simulationState = WbSimulationState::instance();
  simulationState->pauseSimulation();

  QString w3dFilename = fileName;
  w3dFilename.replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".w3d");

  QString cssFileName = fileName;
  cssFileName.replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".css");

  bool success = true;

  try {
    // export w3d file
    success = exportAsW3d(w3dFilename);
    if (!success)
      throw tr("Cannot export the w3d file to '%1'").arg(w3dFilename);

    // export css file
    QString typeString = (animation) ? "Animation" : "Scene";
    QString titleString(WbWorld::instance()->worldInfo()->title());
    titleString = titleString.toHtmlEscaped();

    QList<std::pair<QString, QString>> cssTemplateValues;
    cssTemplateValues << std::pair<QString, QString>("%title%", titleString);
    cssTemplateValues << std::pair<QString, QString>("%type%", typeString);

    success = WbFileUtil::copyAndReplaceString(WbStandardPaths::resourcesWebPath() + "templates/w3d_playback.css", cssFileName,
                                               cssTemplateValues);
    if (!success)
      throw tr("Cannot copy the 'w3d_playback.css' file to '%1'").arg(cssFileName);

    // export html file
    QString infoString;
    const WbMFString &info = WbWorld::instance()->worldInfo()->info();
    for (int i = 0; i < info.size(); ++i) {
      QString line = info.itemToString(i, WbPrecision::DOUBLE_MAX);
      line.replace(QRegularExpression("^\""), "");
      line.replace(QRegularExpression("\"$"), "");
      infoString += line + "\n";
    }

    infoString = infoString.toHtmlEscaped();
    infoString.replace("\n", "<br/>");

    QList<std::pair<QString, QString>> templateValues;
    templateValues << std::pair<QString, QString>("%w3dFilename%", QFileInfo(w3dFilename).fileName());
    templateValues << std::pair<QString, QString>("%type%", typeString);
    templateValues << std::pair<QString, QString>("%title%", titleString);
    templateValues << std::pair<QString, QString>("%description%", infoString);
    templateValues << std::pair<QString, QString>(
      "%w3dName%",
      fileName.split('/').last().replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".w3d"));
    templateValues << std::pair<QString, QString>(
      "%jpgName%",
      fileName.split('/').last().replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".jpg"));
    templateValues << std::pair<QString, QString>(
      "%cssName%",
      fileName.split('/').last().replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".css"));
    if (animation)
      templateValues << std::pair<QString, QString>(
        "%jsonName%",
        fileName.split('/').last().replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".json"));
    else
      templateValues << std::pair<QString, QString>("%jsonName%", "");

    success = WbFileUtil::copyAndReplaceString(WbStandardPaths::resourcesWebPath() + "templates/w3d_playback.html", fileName,
                                               templateValues);
    if (!success)
      throw tr("Cannot copy 'w3d_playback.html' to '%1'").arg(fileName);

  } catch (const QString &e) {
    WbLog::error(tr("Cannot export html: '%1'").arg(e), true);
  }

  simulationState->resumeSimulation();
  return success;
}

bool WbWorld::exportAsW3d(const QString &fileName) const {
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
    return false;

  WbWriter writer(&file, fileName);
  write(writer);

  return true;
}

void WbWorld::write(WbWriter &writer) const {
  if (writer.isW3d()) {
    // make sure all the meshes data are up-to-date
    // only W3D exporter relies on OpenGL data
    // this is needed for example in minimize and streaming mode because the world is exported before the first main rendering
    WbWrenOpenGlContext::makeWrenCurrent();
    wr_scene_apply_pending_updates(wr_scene_get_instance());
    WbWrenOpenGlContext::doneWren();
  }

  writer.writeHeader(worldInfo()->title());

  // write nodes
  const int count = mRoot->childCount();
  for (int i = 0; i < count; ++i) {
    mRoot->child(i)->write(writer);
    writer << "\n";
  }
  QStringList list;
  const WbMFString &info = worldInfo()->info();
  const int n = info.size();
  for (int i = 0; i < n; ++i)
    list << info.item(i);
  writer.writeFooter(&list);
}

WbNode *WbWorld::findTopLevelNode(const QString &modelName, int preferredPosition) const {
  WbNode *result = NULL;

  WbMFNode::Iterator it(mRoot->children());
  int position = 0;
  while (it.hasNext()) {
    WbNode *const node = it.next();
    if (node->nodeModelName() == modelName) {
      if (result)
        WbLog::warning(tr("'%1': found duplicate %2 node.").arg(mFileName, modelName), false, WbLog::PARSING);
      else {
        result = node;
        if (position != preferredPosition)
          WbLog::warning(tr("'%1': %2 node should be preferably included at position %3 instead of position %4.")
                           .arg(mFileName)
                           .arg(modelName)
                           .arg(preferredPosition + 1)
                           .arg(position + 1),
                         false, WbLog::PARSING);
      }
    }
    ++position;
  }

  if (!result)
    WbLog::warning(tr("'%1': added missing %2 node.").arg(mFileName, modelName), false, WbLog::PARSING);

  return result;
}

void WbWorld::checkPresenceOfMandatoryNodes() {
  mWorldInfo = static_cast<WbWorldInfo *>(findTopLevelNode("WorldInfo", 0));
  if (!mWorldInfo) {
    mWorldInfo = new WbWorldInfo();
    mRoot->insertChild(0, mWorldInfo);
  }

  mViewpoint = static_cast<WbViewpoint *>(findTopLevelNode("Viewpoint", 1));
  if (!mViewpoint) {
    mViewpoint = new WbViewpoint();
    mRoot->insertChild(1, mViewpoint);
  }
}

WbSolid *WbWorld::findSolid(const QString &name) const {
  WbMFNode::Iterator it(mRoot->children());
  while (it.hasNext()) {
    WbSolid *const solidChild = dynamic_cast<WbSolid *>(it.next());
    if (solidChild) {
      WbSolid *const found = solidChild->findSolid(name);
      if (found)
        return found;
    }
  }
  return NULL;
}

QList<WbSolid *> WbWorld::findSolids(bool visibleNodes) const {
  const QList<WbNode *> &allNodes = mRoot->subNodes(true, !visibleNodes, false);
  QList<WbSolid *> allSolids;

  foreach (WbNode *const node, allNodes) {
    // cppcheck-suppress constVariablePointer
    WbSolid *const solid = dynamic_cast<WbSolid *>(node);
    if (solid)
      allSolids.append(solid);
  }

  return allSolids;
}

QList<std::pair<QString, WbMFString *>> WbWorld::listTextureFiles() const {
  QList<std::pair<QString, WbMFString *>> list = mRoot->listTextureFiles();
  return list;
}

// update the list of robots and top level solids
void WbWorld::updateTopLevelLists() {
  mTopSolids.clear();
  mTopSolids = WbNodeUtilities::findSolidDescendants(mRoot);
}

void WbWorld::removeRobotIfPresent(WbRobot *robot) {
  if (!robot)
    return;

  mRobots.removeAll(robot);
  emit robotRemoved(robot);
}

void WbWorld::addRobotIfNotAlreadyPresent(WbRobot *robot) {
  assert(robot);

  // don't add a robot that's already in the global list
  if (mRobots.contains(robot))
    return;

  mRobots.append(robot);
  setUpControllerForNewRobot(robot);
  emit robotAdded(robot);
}

void WbWorld::updateProjectPath(const QString &oldPath, const QString &newPath) {
  const QFileInfo infoPath(mFileName);
  const QFileInfo infoNewPath(newPath);
  const QString newFilename = infoNewPath.absolutePath() + "/worlds/" + infoPath.fileName();
  if (QFile::exists(newFilename)) {
    mFileName = newFilename;
    WbUrl::setWorldFileName(mFileName);
  }
}

void WbWorld::setViewpoint(WbViewpoint *viewpoint) {
  bool viewpointHasChanged = mViewpoint != viewpoint;
  mViewpoint = viewpoint;
  if (viewpointHasChanged)
    emit viewpointChanged();
}

double WbWorld::orthographicViewHeight() const {
  return mViewpoint->orthographicViewHeight();
}

void WbWorld::setOrthographicViewHeight(double ovh) const {
  mViewpoint->setOrthographicViewHeight(ovh);
}

bool WbWorld::reloadPerspective() {
  delete mPerspective;
  mPerspective = new WbPerspective(mFileName);
  return mPerspective->load();
}

void WbWorld::appendOdeContact(const WbOdeContact &odeContact) {
  mOdeContactsMutex.lock();  // TODO: understand why this mutex is here. Related with MT-safe physics plugin?
  mOdeContacts << odeContact;
  mOdeContactsMutex.unlock();
}

void WbWorld::appendOdeImmersionGeom(const dImmersionGeom &immersionGeom) {
  mOdeContactsMutex.lock();  // TODO: check if this mutex is necessary and if it needs to be renamed
  mImmersionGeoms.append(immersionGeom);
  mOdeContactsMutex.unlock();
}

void WbWorld::awake() {
  double currentSimulationTime = WbSimulationState::instance()->time();
  if (currentSimulationTime > mLastAwakeningTime) {  // we don't want to awake all the world several times in the same step
    mLastAwakeningTime = currentSimulationTime;
    WbMFNode::Iterator it(mRoot->children());
    while (it.hasNext()) {
      WbGroup *const group = dynamic_cast<WbGroup *>(it.next());
      if (group)
        WbSolid::awakeSolids(group);
    }
  }
}

void WbWorld::retrieveNodeNamesWithOptionalRendering(QStringList &centerOfMassNodeNames, QStringList &centerOfBuoyancyNodeNames,
                                                     QStringList &supportPolygonNodeNames) const {
  centerOfMassNodeNames.clear();
  centerOfBuoyancyNodeNames.clear();
  supportPolygonNodeNames.clear();

  const WbSolid *solid = NULL;
  const QList<WbNode *> &allNodes = mRoot->subNodes(true);
  for (int i = 0; i < allNodes.size(); ++i) {
    solid = dynamic_cast<WbSolid *>(allNodes[i]);
    if (solid && (solid->globalCenterOfMassRepresentationEnabled() || solid->centerOfBuoyancyRepresentationEnabled() ||
                  solid->supportPolygonRepresentationEnabled())) {
      const QString name = solid->computeUniqueName();
      if (solid->globalCenterOfMassRepresentationEnabled())
        centerOfMassNodeNames << name;
      if (solid->centerOfBuoyancyRepresentationEnabled())
        centerOfBuoyancyNodeNames << name;
      if (solid->supportPolygonRepresentationEnabled())
        supportPolygonNodeNames << name;
    }
  }
}

QString WbWorld::logWorldMetrics() const {
  int solidCount = 0;
  int jointCount = 0;
  int geomCount = 0;
  const QList<WbNode *> &allNodes = mRoot->subNodes(true);
  foreach (const WbNode *node, allNodes) {
    if (dynamic_cast<const WbBasicJoint *>(node)) {
      jointCount++;
      continue;
    }
    const WbSolid *solid = dynamic_cast<const WbSolid *>(node);
    if (solid && (solid->isKinematic() || solid->isSolidMerger())) {
      solidCount++;
      continue;
    }
    const WbGeometry *geometry = dynamic_cast<const WbGeometry *>(node);
    if (geometry && !geometry->isInBoundingObject())
      geomCount++;
  }

  return QString("%1 solids, %2 joints, %3 graphical geometries").arg(solidCount).arg(jointCount).arg(geomCount);
}
