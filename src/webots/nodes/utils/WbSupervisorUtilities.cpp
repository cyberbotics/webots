// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbSupervisorUtilities.hpp"

#include "WbAbstractCamera.hpp"
#include "WbApplication.hpp"
#include "WbDictionary.hpp"
#include "WbField.hpp"
#include "WbFieldModel.hpp"
#include "WbMFBool.hpp"
#include "WbMFColor.hpp"
#include "WbMFDouble.hpp"
#include "WbMFInt.hpp"
#include "WbMFNode.hpp"
#include "WbMFRotation.hpp"
#include "WbMFString.hpp"
#include "WbMFVector2.hpp"
#include "WbMFVector3.hpp"
#include "WbNodeOperations.hpp"
#include "WbNodeUtilities.hpp"
#include "WbProject.hpp"
#include "WbRgb.hpp"
#include "WbRobot.hpp"
#include "WbSFColor.hpp"
#include "WbSFNode.hpp"
#include "WbSFVector2.hpp"
#include "WbSelection.hpp"
#include "WbStandardPaths.hpp"
#include "WbTemplateManager.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenOpenGlContext.hpp"

#ifdef _WIN32
#include "WbVirtualRealityHeadset.hpp"
#endif

#include "../../../include/controller/c/webots/supervisor.h"
#include "../../lib/Controller/api/messages.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <cassert>

static const int MAX_LABELS = 100;

struct WbFieldGetRequest {
  WbField *field;
  int index;  // for MF fields only
};

struct WbDeletedNodeInfo {
  int nodeId;
  WbNode *parent;
  WbField *parentField;
};

class WbFieldSetRequest {
public:
  virtual void apply() const = 0;
  virtual ~WbFieldSetRequest() {}

protected:
  WbFieldSetRequest(WbField *field, int index) : mField(field), mIndex(index) {}
  WbField *mField;
  int mIndex;
};

class WbBoolFieldSetRequest : public WbFieldSetRequest {
public:
  WbBoolFieldSetRequest(WbField *f, int index, bool value) : WbFieldSetRequest(f, index), mValue(value) {
    assert((f->type() == WB_SF_BOOL && dynamic_cast<WbSFBool *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_BOOL && dynamic_cast<WbMFBool *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<WbSFBool *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<WbMFBool *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  bool mValue;
};

class WbIntFieldSetRequest : public WbFieldSetRequest {
public:
  WbIntFieldSetRequest(WbField *f, int index, int value) : WbFieldSetRequest(f, index), mValue(value) {
    assert((f->type() == WB_SF_INT32 && dynamic_cast<WbSFInt *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_INT32 && dynamic_cast<WbMFInt *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<WbSFInt *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<WbMFInt *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  int mValue;
};

class WbDoubleFieldSetRequest : public WbFieldSetRequest {
public:
  WbDoubleFieldSetRequest(WbField *f, int index, double value) : WbFieldSetRequest(f, index), mValue(value) {
    assert((f->type() == WB_SF_FLOAT && dynamic_cast<WbSFDouble *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_FLOAT && dynamic_cast<WbMFDouble *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<WbSFDouble *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<WbMFDouble *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  double mValue;
};

class WbVector2FieldSetRequest : public WbFieldSetRequest {
public:
  WbVector2FieldSetRequest(WbField *f, int index, double x, double y) : WbFieldSetRequest(f, index), mValue(x, y) {
    assert((f->type() == WB_SF_VEC2F && dynamic_cast<WbSFVector2 *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_VEC2F && dynamic_cast<WbMFVector2 *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<WbSFVector2 *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<WbMFVector2 *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  WbVector2 mValue;
};

class WbVector3FieldSetRequest : public WbFieldSetRequest {
public:
  WbVector3FieldSetRequest(WbField *f, int index, double x, double y, double z) : WbFieldSetRequest(f, index), mValue(x, y, z) {
    assert((f->type() == WB_SF_VEC3F && dynamic_cast<WbSFVector3 *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_VEC3F && dynamic_cast<WbMFVector3 *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<WbSFVector3 *>(mField->value()))->setValueByUser(mValue, true);
    else
      (dynamic_cast<WbMFVector3 *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  WbVector3 mValue;
};

class WbColorFieldSetRequest : public WbFieldSetRequest {
public:
  WbColorFieldSetRequest(WbField *f, int index, double red, double green, double blue) :
    WbFieldSetRequest(f, index),
    mValue(red, green, blue) {
    assert((f->type() == WB_SF_COLOR && dynamic_cast<WbSFColor *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_COLOR && dynamic_cast<WbMFColor *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<WbSFColor *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<WbMFColor *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  WbRgb mValue;
};

class WbRotationFieldSetRequest : public WbFieldSetRequest {
public:
  WbRotationFieldSetRequest(WbField *f, int index, double x, double y, double z, double a) :
    WbFieldSetRequest(f, index),
    mValue(x, y, z, a) {
    assert((f->type() == WB_SF_ROTATION && dynamic_cast<WbSFRotation *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_ROTATION && dynamic_cast<WbMFRotation *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<WbSFRotation *>(mField->value()))->setValueByUser(mValue, true);
    else
      (dynamic_cast<WbMFRotation *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  WbRotation mValue;
};

class WbStringFieldSetRequest : public WbFieldSetRequest {
public:
  WbStringFieldSetRequest(WbField *f, int index, const QString &string) : WbFieldSetRequest(f, index), mValue(string) {
    assert((f->type() == WB_SF_STRING && dynamic_cast<WbSFString *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_STRING && dynamic_cast<WbMFString *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<WbSFString *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<WbMFString *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  QString mValue;
};

WbSupervisorUtilities::WbSupervisorUtilities(WbRobot *robot) : mRobot(robot) {
  initControllerRequests();

  connect(WbApplication::instance(), &WbApplication::animationStartStatusChanged, this,
          &WbSupervisorUtilities::animationStartStatusChanged);
  connect(WbApplication::instance(), &WbApplication::animationStopStatusChanged, this,
          &WbSupervisorUtilities::animationStopStatusChanged);
  connect(WbApplication::instance(), &WbApplication::videoCreationStatusChanged, this,
          &WbSupervisorUtilities::movieStatusChanged);
  connect(WbNodeOperations::instance(), &WbNodeOperations::nodeDeleted, this, &WbSupervisorUtilities::updateDeletedNodeList);

  // Do not apply the change simulation mode during dealing with a controller message
  // otherwise, conflicts can occur in case of multiple controllers
  connect(this, &WbSupervisorUtilities::changeSimulationModeRequested, this, &WbSupervisorUtilities::changeSimulationMode,
          Qt::QueuedConnection);
}

WbSupervisorUtilities::~WbSupervisorUtilities() {
  foreach (int labelId, mLabelIds)
    WbWrenLabelOverlay::removeLabel(labelId);

  deleteControllerRequests();
}

void WbSupervisorUtilities::deleteControllerRequests() {
  foreach (WbFieldSetRequest *request, mFieldSetRequests)
    delete request;
  mFieldSetRequests.clear();
  delete mFieldGetRequest;
  delete mAnimationStartStatus;
  delete mAnimationStopStatus;
  delete mMovieStatus;
  delete mSaveStatus;
}

void WbSupervisorUtilities::initControllerRequests() {
  mFoundNodeUniqueId = -1;
  mFoundNodeType = 0;
  mFoundNodeParentUniqueId = -1;
  mFoundFieldId = -2;
  mFoundFieldType = 0;
  mFoundFieldCount = -1;
  mGetSelectedNode = false;
  mGetFromId = false;
  mNeedToRestartController = false;
  mNodeGetPosition = NULL;
  mNodeGetOrientation = NULL;
  mNodeGetCenterOfMass = NULL;
  mNodeGetContactPoints = NULL;
  mNodeGetStaticBalance = NULL;
  mNodeGetVelocity = NULL;
  mImportedNodesNumber = -1;
  mLoadWorldRequested = false;
  mVirtualRealityHeadsetIsUsedRequested = false;
  mVirtualRealityHeadsetPositionRequested = false;
  mVirtualRealityHeadsetOrientationRequested = false;
  mFieldGetRequest = NULL;
  mAnimationStartStatus = NULL;
  mAnimationStopStatus = NULL;
  mMovieStatus = NULL;
  mSaveStatus = NULL;
  mWorldToLoad.clear();
  mNodesDeletedSinceLastStep.clear();
}

QString WbSupervisorUtilities::readString(QDataStream &stream) {
  QByteArray txt;
  unsigned char uc;
  do {
    stream >> (unsigned char &)uc;
    txt.append(uc);
  } while (uc != 0 && !stream.atEnd());
  return QString(txt.constData());
}

// if filename is relative: make it absolute with respect to Supervisor's controller dir
void WbSupervisorUtilities::makeFilenameAbsolute(QString &filename) {
  if (QDir::isRelativePath(filename)) {
    QDir dir(mRobot->controllerDir());
    filename = dir.absoluteFilePath(filename);
  }
}

WbSimulationState::Mode WbSupervisorUtilities::convertSimulationMode(int supervisorMode) {
  switch (supervisorMode) {
    case WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME:
      return WbSimulationState::REALTIME;
    case WB_SUPERVISOR_SIMULATION_MODE_RUN:
      return WbSimulationState::RUN;
    case WB_SUPERVISOR_SIMULATION_MODE_FAST:
      return WbSimulationState::FAST;
    default:
      return WbSimulationState::PAUSE;
  }
}

void WbSupervisorUtilities::processImmediateMessages() {
  int n = mFieldSetRequests.size();
  if (n == 0)
    return;
  WbTemplateManager::instance()->blockRegeneration(true);
  for (int i = 0; i < n; ++i) {
    const WbFieldSetRequest *r = mFieldSetRequests.at(i);
    WbWrenOpenGlContext::makeWrenCurrent();
    r->apply();
    WbWrenOpenGlContext::doneWren();
    delete r;
  }
  mFieldSetRequests.clear();
  WbTemplateManager::instance()->blockRegeneration(false);
  emit worldModified();
}

void WbSupervisorUtilities::postPhysicsStep() {
  if (mLoadWorldRequested) {
    emit WbApplication::instance()->worldLoadRequested(mWorldToLoad);
    mLoadWorldRequested = false;
  }

  if (mNeedToRestartController) {
    mRobot->restartController();
    mNeedToRestartController = false;
  }
}

void WbSupervisorUtilities::reset() {
  foreach (int labelId, mLabelIds)
    WbWrenLabelOverlay::removeLabel(labelId);
  mLabelIds.clear();

  // delete pending requests and reinitialize them
  deleteControllerRequests();
  initControllerRequests();
}

const WbNode *WbSupervisorUtilities::getNodeFromDEF(const QString &defName, const WbNode *fromNode) {
  assert(!defName.isEmpty());
  if (defName.isEmpty())
    return NULL;

  const QStringList list(defName.split("."));

  const QString &currentDefName = list.at(0);
  const int remainingChars = defName.size() - (currentDefName.size() + 1);
  const QString &nextDefName = (remainingChars <= 0) ? QString() : defName.right(remainingChars);

  const WbNode *baseNode = fromNode;
  if (baseNode == NULL) {
    baseNode = WbDictionary::instance()->getNodeFromDEF(currentDefName);
    if (!baseNode || nextDefName.isEmpty())
      return baseNode;
    return getNodeFromDEF(nextDefName, baseNode);
  }

  const QList<WbNode *> &descendants = baseNode->subNodes(false, false, false);
  for (int i = 0; i < descendants.size(); ++i) {
    const WbNode *child = descendants.at(i);
    if (child->defName() == currentDefName) {
      if (nextDefName.isEmpty())
        return child;
      return getNodeFromDEF(nextDefName, child);
    }
  }

  return NULL;
}

void WbSupervisorUtilities::notifyNodeUpdate(WbNode *node) {
  if (!mRobot->isConfigureDone())
    return;
  // send updated node info to the libController
  // this is mainly used to update the cached DEF names
  mUpdatedNodeIds.append(node->uniqueId());
}

WbNode *WbSupervisorUtilities::getProtoParameterNodeInstance(WbNode *const node) const {
  if (node && node->isProtoParameterNode()) {
    // if node is a proto parameter node we need to find the corresponding proto parameter node instance
    for (int i = 0; i < node->protoParameterNodeInstances().size(); ++i) {
      WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(node->protoParameterNodeInstances()[i]);
      if (baseNode->isPostFinalizedCalled())  // if there is more than one proto parameter node instance the valid one is the
                                              // one finalized
        return baseNode;
    }
  }
  return node;
}

void WbSupervisorUtilities::changeSimulationMode(int newMode) {
  WbSimulationState::Mode mode = convertSimulationMode(newMode);
  WbSimulationState::instance()->setMode(mode);
}

void WbSupervisorUtilities::updateDeletedNodeList(WbNode *node) {
  if (!node)
    return;

  struct WbDeletedNodeInfo nodeInfo;
  nodeInfo.nodeId = node->uniqueId();
  nodeInfo.parent = node->parent();
  nodeInfo.parentField = node->parentField();
  mNodesDeletedSinceLastStep.push_back(nodeInfo);
}

void WbSupervisorUtilities::handleMessage(QDataStream &stream) {
  unsigned char byte;
  stream >> (unsigned char &)byte;

  switch (byte) {
    case C_SUPERVISOR_SIMULATION_QUIT: {
      int exitStatus;
      stream >> (int &)exitStatus;
      WbApplication::instance()->simulationQuit(exitStatus);
      return;
    }
    case C_SUPERVISOR_SIMULATION_RESET:
      WbApplication::instance()->simulationReset();
      return;
    case C_SUPERVISOR_RELOAD_WORLD:
      WbApplication::instance()->worldReload();
      return;
    case C_SUPERVISOR_SIMULATION_RESET_PHYSICS:
      WbApplication::instance()->resetPhysics();
      return;
    case C_SUPERVISOR_SIMULATION_CHANGE_MODE: {
      int newMode;
      stream >> (int &)newMode;
      emit changeSimulationModeRequested(newMode);
      return;
    }
    case C_SUPERVISOR_SET_LABEL: {
      unsigned short id;
      double x, y, size;
      unsigned int color;

      stream >> (unsigned short &)id;
      stream >> (double &)x;
      stream >> (double &)y;
      stream >> (double &)size;
      stream >> (unsigned int &)color;
      const QString &text = readString(stream);
      const QString font = readString(stream);

      bool fileFound = false;
      QString filename = WbStandardPaths::fontsPath() + font + ".ttf";
      if (QFile::exists(filename))
        fileFound = true;
      else {
        filename = WbProject::current()->path() + "fonts/" + font + ".ttf";
        if (QFile::exists(filename))
          fileFound = true;
      }

      if (!fileFound) {
        mRobot->warn(tr("wb_supervisor_set_label() called with an invalid '%1' font, 'Arial' used instead.").arg(font));
        filename = WbStandardPaths::fontsPath() + "Arial.ttf";
      }

      if (id < MAX_LABELS) {
        int labelId = (int)id + mRobot->uniqueId() * MAX_LABELS;  // kind of hash to avoid an id clash.

        mLabelIds.removeAll(labelId);
        mLabelIds << labelId;

        WbWrenLabelOverlay *label = WbWrenLabelOverlay::createOrRetrieve(labelId, filename);
        QString error = label->getFontError();
        if (error != "") {
          mRobot->warn(tr(error.toStdString().c_str()));
          return;
        }
        label->setText(text);
        label->setPosition(x, y);
        label->setSize(size);
        label->setColor(color);
        label->applyChangesToWren();
        emit labelChanged(createLabelUpdateString(label));
      } else
        mRobot->warn(tr("wb_supervisor_set_label() is out of range. The supported range is [0, %1].").arg(MAX_LABELS));

      return;
    }
    case C_SUPERVISOR_EXPORT_IMAGE: {
      unsigned char quality;
      stream >> (unsigned char &)quality;
      QString filename = readString(stream);
      makeFilenameAbsolute(filename);
      WbApplication::instance()->takeScreenshot(filename, quality);
      return;
    }
    case C_SUPERVISOR_START_MOVIE: {
      int width, height;
      unsigned char codec, quality, acceleration, caption;
      stream >> (int &)width;
      stream >> (int &)height;
      stream >> (unsigned char &)codec;
      stream >> (unsigned char &)quality;
      stream >> (unsigned char &)acceleration;
      stream >> (unsigned char &)caption;
      QString filename = readString(stream);
      makeFilenameAbsolute(filename);
      WbApplication::instance()->startVideoCapture(filename, codec, width, height, quality, acceleration, caption == 1);
      return;
    }
    case C_SUPERVISOR_STOP_MOVIE:
      WbApplication::instance()->stopVideoCapture();
      return;
    case C_SUPERVISOR_START_ANIMATION: {
      QString filename = readString(stream);
      makeFilenameAbsolute(filename);
      WbApplication::instance()->startAnimationCapture(filename);
      return;
    }
    case C_SUPERVISOR_STOP_ANIMATION:
      WbApplication::instance()->stopAnimationCapture();
      return;
    case C_SUPERVISOR_NODE_GET_FROM_ID: {
      int id;

      stream >> (int &)id;

      const WbBaseNode *node = dynamic_cast<const WbBaseNode *>(WbNode::findNode(id));

      if (node) {
        // since 8.6 -> each message has its own mechanism
        mGetFromId = true;
        mCurrentDefName = node->defName();
        mFoundNodeUniqueId = node->uniqueId();
        mFoundNodeType = node->nodeType();
        mFoundNodeModelName = node->modelName();
        mFoundNodeParentUniqueId = (node->parent() ? node->parent()->uniqueId() : -1);
        connect(node, &WbNode::defUseNameChanged, this, &WbSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
      }

      return;
    }
    case C_SUPERVISOR_NODE_GET_FROM_DEF: {
      const QString &nodeName = readString(stream);
      const WbBaseNode *baseNode = dynamic_cast<const WbBaseNode *>(getNodeFromDEF(nodeName));
      if (baseNode && !baseNode->parentField())  // make sure the parent field is visible
        baseNode = NULL;
      mFoundNodeUniqueId = baseNode ? baseNode->uniqueId() : 0;
      mFoundNodeType = baseNode ? baseNode->nodeType() : 0;
      mFoundNodeModelName = baseNode ? baseNode->modelName() : QString();
      mFoundNodeParentUniqueId = -1;
      if (baseNode) {
        if (baseNode->parent()) {
          if (baseNode->parent() != WbWorld::instance()->root())
            mFoundNodeParentUniqueId = baseNode->parent()->uniqueId();
          else
            mFoundNodeParentUniqueId = 0;
        }
        connect(baseNode, &WbNode::defUseNameChanged, this, &WbSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
      }
      return;
    }
    case C_SUPERVISOR_NODE_GET_SELECTED: {
      const WbBaseNode *baseNode = dynamic_cast<const WbBaseNode *>(WbSelection::instance()->selectedNode());
      if (baseNode) {
        mGetSelectedNode = true;
        mCurrentDefName = baseNode->defName();
        mFoundNodeUniqueId = baseNode->uniqueId();
        mFoundNodeType = baseNode->nodeType();
        mFoundNodeModelName = baseNode->modelName();
        mFoundNodeParentUniqueId = -1;
        if (baseNode->parent()) {
          if (baseNode->parent() != WbWorld::instance()->root())
            mFoundNodeParentUniqueId = baseNode->parent()->uniqueId();
          else
            mFoundNodeParentUniqueId = 0;
        }
        connect(baseNode, &WbNode::defUseNameChanged, this, &WbSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
      }
      return;
    }
    case C_SUPERVISOR_NODE_GET_POSITION: {
      unsigned int id;

      stream >> (unsigned int &)id;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbTransform *const transform = dynamic_cast<WbTransform *>(node);
      mNodeGetPosition = transform;
      if (!transform)
        mRobot->warn(tr("wb_supervisor_node_get_position() can exclusively be used with Transform (or derived)."));
      return;
    }
    case C_SUPERVISOR_NODE_GET_ORIENTATION: {
      unsigned int id;

      stream >> (unsigned int &)id;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbTransform *const transform = dynamic_cast<WbTransform *>(node);
      mNodeGetOrientation = transform;
      if (!transform)
        mRobot->warn(tr("wb_supervisor_node_get_orientation() can exclusively be used with Transform (or derived)."));
      return;
    }
    case C_SUPERVISOR_NODE_GET_CENTER_OF_MASS: {
      unsigned int id;

      stream >> (unsigned int &)id;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbSolid *const solid = dynamic_cast<WbSolid *>(node);
      mNodeGetCenterOfMass = solid;
      if (!solid)
        mRobot->warn(tr("wb_supervisor_node_get_center_of_mass() can exclusively be used with Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_GET_CONTACT_POINTS: {
      unsigned int id;

      stream >> (unsigned int &)id;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbSolid *const solid = dynamic_cast<WbSolid *>(node);
      mNodeGetContactPoints = solid;
      if (!solid)
        mRobot->warn(
          tr("wb_supervisor_node_get_number_of_contact_points() and wb_supervisor_node_get_contact_point() can exclusively "
             "be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_GET_STATIC_BALANCE: {
      unsigned int id;

      stream >> (unsigned int &)id;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbSolid *const solid = dynamic_cast<WbSolid *>(node);
      mNodeGetStaticBalance = solid;
      if (!solid || !solid->isTopLevel())
        mRobot->warn(tr("wb_supervisor_node_get_static_balance() can exclusively be used with a top Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_GET_VELOCITY: {
      unsigned int id;

      stream >> (unsigned int &)id;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbSolid *const solid = dynamic_cast<WbSolid *>(node);
      if (solid)
        mNodeGetVelocity = solid;
      else
        mRobot->warn(tr("wb_supervisor_node_get_velocity() can exclusively be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_SET_VELOCITY: {
      unsigned int id;
      double a0, a1, a2, l0, l1, l2;

      stream >> (unsigned int &)id;
      stream >> (double &)l0;
      stream >> (double &)l1;
      stream >> (double &)l2;
      stream >> (double &)a0;
      stream >> (double &)a1;
      stream >> (double &)a2;

      const double linearVelocity[3] = {l0, l1, l2};
      const double angularVelocity[3] = {a0, a1, a2};
      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbSolid *const solid = dynamic_cast<WbSolid *>(node);
      if (solid) {
        solid->setLinearVelocity(linearVelocity);
        solid->setAngularVelocity(angularVelocity);
      } else
        mRobot->warn(tr("wb_supervisor_node_set_velocity() can exclusively be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_RESET_PHYSICS: {
      unsigned int id;

      stream >> (unsigned int &)id;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbSolid *const solid = dynamic_cast<WbSolid *>(node);
      if (solid)
        solid->resetPhysics();
      else
        mRobot->warn(tr("wb_supervisor_node_reset_physics() can exclusively be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_RESTART_CONTROLLER: {
      unsigned int id;

      stream >> (unsigned int &)id;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(id));
      WbRobot *const robot = dynamic_cast<WbRobot *>(node);
      if (robot == mRobot)
        // we can't restart the controller associated to this supervisor here
        // we postpone it to the end of the physic step
        mNeedToRestartController = true;
      else if (robot)
        robot->restartController();
      else
        mRobot->warn(tr("wb_supervisor_node_restart_controller() can exclusively be used with a Robot"));
      return;
    }
    case C_SUPERVISOR_NODE_SET_VISIBILITY: {
      unsigned int nodeId, fromId;
      unsigned char visible;

      stream >> (unsigned int &)nodeId;
      stream >> (unsigned int &)fromId;
      stream >> (unsigned char &)visible;

      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(nodeId));
      WbNode *const cameraNode = getProtoParameterNodeInstance(WbNode::findNode(fromId));
      WbAbstractCamera *const camera = dynamic_cast<WbAbstractCamera *>(cameraNode);
      WbViewpoint *const viewpoint = dynamic_cast<WbViewpoint *>(cameraNode);
      WbBaseNode *const baseNode = dynamic_cast<WbBaseNode *>(node);
      assert(baseNode);
      if (camera)
        camera->setNodeVisibility(baseNode, visible == 1);
      else if (viewpoint)
        viewpoint->setNodeVisibility(baseNode, visible == 1);
      return;
    }
    case C_SUPERVISOR_NODE_MOVE_VIEWPOINT: {
      unsigned int nodeId;
      stream >> (unsigned int &)nodeId;
      WbNode *const node = getProtoParameterNodeInstance(WbNode::findNode(nodeId));
      WbBaseNode *const baseNode = dynamic_cast<WbBaseNode *>(node);
      assert(baseNode);
      if (WbNodeUtilities::boundingSphereAncestor(baseNode) != NULL)
        WbWorld::instance()->viewpoint()->moveViewpointToObject(baseNode);
      return;
    }
    case C_SUPERVISOR_LOAD_WORLD: {
      mWorldToLoad = readString(stream);
      makeFilenameAbsolute(mWorldToLoad);
      mLoadWorldRequested = true;
      return;
    }
    case C_SUPERVISOR_SAVE_WORLD: {
      bool saveAs;

      stream >> (unsigned char &)saveAs;

      if (saveAs) {
        QString filename = readString(stream);
        makeFilenameAbsolute(filename);
        bool status = WbWorld::instance()->saveAs(filename);
        mSaveStatus = new bool[1];
        *mSaveStatus = status;
      } else {
        bool status = WbWorld::instance()->save();
        mSaveStatus = new bool[1];
        *mSaveStatus = status;
      }
      return;
    }
    case C_SUPERVISOR_FIELD_GET_FROM_NAME: {
      int id;
      stream >> (int &)id;
      const QString name = readString(stream);

      mFoundFieldId = -1;
      mFoundFieldType = 0;
      mFoundFieldCount = -1;

      WbNode *const node = WbNode::findNode(id);
      if (node) {
        // qDebug() << "Node=" << node->fullName() << " uniqueId=" << id << " fieldName=" << name;
        id = node->findFieldId(name);
        if (id != -1) {
          WbField *field = node->field(id);
          if (field) {
            WbMultipleValue *mv = dynamic_cast<WbMultipleValue *>(field->value());
            mFoundFieldCount = mv ? mv->size() : -1;
            mFoundFieldId = id;
            mFoundFieldType = field->type();
          }
        }
      }
      return;
    }
    case C_SUPERVISOR_FIELD_GET_VALUE: {
      unsigned int uniqueId, fieldId;
      int index = -1;

      stream >> (unsigned int &)uniqueId;
      stream >> (unsigned int &)fieldId;

      WbNode *const node = WbNode::findNode(uniqueId);
      WbField *field = NULL;

      if (node) {
        field = node->field(fieldId);
        if (field && field->isMultiple())
          stream >> (int &)index;
      }

      assert(!mFieldGetRequest);
      mFieldGetRequest = new struct WbFieldGetRequest;
      mFieldGetRequest->field = field;
      mFieldGetRequest->index = index;
      return;
    }
    case C_SUPERVISOR_FIELD_SET_VALUE: {
      unsigned int uniqueId, fieldId, fieldType;
      int index;

      stream >> (unsigned int &)uniqueId;
      stream >> (unsigned int &)fieldId;
      stream >> (unsigned int &)fieldType;
      stream >> (int &)index;
      WbNode *const node = WbNode::findNode(uniqueId);
      WbField *field = node ? node->field(fieldId) : NULL;

      // we read the data depending on the field type
      unsigned char b = 0;
      int i = 0;
      double d0 = 0.0, d1 = 0.0, d2 = 0.0, d3 = 0.0;

      switch (fieldType) {
        case WB_SF_BOOL:
        case WB_MF_BOOL:
          stream >> (unsigned char &)b;
          mFieldSetRequests << new WbBoolFieldSetRequest(field, index, b);
          break;
        case WB_SF_INT32:
        case WB_MF_INT32:
          stream >> (int &)i;
          mFieldSetRequests << new WbIntFieldSetRequest(field, index, i);
          break;
        case WB_SF_FLOAT:
        case WB_MF_FLOAT:
          stream >> (double &)d0;
          mFieldSetRequests << new WbDoubleFieldSetRequest(field, index, d0);
          break;
        case WB_SF_VEC2F:
        case WB_MF_VEC2F:
          stream >> (double &)d0;
          stream >> (double &)d1;
          mFieldSetRequests << new WbVector2FieldSetRequest(field, index, d0, d1);
          break;
        case WB_SF_COLOR:
        case WB_MF_COLOR:
          stream >> (double &)d0;
          stream >> (double &)d1;
          stream >> (double &)d2;
          mFieldSetRequests << new WbColorFieldSetRequest(field, index, d0, d1, d2);
          break;
        case WB_SF_VEC3F:
        case WB_MF_VEC3F:
          stream >> (double &)d0;
          stream >> (double &)d1;
          stream >> (double &)d2;
          mFieldSetRequests << new WbVector3FieldSetRequest(field, index, d0, d1, d2);
          break;
        case WB_SF_ROTATION:
        case WB_MF_ROTATION:
          stream >> (double &)d0;
          stream >> (double &)d1;
          stream >> (double &)d2;
          stream >> (double &)d3;
          mFieldSetRequests << new WbRotationFieldSetRequest(field, index, d0, d1, d2, d3);
          break;
        case WB_SF_STRING:
        case WB_MF_STRING: {
          const QString s = readString(stream);
          mFieldSetRequests << new WbStringFieldSetRequest(field, index, s);
          break;
        }
      }
      return;
    }
    case C_SUPERVISOR_FIELD_INSERT_VALUE: {
      unsigned int nodeId, fieldId, index;

      stream >> (unsigned int &)nodeId;
      stream >> (unsigned int &)fieldId;
      stream >> (unsigned int &)index;

      WbNode *const node = WbNode::findNode(nodeId);
      WbField *field = node->field(fieldId);

      switch (field->type()) {  // import value
        case WB_MF_BOOL: {
          unsigned char value;
          stream >> (unsigned char &)value;
          (dynamic_cast<WbMFBool *>(field->value()))->insertItem(index, value == 1);
          break;
        }
        case WB_MF_INT32: {
          int value = 0;
          stream >> (int &)value;
          (dynamic_cast<WbMFInt *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_FLOAT: {
          double value = 0;
          stream >> (double &)value;
          (dynamic_cast<WbMFDouble *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_VEC2F: {
          double d0 = 0.0, d1 = 0.0;
          stream >> (double &)d0;
          stream >> (double &)d1;
          WbVector2 value(d0, d1);
          (dynamic_cast<WbMFVector2 *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_VEC3F: {
          double d0 = 0.0, d1 = 0.0, d2 = 0.0;
          stream >> (double &)d0;
          stream >> (double &)d1;
          stream >> (double &)d2;
          WbVector3 value(d0, d1, d2);
          (dynamic_cast<WbMFVector3 *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_ROTATION: {
          double d0 = 0.0, d1 = 0.0, d2 = 0.0, d3 = 0.0;
          stream >> (double &)d0;
          stream >> (double &)d1;
          stream >> (double &)d2;
          stream >> (double &)d3;
          WbRotation value(d0, d1, d2, d3);
          (dynamic_cast<WbMFRotation *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_COLOR: {
          double d0 = 0.0, d1 = 0.0, d2 = 0.0;
          stream >> (double &)d0;
          stream >> (double &)d1;
          stream >> (double &)d2;
          WbRgb value(d0, d1, d2);
          (dynamic_cast<WbMFColor *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_STRING: {
          const QString string = readString(stream);
          (dynamic_cast<WbMFString *>(field->value()))->insertItem(index, string);
          break;
        }
        case WB_MF_NODE: {
          QString filename = readString(stream);
          makeFilenameAbsolute(filename);
          int importedNodesNumber;
          WbNodeOperations::OperationResult operationResult;
          if (filename.endsWith(".wrl", Qt::CaseInsensitive))
            operationResult = WbNodeOperations::instance()->importVrml(filename, &importedNodesNumber, true);
          else if (filename.endsWith(".wbo", Qt::CaseInsensitive))
            operationResult =
              WbNodeOperations::instance()->importNode(nodeId, fieldId, index, filename, "", &importedNodesNumber, true);
          else {
            operationResult = WbNodeOperations::FAILURE;
            assert(false);
          }
          if (operationResult != WbNodeOperations::FAILURE)
            mImportedNodesNumber = importedNodesNumber;
          break;
        }
        default:
          assert(0);
      }

      emit worldModified();
      return;
    }
    case C_SUPERVISOR_FIELD_IMPORT_NODE_FROM_STRING: {
      unsigned int nodeId, fieldId, index;

      stream >> (unsigned int &)nodeId;
      stream >> (unsigned int &)fieldId;
      stream >> (unsigned int &)index;
      const QString nodeString = readString(stream);

      int importedNodesNumber;
      WbNodeOperations::OperationResult operationResult =
        WbNodeOperations::instance()->importNode(nodeId, fieldId, index, "", nodeString, &importedNodesNumber, true);
      if (operationResult != WbNodeOperations::FAILURE)
        mImportedNodesNumber = importedNodesNumber;
      emit worldModified();
      return;
    }
    case C_SUPERVISOR_NODE_REMOVE_NODE: {
      unsigned int nodeId;
      stream >> (unsigned int &)nodeId;
      WbNode *node = WbNode::findNode(nodeId);
      if (node) {
        WbNodeOperations::instance()->deleteNode(node, true);
        emit worldModified();
      }
      return;
    }
    case C_SUPERVISOR_FIELD_REMOVE_VALUE: {
      int index;
      unsigned int nodeId, fieldId;
      stream >> (unsigned int &)nodeId;
      stream >> (unsigned int &)fieldId;
      stream >> (int &)index;

      WbNode *parentNode = WbNode::findNode(nodeId);
      WbField *field = parentNode->field(fieldId);
      switch (field->type()) {  // remove value
        case WB_MF_BOOL:
        case WB_MF_INT32:
        case WB_MF_FLOAT:
        case WB_MF_VEC2F:
        case WB_MF_VEC3F:
        case WB_MF_ROTATION:
        case WB_MF_COLOR:
        case WB_MF_STRING: {
          WbMultipleValue *multipleValue = dynamic_cast<WbMultipleValue *>(field->value());
          assert(multipleValue->size() > index);
          multipleValue->removeItem(index);
          emit worldModified();
          break;
        }
        case WB_MF_NODE: {
          WbMFNode *mfNode = dynamic_cast<WbMFNode *>(field->value());
          assert(mfNode->size() > index);
          WbNode *node = mfNode->item(index);

          WbViewpoint *viewpoint = dynamic_cast<WbViewpoint *>(node);
          WbWorldInfo *worldInfo = dynamic_cast<WbWorldInfo *>(node);
          if (viewpoint || worldInfo) {
            node = NULL;
            mRobot->warn(tr(
              "wb_supervisor_field_remove_mf() called with the 'index' argument referring to a Viewpoint or WorldInfo node."));
          }

          if (node) {
            WbNodeOperations::instance()->deleteNode(node, true);
            emit worldModified();
          }
          break;
        }
        default:
          assert(0);
      }

      return;
    }
    case C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_IS_USED: {
      mVirtualRealityHeadsetIsUsedRequested = true;
      return;
    }
    case C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_POSITION: {
#ifdef _WIN32
      if (!WbVirtualRealityHeadset::isInUse())
#endif
        mRobot->warn(tr(
          "wb_supervisor_virtual_reality_headset_get_position() called but no virtual reality headset is currently in use."));
#ifdef _WIN32
      else if (WbVirtualRealityHeadset::instance()->isPositionTrackingEnabled())
        mVirtualRealityHeadsetPositionRequested = true;
#endif
      return;
    }
    case C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_ORIENTATION: {
#ifdef _WIN32
      if (!WbVirtualRealityHeadset::isInUse())
#endif
        mRobot->warn(
          tr("wb_supervisor_virtual_reality_headset_get_orientation() called but no virtual reality headset is currently in "
             "use."));
#ifdef _WIN32
      else if (WbVirtualRealityHeadset::instance()->isOrientationTrackingEnabled())
        mVirtualRealityHeadsetOrientationRequested = true;
#endif
      return;
    }
    default:
      assert(0);
  }
}

void WbSupervisorUtilities::writeNode(QDataStream &stream, const WbBaseNode *baseNode, int messageType) {
  assert(baseNode);
  stream << (int)baseNode->uniqueId();
  stream << (int)baseNode->nodeType();
  stream << (int)(baseNode->parent() ? baseNode->parent()->uniqueId() : -1);
  const QByteArray &modelName = baseNode->modelName().toUtf8();
  const QByteArray &defName = baseNode->defName().toUtf8();
  stream.writeRawData(modelName.constData(), modelName.size() + 1);
  stream.writeRawData(defName.constData(), defName.size() + 1);
  if (messageType == C_SUPERVISOR_FIELD_GET_VALUE)
    connect(baseNode, &WbNode::defUseNameChanged, this, &WbSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
}

void WbSupervisorUtilities::writeAnswer(QDataStream &stream) {
  if (!mUpdatedNodeIds.isEmpty()) {
    foreach (int id, mUpdatedNodeIds) {
      const WbBaseNode *baseNode = dynamic_cast<const WbBaseNode *>(WbNode::findNode(id));
      if (baseNode) {
        stream << (short unsigned int)0;
        stream << (unsigned char)C_SUPERVISOR_NODE_GET_FROM_ID;
        writeNode(stream, baseNode, C_SUPERVISOR_NODE_GET_FROM_ID);
      }
    }
    mUpdatedNodeIds.clear();
  }
  if (mGetFromId || mGetSelectedNode) {
    mGetFromId = false;
    stream << (short unsigned int)0;
    if (mGetSelectedNode) {
      mGetSelectedNode = false;
      stream << (unsigned char)C_SUPERVISOR_NODE_GET_SELECTED;
    } else
      stream << (unsigned char)C_SUPERVISOR_NODE_GET_FROM_ID;
    stream << (int)mFoundNodeUniqueId;
    stream << (int)mFoundNodeType;
    stream << (int)mFoundNodeParentUniqueId;
    const QByteArray &modelName = mFoundNodeModelName.toUtf8();
    const QByteArray &defName = mCurrentDefName.toUtf8();
    stream.writeRawData(modelName.constData(), modelName.size() + 1);
    stream.writeRawData(defName.constData(), defName.size() + 1);
    mFoundNodeUniqueId = -1;
    mCurrentDefName.clear();
  }
  if (mFoundNodeUniqueId != -1) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_FROM_DEF;
    stream << (int)mFoundNodeUniqueId;
    stream << (int)mFoundNodeType;
    stream << (int)mFoundNodeParentUniqueId;
    QByteArray s = mFoundNodeModelName.toUtf8();
    stream.writeRawData(s.constData(), s.size() + 1);
    mFoundNodeUniqueId = -1;
  }
  if (mFoundFieldId != -2) {  // enabled, -1 means not found
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_FIELD_GET_FROM_NAME;
    stream << (int)mFoundFieldId;
    stream << (int)mFoundFieldType;
    if (mFoundFieldCount != -1)
      stream << (int)mFoundFieldCount;
    mFoundFieldId = -2;
  }
  if (!mNodesDeletedSinceLastStep.isEmpty()) {
    for (int i = 0; i < mNodesDeletedSinceLastStep.size(); ++i) {
      struct WbDeletedNodeInfo deletedNodeInfo = mNodesDeletedSinceLastStep.at(i);

      stream << (short unsigned int)0;
      stream << (unsigned char)C_SUPERVISOR_NODE_REMOVE_NODE;
      stream << (int)deletedNodeInfo.nodeId;
      if (deletedNodeInfo.parent == WbWorld::instance()->root())
        stream << (int)0;
      else
        stream << (int)deletedNodeInfo.parent->uniqueId();
      QByteArray ba = deletedNodeInfo.parentField->name().toUtf8();
      stream.writeRawData(ba.constData(), ba.size() + 1);
      if (deletedNodeInfo.parentField->isMultiple())
        stream << (int)dynamic_cast<WbMultipleValue *>(deletedNodeInfo.parentField->value())->size();
      else
        stream << (int)-1;
    }
    mNodesDeletedSinceLastStep.clear();
  }
  if (mNodeGetPosition) {
    const WbVector3 &pos = mNodeGetPosition->matrix().translation();
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_POSITION;
    stream << (double)pos.x();
    stream << (double)pos.y();
    stream << (double)pos.z();
    mNodeGetPosition = NULL;
  }
  if (mNodeGetOrientation) {
    const WbMatrix4 &m = mNodeGetOrientation->matrix();
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_ORIENTATION;
    stream << (double)m(0, 0) << (double)m(0, 1) << (double)m(0, 2);
    stream << (double)m(1, 0) << (double)m(1, 1) << (double)m(1, 2);
    stream << (double)m(2, 0) << (double)m(2, 1) << (double)m(2, 2);
    mNodeGetOrientation = NULL;
  }
  if (mNodeGetCenterOfMass) {
    const WbVector3 &com = mNodeGetCenterOfMass->computedGlobalCenterOfMass();
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_CENTER_OF_MASS;
    stream << (double)com.x() << (double)com.y() << (double)com.z();
    mNodeGetCenterOfMass = NULL;
  }
  if (mNodeGetContactPoints) {
    const QVector<WbVector3> &contactPoints = mNodeGetContactPoints->computedContactPoints();
    const int size = contactPoints.size();
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_CONTACT_POINTS;
    stream << (int)size;
    for (int i = 0; i < size; ++i) {
      const WbVector3 &v = contactPoints.at(i);
      stream << (double)v.x();
      stream << (double)v.y();
      stream << (double)v.z();
    }
    mNodeGetContactPoints = NULL;
  }
  if (mNodeGetStaticBalance) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_STATIC_BALANCE;
    stream << (unsigned char)mNodeGetStaticBalance->staticBalance();
    mNodeGetStaticBalance = NULL;
  }
  if (mNodeGetVelocity) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_VELOCITY;
    WbVector3 linearVelocity = mNodeGetVelocity->relativeLinearVelocity();
    WbVector3 angularVelocity = mNodeGetVelocity->relativeAngularVelocity();
    stream << (double)linearVelocity[0];
    stream << (double)linearVelocity[1];
    stream << (double)linearVelocity[2];
    stream << (double)angularVelocity[0];
    stream << (double)angularVelocity[1];
    stream << (double)angularVelocity[2];
    mNodeGetVelocity = NULL;
  }
  if (mImportedNodesNumber >= 0) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_FIELD_INSERT_VALUE;
    stream << (int)mImportedNodesNumber;
    mImportedNodesNumber = -1;
  }
  if (mFieldGetRequest) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_FIELD_GET_VALUE;

    WbField *field = mFieldGetRequest->field;
    if (!field) {  // may happen if the object was deleted
      delete mFieldGetRequest;
      mFieldGetRequest = NULL;
      stream << (int)0;
      return;
    }
    stream << (int)field->type();
    switch (field->type()) {
      case WB_SF_BOOL: {
        bool v = dynamic_cast<WbSFBool *>(field->value())->value();
        stream << (unsigned char)v;
        break;
      }
      case WB_SF_INT32: {
        int v = dynamic_cast<WbSFInt *>(field->value())->value();
        stream << (int)v;
        break;
      }
      case WB_SF_FLOAT: {
        double v = dynamic_cast<WbSFDouble *>(field->value())->value();
        stream << (double)v;
        break;
      }
      case WB_SF_VEC2F: {
        const WbVector2 &v = dynamic_cast<WbSFVector2 *>(field->value())->value();
        stream << (double)v.x();
        stream << (double)v.y();
        break;
      }
      case WB_SF_VEC3F: {
        const WbVector3 &v = dynamic_cast<WbSFVector3 *>(field->value())->value();
        stream << (double)v.x();
        stream << (double)v.y();
        stream << (double)v.z();
        break;
      }
      case WB_SF_ROTATION: {
        const WbRotation &v = dynamic_cast<WbSFRotation *>(field->value())->value();
        stream << (double)v.x();
        stream << (double)v.y();
        stream << (double)v.z();
        stream << (double)v.angle();
        break;
      }
      case WB_SF_COLOR: {
        const WbRgb &v = dynamic_cast<WbSFColor *>(field->value())->value();
        stream << (double)v.red();
        stream << (double)v.green();
        stream << (double)v.blue();
        break;
      }
      case WB_SF_STRING: {
        const QString &v = dynamic_cast<WbSFString *>(field->value())->value();
        QByteArray ba = v.toUtf8();
        stream.writeRawData(ba.constData(), ba.size() + 1);
        break;
      }
      case WB_SF_NODE: {
        WbNode *const node = dynamic_cast<WbSFNode *>(field->value())->value();
        const WbBaseNode *const baseNode = dynamic_cast<WbBaseNode *>(node);
        if (baseNode)
          writeNode(stream, baseNode, C_SUPERVISOR_FIELD_GET_VALUE);
        else
          stream << (int)0;  // NULL node case
        break;
      }
      case WB_MF_BOOL: {
        const bool v = dynamic_cast<WbMFBool *>(field->value())->item(mFieldGetRequest->index);
        stream << (unsigned char)v;
        break;
      }
      case WB_MF_INT32: {
        const int v = dynamic_cast<WbMFInt *>(field->value())->item(mFieldGetRequest->index);
        stream << (int)v;
        break;
      }
      case WB_MF_FLOAT: {
        const double v = dynamic_cast<WbMFDouble *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v;
        break;
      }
      case WB_MF_VEC2F: {
        const WbVector2 &v = dynamic_cast<WbMFVector2 *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v.x();
        stream << (double)v.y();
        break;
      }
      case WB_MF_VEC3F: {
        const WbVector3 &v = dynamic_cast<WbMFVector3 *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v.x();
        stream << (double)v.y();
        stream << (double)v.z();
        break;
      }
      case WB_MF_COLOR: {
        const WbRgb &v = dynamic_cast<WbMFColor *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v.red();
        stream << (double)v.green();
        stream << (double)v.blue();
        break;
      }
      case WB_MF_ROTATION: {
        const WbRotation &v = dynamic_cast<WbMFRotation *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v.x();
        stream << (double)v.y();
        stream << (double)v.z();
        stream << (double)v.angle();
        break;
      }
      case WB_MF_STRING: {
        const QString &v = dynamic_cast<WbMFString *>(field->value())->item(mFieldGetRequest->index);
        QByteArray ba = v.toUtf8();
        stream.writeRawData(ba.constData(), ba.size() + 1);
        break;
      }
      case WB_MF_NODE: {
        const WbMFNode::WbNodePtr &v = dynamic_cast<WbMFNode *>(field->value())->item(mFieldGetRequest->index);
        WbNode *const node = dynamic_cast<WbNode *>(v);
        const WbBaseNode *const baseNode = dynamic_cast<WbBaseNode *>(node);
        if (baseNode)
          writeNode(stream, baseNode, C_SUPERVISOR_FIELD_GET_VALUE);
        else
          stream << (int)0;  // NULL node case
        break;
      }
      default:
        assert(0);
        break;
    }
    delete mFieldGetRequest;
    mFieldGetRequest = NULL;
  }
  if (mMovieStatus) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_MOVIE_STATUS;
    stream << (unsigned char)*mMovieStatus;
    delete mMovieStatus;
    mMovieStatus = NULL;
  }
  if (mAnimationStartStatus) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_ANIMATION_START_STATUS;
    stream << (unsigned char)*mAnimationStartStatus;
    delete mAnimationStartStatus;
    mAnimationStartStatus = NULL;
  }
  if (mAnimationStopStatus) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_ANIMATION_STOP_STATUS;
    stream << (unsigned char)*mAnimationStopStatus;
    delete mAnimationStopStatus;
    mAnimationStopStatus = NULL;
  }
  if (mSaveStatus) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_SAVE_WORLD;
    stream << (unsigned char)*mSaveStatus;
    delete mSaveStatus;
    mSaveStatus = NULL;
  }
  if (mVirtualRealityHeadsetIsUsedRequested) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_IS_USED;
#ifdef _WIN32
    stream << (unsigned char)WbVirtualRealityHeadset::isInUse();
#else
    stream << (unsigned char)0;
#endif
    mVirtualRealityHeadsetIsUsedRequested = false;
  }
  if (mVirtualRealityHeadsetPositionRequested) {
#ifdef _WIN32
    const WbVector3 position =
      WbVirtualRealityHeadset::isInUse() ? WbVirtualRealityHeadset::instance()->currentPosition() : WbVector3();
#else
    const WbVector3 position = WbVector3();
#endif
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_POSITION;
    for (int i = 0; i < 3; ++i)
      stream << (double)position[i];
    mVirtualRealityHeadsetPositionRequested = false;
  }
  if (mVirtualRealityHeadsetOrientationRequested) {
#ifdef _WIN32
    const WbMatrix3 orientation =
      WbVirtualRealityHeadset::isInUse() ? WbVirtualRealityHeadset::instance()->currentOrientation() : WbMatrix3();
#else
    const WbMatrix3 orientation = WbMatrix3();
#endif
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_ORIENTATION;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j)
        stream << (double)orientation(i, j);
    }
    mVirtualRealityHeadsetOrientationRequested = false;
  }
}

void WbSupervisorUtilities::writeConfigure(QDataStream &stream) {
  stream << (short unsigned int)0;
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mRobot->uniqueId();
  QByteArray s = mRobot->modelName().toUtf8();
  stream.writeRawData(s.constData(), s.size() + 1);
  QByteArray ba = mRobot->defName().toUtf8();
  stream.writeRawData(ba.constData(), ba.size() + 1);
}

void WbSupervisorUtilities::movieStatusChanged(int status) {
  mMovieStatus = new int[1];
  *mMovieStatus = status;
}

void WbSupervisorUtilities::animationStartStatusChanged(int status) {
  mAnimationStartStatus = new int[1];
  *mAnimationStartStatus = status;
}

void WbSupervisorUtilities::animationStopStatusChanged(int status) {
  mAnimationStopStatus = new int[1];
  *mAnimationStopStatus = status;
}

QStringList WbSupervisorUtilities::labelsState() const {
  QStringList labelsList;
  foreach (int labelId, mLabelIds) {
    const WbWrenLabelOverlay *labelOverlay = WbWrenLabelOverlay::retrieveById(labelId);
    if (labelOverlay)
      labelsList << createLabelUpdateString(labelOverlay);
  }
  return labelsList;
}

QString WbSupervisorUtilities::createLabelUpdateString(const WbWrenLabelOverlay *labelOverlay) const {
  assert(labelOverlay);
  float x, y, alpha;
  int r, g, b;
  labelOverlay->position(x, y);
  labelOverlay->color(r, g, b, alpha);
  return QString("label:%1;%2;rgba(%3,%4,%5,%6);%7;%8;%9;%10")
    .arg(labelOverlay->id())
    .arg(labelOverlay->font())
    .arg(r)
    .arg(g)
    .arg(b)
    .arg(alpha)
    .arg(labelOverlay->size())
    .arg(x)
    .arg(y)
    .arg(labelOverlay->text());
}
