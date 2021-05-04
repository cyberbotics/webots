// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "WbBaseNode.hpp"
#include "WbBasicJoint.hpp"
#include "WbBoundingSphere.hpp"
#include "WbDictionary.hpp"
#include "WbField.hpp"
#include "WbNodeOperations.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbTemplateManager.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"

#include <wren/scene.h>

void WbBaseNode::init() {
  mPreFinalizeCalled = false;
  mPostFinalizeCalled = false;
  mWrenObjectsCreatedCalled = false;
  mOdeObjectsCreatedCalled = false;
  mWrenNode = NULL;
  mIsInBoundingObject = false;
  mUpperTransform = NULL;
  mUpperSolid = NULL;
  mTopSolid = NULL;
  mBoundingObjectFirstTimeSearch = true;
  mUpperTransformFirstTimeSearch = true;
  mUpperSolidFirstTimeSearch = true;
  mTopSolidFirstTimeSearch = true;
  mFinalizationCanceled = false;
  mNodeUse = WbNode::UNKNOWN_USE;
  mNodeUseDirty = true;
}

WbBaseNode::WbBaseNode(const QString &modelName, WbTokenizer *tokenizer) :
  WbNode(modelName, WbWorld::instance() ? WbWorld::instance()->fileName() : "", tokenizer) {
  init();
}

WbBaseNode::WbBaseNode(const WbBaseNode &other) : WbNode(other) {
  init();
}

WbBaseNode::WbBaseNode(const WbNode &other) : WbNode(other) {
  init();
}

WbBaseNode::~WbBaseNode() {
  emit isBeingDestroyed(this);
  if (mPostFinalizeCalled && !defName().isEmpty() && !WbWorld::instance()->isCleaning() && !WbTemplateManager::isRegenerating())
    WbDictionary::instance()->removeNodeFromDictionary(this);
}

void WbBaseNode::finalize() {
  if (isProtoParameterNode()) {
    // finalize PROTO parameter node instances of the current node
    QVector<WbNode *> nodeInstances = protoParameterNodeInstances();
    WbBaseNode *baseNodeInstance = NULL;
    foreach (WbNode *nodeInstance, nodeInstances) {
      baseNodeInstance = dynamic_cast<WbBaseNode *>(nodeInstance);
      // recursive call to finalize nested parameter instances
      baseNodeInstance->finalize();
    }
    setFieldsParentNode();
    return;
  }

  WbWrenOpenGlContext::makeWrenCurrent();

  if (!isPreFinalizedCalled())
    preFinalize();

  if (!areOdeObjectsCreated() && (WbWorld::instance()->isLoading() || !WbNodeUtilities::isTrackAnimatedGeometry(this)))
    // in case of nodes descending from Track.animatedGeometries field we don't want to create ODE objects
    // these nodes are automatically skipped if a Track or ancestor node is finalized, so we only have to check in case of node
    // insertion
    createOdeObjects();

  if (!areWrenObjectsInitialized())
    createWrenObjects();

  if (mFinalizationCanceled) {
    WbWrenOpenGlContext::doneWren();
    return;
  }

  setFieldsParentNode();

  if (!isPostFinalizedCalled())
    postFinalize();

  validateProtoNodes();

  WbWrenOpenGlContext::doneWren();

  emit finalizationCompleted(this);
}

void WbBaseNode::postFinalize() {
  mPostFinalizeCalled = true;
  connect(this, &WbNode::defUseNameChanged, WbNodeOperations::instance(), &WbNodeOperations::requestUpdateSceneDictionary);
}

void WbBaseNode::validateProtoNodes() {
  QList<WbNode *> nodes = subNodes(true, false, false);
  nodes.prepend(this);

  foreach (WbNode *node, nodes) {
    if (node->isProtoInstance())
      dynamic_cast<WbBaseNode *>(node)->validateProtoNode();
  }
}
/*
void WbBaseNode::printChainCandidate(WbNode *node, int depth, bool end) {
  // This function is only for debug purposes, no need to review it
  if (node == NULL) {
    return;
  }
  if (!end) {
    if (depth > 0 && node->protoParameterNode() == NULL)
      printChainCandidate(node->parentNode(), depth + 1, true);
    else
      printChainCandidate(node->protoParameterNode(), depth + 1);
  }
  QString indent = "";
  for (int i = 0; i < depth; ++i) {
    indent += "  ";
  }
  QString type = "";
  if (node->isNestedProtoNode() && !node->isProtoParameterNode())
    type = "[N]";
  else if (!node->isNestedProtoNode() && node->isProtoParameterNode())
    type = "[P]";
  else if (node->isNestedProtoNode() && node->isProtoParameterNode())
    type = "[P/N]";
  else if (node->isInternalNode())
    type = "[I] ";
  if (end)
    printf("%s(%s %s (%p) -> (%p))\n", indent.toUtf8().constData(), type.toUtf8().constData(),
           node->usefulName().toUtf8().constData(), node, node->protoParameterNode());
  else
    printf("%s%s %s (%p) -> (%p)\n", indent.toUtf8().constData(), type.toUtf8().constData(),
           node->usefulName().toUtf8().constData(), node, node->protoParameterNode());
}

void WbBaseNode::printNodeFlags() {
  // This function is only for debug purposes, no need to review it
  QList<WbNode *> nodes = subNodes(true, true, true);

  printf("\nNODE FLAGS\n\n");
  for (int i = 0; i < nodes.size(); ++i) {
    printf("%d) %60s (%p) :  isVis %d / isVisOrVisField %d / isPPN %d / isNP %d / isPI %d / NN-PPN? %d / isDef %d\n", i,
           nodes[i]->usefulName().toUtf8().constData(), nodes[i], WbNodeUtilities::isVisible(nodes[i]),
           isInternalNodeVisible(nodes[i]), nodes[i]->isProtoParameterNode(), nodes[i]->isNestedProtoNode(),
           nodes[i]->isProtoInstance(), nodes[i]->protoParameterNode() != NULL, nodes[i]->isDefNode());
  }
}
*/
/*
void WbBaseNode::printNodeStructure() {
  // This function is only for debug purposes, no need to review it
  QList<WbNode *> nodes = subNodes(true, true, true);

  printf("=============================\n");
  root->printDebugNodeStructure();
  printf("=============================\n");
}

void WbBaseNode::printFieldsAndParams() {
  // This function is only for debug purposes, no need to review it
  QList<WbNode *> nodes = subNodes(true, true, true);

  for (int i = 0; i < nodes.size(); ++i) {
    printf("---------------------\n");
    printf("NODE %s (%p)\n", nodes[i]->usefulName().toUtf8().constData(), nodes[i]);
    QVector<WbField *> fieldsList = nodes[i]->fields();
    QVector<WbField *> parametersList = nodes[i]->parameters();

    printf("FIELDS\n");
    for (int i = 0; i < fieldsList.size(); ++i) {
      printf(" > field %s (%p (-> %p))\n", fieldsList[i]->name().toUtf8().constData(), fieldsList[i],
fieldsList[i]->parameter());
    }
    printf("PARAMETERS\n");
    for (int i = 0; i < parametersList.size(); ++i) {
      printf(" > %s (%p (-> %p))\n", parametersList[i]->name().toUtf8().constData(), parametersList[i],
             parametersList[i]->parameter());
    }
    printf("---------------------\n");
  }
}

void WbBaseNode::printNodeFieldVisibility() {
  // This function is only for debug purposes, no need to review it
  QList<WbNode *> nodes = subNodes(true, true, true);

  printf("\nNODE/FIELD VISIBILITY\n\n");
  for (int i = 0; i < nodes.size(); ++i) {
    printf("%40s visibility: %d\n", nodes[i]->usefulName().toUtf8().constData(), WbNodeUtilities::isVisible(nodes[i]));

    QVector<WbField *> fieldList = nodes[i]->fields();
    for (int j = 0; j < fieldList.size(); ++j) {
      printf("  %40s (field) visibility: %d\n", fieldList[j]->name().toUtf8().constData(),
             WbNodeUtilities::isVisible(fieldList[j]));
    }

    QVector<WbField *> parameterList = nodes[i]->parameters();
    for (int j = 0; j < parameterList.size(); ++j) {
      printf("  %40s (param) visibility: %d\n", parameterList[j]->name().toUtf8().constData(),
             WbNodeUtilities::isVisible(parameterList[j]));
    }
  }
}
*/

bool WbBaseNode::isInternalNodeVisible(WbNode *internal) const {
  // reach the highest parameter node in the chain, there can be multiple in a heavily nested PROTO
  const WbNode *n = internal;
  while (n && n->protoParameterNode() != NULL)
    n = n->protoParameterNode();
  // check if the parameter node itself is visible
  if (WbNodeUtilities::isVisible(n))
    return true;
  // or if it exposes any visible parameter. It's possible for it to expose a single field without exposing the parameter
  // (usually the case when SFNodes are involved) so the test is made on the fields instead
  const QVector<WbField *> fields = n->fields();
  for (int i = 0; i < fields.size(); ++i)
    if (WbNodeUtilities::isVisible(fields[i]))
      return true;

  return false;
}

void WbBaseNode::removeInvisibleProtoNodes() {
  // printNodeStructure();        // TODO: remove before merge
  // printNodeFlags();            // TODO: remove before merge
  // printFieldsAndParams();      // TODO: remove before merge
  // printNodeFieldVisibility();  // TODO: remove before merge

  // when loading, root is the global root. When regenerating, root is the finalized node after the regeneration process
  const QList<WbNode *> nodes = subNodes(true, true, true);

  // the internal node is used to keep track of what can be collapsed since it's the bottom of the chain and it's unique
  // whereas the chain itself can be comprised of multiple parameter nodes which complicates keeping track of how they relate
  QList<WbNode *> internalProtoNodes;

  for (int i = 0; i < nodes.size(); ++i)
    if (nodes[i]->isInternalNode())
      internalProtoNodes.append(nodes[i]);
  /*
  // TODO: remove before merge
  printf("PRINT CHAINS FOR UNFILTERED CANDIDATES\n");
  for (int i = 0; i < internalProtoNodes.size(); ++i) {
    printf("\n");
    printChainCandidate(internalProtoNodes[i]);
  }
  */

  QList<WbNode *> tmp = internalProtoNodes;
  for (int i = 0; i < internalProtoNodes.size(); ++i) {
    if (isInternalNodeVisible(internalProtoNodes[i])) {
      // cannot collapse visible nodes otherwise they no longer refresh on the interface
      tmp.removeAll(internalProtoNodes[i]);
      // also remove among the candidates any ancestor to this node otherwise it will be deleted indirectly.
      // likewise any descendants can't be deleted either as they might be referenced indirectly (e.g if the texture url field
      // is visible, the corresponding TextureCoordinate/IndexedFaceSet nodes can't be deleted even if themselves aren't)
      for (int j = 0; j < internalProtoNodes.size(); ++j)
        if (internalProtoNodes[j]->isAnAncestorOf(internalProtoNodes[i]) ||
            internalProtoNodes[i]->isAnAncestorOf(internalProtoNodes[j]))
          tmp.removeAll(internalProtoNodes[j]);
    }
  }
  internalProtoNodes = tmp;

  QList<WbNode *> invisibleProtoParameterNodes;
  // follow the chain upwards, starting from the internal node, to extract all the protoParameterNodes that can be deleted
  for (int i = 0; i < internalProtoNodes.size(); ++i) {
    WbNode *n = internalProtoNodes[i]->protoParameterNode();

    while (n != NULL) {
      bool added = false;
      for (int j = 0; j < invisibleProtoParameterNodes.size(); ++j)
        if (n->level() > invisibleProtoParameterNodes[j]->level()) {  // insert them from lowest to highest level
          invisibleProtoParameterNodes.insert(j, n);
          added = true;
          break;  // need to break otherwise invisibleProtoParameterNodes grows infinitly
        }

      if (!added)
        invisibleProtoParameterNodes.append(n);

      n = n->protoParameterNode();
    }
  }

  // TODO: remove before merge
  printf("\nINVISIBLE PROTO PARAMETER NODES (WHAT WILL BE REMOVED)\n");
  for (int i = 0; i < invisibleProtoParameterNodes.size(); ++i) {
    printf("  [L%d] %s [%p]\n", invisibleProtoParameterNodes[i]->level(),
           invisibleProtoParameterNodes[i]->usefulName().toUtf8().constData(), invisibleProtoParameterNodes[i]);
  }

  if (invisibleProtoParameterNodes.size() == 0)
    return;

  // break link between [field] -> [parameter] and [internal node] -> [parameter node] (from internal node side)
  for (int i = 0; i < internalProtoNodes.size(); ++i) {
    internalProtoNodes[i]->disconnectInternalNode();
    const QVector<WbField *> fields = internalProtoNodes[i]->fields();

    for (int j = 0; j < fields.size(); j++)
      fields[j]->setParameter(NULL);

    internalProtoNodes[i]->setProtoParameterNode(NULL);  // break link with proto parameter node
  }

  // break link [parameter] -> [internal field] and [parameter node] -> [internal node] (from parameter node side)
  for (int i = 0; i < invisibleProtoParameterNodes.size(); ++i) {
    invisibleProtoParameterNodes[i]->clearProtoParameterNodeInstances();  // clear downward references

    // clear internal field references (for protoParameterNodes the reference is kept in its fields)
    QVector<WbField *> fields = invisibleProtoParameterNodes[i]->fields();
    for (int i = 0; i < fields.size(); ++i)
      fields[i]->clearInternalFields();
  }

  // now the proto parameter nodes can be deleted, depending on the situation it can either be in the parameter or field side of
  // the parent node. The signal is not emitted to prevent the internal node from being deleted as well in the process
  for (int i = 0; i < invisibleProtoParameterNodes.size(); ++i) {
    WbNode *parameterNode = invisibleProtoParameterNodes[i];
    WbNode *parent = parameterNode->parentNode();

    const QVector<WbField *> fields = parent->fields();
    for (int j = 0; j < fields.size(); ++j) {
      WbSFNode *sfnode = dynamic_cast<WbSFNode *>(fields[j]->value());
      WbMFNode *mfnode = dynamic_cast<WbMFNode *>(fields[j]->value());
      if (sfnode && sfnode->value() == parameterNode) {
        sfnode->blockSignals(true);
        sfnode->setValue(NULL);
        sfnode->blockSignals(false);
        parent->removeFromFieldsOrParameters(fields[j]);
      } else {
        if (mfnode && mfnode->nodeIndex(parameterNode) != -1) {
          mfnode->blockSignals(true);
          mfnode->removeNode(parameterNode);
          mfnode->blockSignals(false);
          parent->removeFromFieldsOrParameters(fields[j]);
        }
      }
    }

    const QVector<WbField *> parameters = parent->parameters();
    for (int j = 0; j < parameters.size(); j++) {
      WbSFNode *sfnode = dynamic_cast<WbSFNode *>(parameters[j]->value());
      WbMFNode *mfnode = dynamic_cast<WbMFNode *>(parameters[j]->value());

      if (sfnode && sfnode->value() == parameterNode) {
        sfnode->blockSignals(true);
        sfnode->setValue(NULL);
        sfnode->blockSignals(false);
        parent->removeFromFieldsOrParameters(parameters[j]);
      } else {
        if (mfnode && mfnode->nodeIndex(parameterNode) != -1) {
          mfnode->blockSignals(true);
          mfnode->removeNode(parameterNode);
          mfnode->blockSignals(false);
          parent->removeFromFieldsOrParameters(parameters[j]);
        }
      }
    }
  }

  // printNodeStructure(root);  // TODO: remove before merge
}

void WbBaseNode::reset(const QString &id) {
  WbNode::reset(id);
  WbBoundingSphere *const nodeBoundingSphere = boundingSphere();
  if (nodeBoundingSphere)
    nodeBoundingSphere->resetGlobalCoordinatesUpdateTime();
}

//////////////////////////
// WREN and ODE objects //
//////////////////////////

void WbBaseNode::createWrenObjects() {
  mWrenObjectsCreatedCalled = true;

  if (parentNode()) {
    const WbBaseNode *const p = static_cast<WbBaseNode *>(parentNode());
    mWrenNode = p->wrenNode();
  } else
    mWrenNode = wr_scene_get_root(wr_scene_get_instance());
}

void WbBaseNode::updateContextDependentObjects() {
  if (isProtoParameterNode()) {
    // update context of PROTO parameter node instances
    // this has no WREN objects to be updated
    QVector<WbNode *> nodeInstances = protoParameterNodeInstances();
    foreach (WbNode *nodeInstance, nodeInstances) { static_cast<WbBaseNode *>(nodeInstance)->updateContextDependentObjects(); }

  } else {
    QList<WbNode *> sbn = subNodes(false);
    foreach (WbNode *node, sbn)
      static_cast<WbBaseNode *>(node)->updateContextDependentObjects();
  }
}

// Utility functions
bool WbBaseNode::isInBoundingObject() const {
  if (mBoundingObjectFirstTimeSearch) {
    mIsInBoundingObject = WbNodeUtilities::isInBoundingObject(this);
    if (areWrenObjectsInitialized())
      mBoundingObjectFirstTimeSearch = false;
  }

  return mIsInBoundingObject;
}

WbNode::NodeUse WbBaseNode::nodeUse() const {
  if (mNodeUseDirty) {
    mNodeUse = WbNodeUtilities::checkNodeUse(this);
    if (areWrenObjectsInitialized())
      mNodeUseDirty = false;
  }

  return mNodeUse;
}

WbTransform *WbBaseNode::upperTransform() const {
  if (mUpperTransformFirstTimeSearch) {
    mUpperTransform = WbNodeUtilities::findUpperTransform(this);
    if (areWrenObjectsInitialized())
      mUpperTransformFirstTimeSearch = false;
  }

  return mUpperTransform;
}

WbSolid *WbBaseNode::upperSolid() const {
  if (mUpperSolidFirstTimeSearch) {
    mUpperSolid = WbNodeUtilities::findUpperSolid(this);
    if (areWrenObjectsInitialized())
      mUpperSolidFirstTimeSearch = false;
  }

  return mUpperSolid;
}

WbSolid *WbBaseNode::topSolid() const {
  if (mTopSolidFirstTimeSearch) {
    mTopSolid = WbNodeUtilities::findTopSolid(this);
    if (areWrenObjectsInitialized())
      mTopSolidFirstTimeSearch = false;
  }

  return mTopSolid;
}

WbBaseNode *WbBaseNode::getSingleFinalizedProtoInstance() {
  if (!isProtoParameterNode())
    return NULL;

  WbBaseNode *finalizedInstance = NULL;
  QVector<WbNode *> nodeInstances = protoParameterNodeInstances();
  foreach (WbNode *node, nodeInstances) {
    WbBaseNode *baseNode = dynamic_cast<WbBaseNode *>(node);
    if (baseNode && baseNode->isPostFinalizedCalled()) {
      // cppcheck-suppress knownConditionTrueFalse
      if (finalizedInstance)
        // multiple finilized instances found
        return NULL;

      finalizedInstance = baseNode;
    }
  }

  return finalizedInstance;
}

bool WbBaseNode::isInvisibleNode() const {
  return WbWorld::instance()->viewpoint()->getInvisibleNodes().contains(const_cast<WbBaseNode *>(this));
}

QString WbBaseNode::documentationUrl() const {
  QStringList bookAndPage = documentationBookAndPage(WbNodeUtilities::isRobotTypeName(nodeModelName()));
  if (!bookAndPage.isEmpty())
    return QString("%1/doc/%2/%3").arg(WbStandardPaths::cyberboticsUrl()).arg(bookAndPage[0]).arg(bookAndPage[1]);
  return QString();
}

bool WbBaseNode::exportNodeHeader(WbVrmlWriter &writer) const {
  if (!writer.isX3d())
    return WbNode::exportNodeHeader(writer);

  writer << "<" << x3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
  if (isInvisibleNode())
    writer << " render=\'false\'";
  QStringList bookAndPage = documentationBookAndPage(WbNodeUtilities::isRobotTypeName(nodeModelName()));
  if (!bookAndPage.isEmpty())
    writer
      << QString(" docUrl=\'%1/doc/%2/%3\'").arg(WbStandardPaths::cyberboticsUrl()).arg(bookAndPage[0]).arg(bookAndPage[1]);

  if (isUseNode() && defNode()) {  // export referred DEF node id
    const WbNode *def = defNode();
    while (def && def->isProtoParameterNode()) {
      const QVector<WbNode *> nodeInstances = def->protoParameterNodeInstances();
      def = nodeInstances.isEmpty() ? NULL : nodeInstances.at(0);
    }
    assert(def != NULL);
    writer << " USE=\'n" + QString::number(def->uniqueId()) + "\'></" + x3dName() + ">";
    return true;
  }
  return false;
}

bool WbBaseNode::isUrdfRootLink() const {
  if (findSFString("name") || dynamic_cast<WbBasicJoint *>(parentNode()))
    return true;
  return false;
}

void WbBaseNode::exportURDFJoint(WbVrmlWriter &writer) const {
  if (!dynamic_cast<WbBasicJoint *>(parentNode())) {
    WbVector3 translation;
    WbVector3 rotationEuler;
    const WbNode *const upperLinkRoot = findUrdfLinkRoot();

    if (dynamic_cast<const WbTransform *>(this) && dynamic_cast<const WbTransform *>(upperLinkRoot)) {
      const WbTransform *const upperLinkRootTransform = static_cast<const WbTransform *>(this);
      translation = upperLinkRootTransform->translationFrom(upperLinkRoot);
      rotationEuler = upperLinkRootTransform->rotationMatrixFrom(upperLinkRoot).toEulerAnglesZYX();
    }

    translation += writer.jointOffset();
    writer.setJointOffset(WbVector3(0.0, 0.0, 0.0));

    writer.increaseIndent();
    writer.indent();
    writer << QString("<joint name=\"%1_%2_joint\" type=\"fixed\">\n").arg(upperLinkRoot->urdfName()).arg(urdfName());

    writer.increaseIndent();
    writer.indent();
    writer << QString("<parent link=\"%1\"/>\n").arg(upperLinkRoot->urdfName());
    writer.indent();
    writer << QString("<child link=\"%1\"/>\n").arg(urdfName());
    writer.indent();
    writer << QString("<origin xyz=\"%1\" rpy=\"%2\"/>\n")
                .arg(translation.toString(WbPrecision::FLOAT_MAX))
                .arg(rotationEuler.toString(WbPrecision::FLOAT_MAX));
    writer.decreaseIndent();

    writer.indent();
    writer << "</joint>\n";
    writer.decreaseIndent();
  }
}
