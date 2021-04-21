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

#include "WbTemplateManager.hpp"

#include "WbAppearance.hpp"
#include "WbBasicJoint.hpp"
#include "WbField.hpp"
#include "WbFieldModel.hpp"
#include "WbGeometry.hpp"
#include "WbGroup.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbNode.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPbrAppearance.hpp"
#include "WbProtoModel.hpp"
#include "WbSFNode.hpp"
#include "WbShape.hpp"
#include "WbSkin.hpp"
#include "WbSlot.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"

#include <QtCore/QCoreApplication>

#include <cassert>

WbTemplateManager *WbTemplateManager::cInstance = NULL;
int WbTemplateManager::cRegeneratingNodeCount = 0;

WbTemplateManager *WbTemplateManager::instance() {
  if (!cInstance) {
    cInstance = new WbTemplateManager();
    qAddPostRoutine(WbTemplateManager::cleanup);
  }
  return cInstance;
}

void WbTemplateManager::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

WbTemplateManager::WbTemplateManager() : mBlockRegeneration(false), mTemplatesNeedRegeneration(false) {
}

WbTemplateManager::~WbTemplateManager() {
  clear();
}

void WbTemplateManager::blockRegeneration(bool block) {
  mBlockRegeneration = block;

  if (!block && mTemplatesNeedRegeneration) {  // regenerates all the required nodes
    while (true) {
      bool regenerated = false;
      foreach (WbNode *node, mTemplates) {
        if (node->isRegenerationRequired()) {
          regenerateNode(node);  // mTemplates can be modified during this call
          regenerated = true;
          break;
        }
      }
      if (regenerated)
        continue;
      else
        break;
    }
    mTemplatesNeedRegeneration = false;
  }
}

void WbTemplateManager::clear() {
  foreach (WbNode *node, mTemplates)
    disconnect(node, &WbNode::regenerationRequired, this, &WbTemplateManager::nodeNeedRegeneration);
  mTemplates.clear();
}

void WbTemplateManager::subscribe(WbNode *node, bool subscribedDescendant) {
  bool subscribed = false;
  if (node->isTemplate() && !mTemplates.contains(node)) {
    subscribed = true;
    mTemplates << node;
    connect(node, &QObject::destroyed, this, &WbTemplateManager::unsubscribe, Qt::UniqueConnection);
    connect(node, &WbNode::regenerateNodeRequest, this, &WbTemplateManager::regenerateNode, Qt::UniqueConnection);
    connect(node, &WbNode::regenerationRequired, this, &WbTemplateManager::nodeNeedRegeneration);
  }

  recursiveFieldSubscribeToRegenerateNode(node, subscribed, subscribedDescendant);
}

void WbTemplateManager::unsubscribe(QObject *node) {
  disconnect(static_cast<WbNode *>(node), &WbNode::regenerationRequired, this, &WbTemplateManager::nodeNeedRegeneration);
  mTemplates.removeAll(static_cast<WbNode *>(node));
}

bool WbTemplateManager::nodeNeedsToSubscribe(WbNode *node) {
  if (!node->isProtoInstance())
    return false;

  QVector<WbField *> fields = node->fieldsOrParameters();
  foreach (WbField *field, fields) {
    if (!field->alias().isEmpty())
      return true;
  }
  return false;
}

void WbTemplateManager::recursiveFieldSubscribeToRegenerateNode(WbNode *node, bool subscribedNode, bool subscribedDescendant) {
  if (subscribedNode || subscribedDescendant) {
    if (node->isProtoInstance())
      connect(node, &WbNode::parameterChanged, this, &WbTemplateManager::regenerateNodeFromParameterChange,
              Qt::UniqueConnection);
    else
      connect(node, &WbNode::fieldChanged, this, &WbTemplateManager::regenerateNodeFromFieldChange, Qt::UniqueConnection);
  }

  // if PROTO node:
  //   - subscribe sub-nodes in parameters
  //   - subscribe sub-nodes in fields if a parameter is redirected to the sub-node
  // else normal nodes:
  //   - subscribe sub-nodes in fields
  QVector<WbField *> fields = node->fields();
  int directSubscribeMinIndex = 0;
  if (node->isProtoInstance()) {
    directSubscribeMinIndex = fields.size();
    fields.append(node->parameters());
  }
  WbField *field = NULL;
  for (int i = 0; i < fields.size(); ++i) {
    field = fields[i];
    bool directSubscriptionEnabled = i >= directSubscribeMinIndex;
    switch (field->type()) {
      case WB_MF_NODE: {
        WbMFNode *mfnode = static_cast<WbMFNode *>(field->value());
        assert(mfnode);
        for (int j = 0; j < mfnode->size(); j++) {
          WbNode *subnode = mfnode->item(j);
          if (directSubscriptionEnabled || nodeNeedsToSubscribe(subnode))
            subscribe(subnode, subscribedDescendant || (subscribedNode && field->isTemplateRegenerator()));
        }
        break;
      }
      case WB_SF_NODE: {
        WbSFNode *sfnode = static_cast<WbSFNode *>(field->value());
        assert(sfnode);
        WbNode *subnode = sfnode->value();
        if (subnode && (directSubscriptionEnabled || nodeNeedsToSubscribe(subnode)))
          subscribe(subnode, subscribedDescendant || (subscribedNode && field->isTemplateRegenerator()));
        break;
      }
      default:
        break;
    }
  }
}

void WbTemplateManager::regenerateNodeFromFieldChange(WbField *field) {
  printf("regenerateNodeFromFieldChange\n");
  // retrieve the right node
  WbNode *templateNode = dynamic_cast<WbNode *>(sender());
  assert(templateNode);
  if (templateNode)
    regenerateNodeFromField(templateNode, field, false);
}

void WbTemplateManager::regenerateNodeFromParameterChange(WbField *field) {
  printf("regenerateNodeFromParameterChange\n");
  // retrieve the right node
  WbNode *templateNode = dynamic_cast<WbNode *>(sender());
  assert(templateNode);
  if (templateNode)
    regenerateNodeFromField(templateNode, field, true);
}

// intermediate function to determine which node should be updated
// Note: The security is probably overkill there, but its also safer for the first versions of the template mechanism
void WbTemplateManager::regenerateNodeFromField(WbNode *templateNode, WbField *field, bool isParameter) {
  printf("regenerateNodeFromField\n");
  // 1. retrieve upper template node where the modification appeared in a template regenerator field
  templateNode = WbNodeUtilities::findUpperTemplateNeedingRegenerationFromField(field, templateNode);

  if (!templateNode)
    return;

  // 2. check it's not a parameter managed by ODE
  if (!isParameter && dynamic_cast<const WbSolid *>(templateNode) &&
      ((field->name() == "translation" && field->type() == WB_SF_VEC3F) ||
       (field->name() == "rotation" && field->type() == WB_SF_ROTATION) ||
       (field->name() == "position" && field->type() == WB_SF_FLOAT)))
    return;

  // 3. regenerate template where the modification appeared in a template regenerator field
  regenerateNode(templateNode);
}

bool WbTemplateManager::isVisibleOrHasVisibleFields(WbNode *node) {
  // reach the highest parameter field in the chain
  const WbNode *n = node;
  while (n && n->protoParameterNode()) {
    n = n->protoParameterNode();
  }

  // check the tip itself is visible
  if (WbNodeUtilities::isVisible(n))
    return true;

  // check if any of the fields are visible
  QVector<WbField *> fields = n->fields();
  for (int i = 0; i < fields.size(); ++i) {
    if (WbNodeUtilities::isVisible(fields[i]))
      return true;
  }

  return false;
}

void WbTemplateManager::printChainCandidate(WbNode *node, int depth, bool end) {
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

void WbTemplateManager::printNodeFlags(WbNode *root) {
  QList<WbNode *> nodes = root->subNodes(true, true, true);

  printf("\nNODE FLAGS\n\n");
  for (int i = 0; i < nodes.size(); ++i) {
    printf("%d) %60s (%p) :  isVis %d / isVisOrVisField %d / isPPN %d / isNP %d / isPI %d / NN-PPN? %d / isDef %d\n", i,
           nodes[i]->usefulName().toUtf8().constData(), nodes[i], WbNodeUtilities::isVisible(nodes[i]),
           isVisibleOrHasVisibleFields(nodes[i]), nodes[i]->isProtoParameterNode(), nodes[i]->isNestedProtoNode(),
           nodes[i]->isProtoInstance(), nodes[i]->protoParameterNode() != NULL, nodes[i]->isDefNode());
  }
}

void WbTemplateManager::printNodeStructure(WbNode *root) {
  QList<WbNode *> nodes = root->subNodes(true, true, true);

  printf("=============================\n");
  root->printDebugNodeStructure();
  printf("=============================\n");
}

void WbTemplateManager::printFieldsAndParams(WbNode *root) {
  QList<WbNode *> nodes = root->subNodes(true, true, true);

  for (int i = 0; i < nodes.size(); ++i) {
    nodes[i]->printFieldsAndParams();
  }
}

void WbTemplateManager::printNodeFieldVisibility(WbNode *root) {
  QList<WbNode *> nodes = root->subNodes(true, true, true);

  printf("\nNODE/FIELD VISIBILITY\n\n");
  for (int i = 0; i < nodes.size(); ++i) {
    printf("%40s visibility: %d\n", nodes[i]->usefulName().toUtf8().constData(), WbNodeUtilities::isVisible(nodes[i]));
    QVector<WbField *> parameterList = nodes[i]->fieldsOrParameters();
    for (int j = 0; j < parameterList.size(); ++j) {
      printf("  %40s visibility: %d\n", parameterList[j]->name().toUtf8().constData(),
             WbNodeUtilities::isVisible(parameterList[j]));
    }
  }
}

void WbTemplateManager::removeInvisibleProtoNodes(WbNode *root) {
  // printNodeStructure(root);

  // printNodeFlags(root);

  // printFieldsAndParams(root);

  // printNodeFieldVisibility(root);

  QList<WbNode *> nodes = root->subNodes(true, true, true);

  // the internal node is used to keep track of what can be collapsed since it's the bottom of the chain and they're unique
  // whereas the chain itself can be comprised of multiple parameter nodes which complicates keeping track of how they relate
  QList<WbNode *> internalProtoNodes;

  for (int i = 0; i < nodes.size(); ++i)
    if (nodes[i]->isInternalNode())
      internalProtoNodes.append(nodes[i]);

  /*
  printf("PRINT CHAINS FOR UNFILTERED CANDIDATES\n");
  for (int i = 0; i < internalProtoNodes.size(); ++i) {
    printf("\n");
    printChainCandidate(internalProtoNodes[i]);
  }
  */

  QList<WbNode *> tmp = internalProtoNodes;

  for (int i = 0; i < internalProtoNodes.size(); ++i) {
    if (isVisibleOrHasVisibleFields(internalProtoNodes[i])) {
      // cant collapse visible ones, so remove them
      tmp.removeOne(internalProtoNodes[i]);

      // also remove any ancestor to it or any node this is an ancestor to
      for (int j = 0; j < internalProtoNodes.size(); ++j) {
        if (internalProtoNodes[j]->isAnAncestorOf(internalProtoNodes[i]) ||
            internalProtoNodes[i]->isAnAncestorOf(internalProtoNodes[j]))
          tmp.removeOne(internalProtoNodes[j]);
      }
    }
  }

  internalProtoNodes = tmp;

  QList<WbNode *> invisibleProtoParameterNodes;  // proto parameter nodes that can be removed

  // follow the chain up from each internal node to extract all the protoParameterNodes associated with it
  for (int i = 0; i < internalProtoNodes.size(); ++i) {
    WbNode *n = internalProtoNodes[i]->protoParameterNode();

    while (n != NULL) {
      bool added = false;
      for (int j = 0; j < invisibleProtoParameterNodes.size(); ++j) {
        if (n->level() > invisibleProtoParameterNodes[j]->level()) {  // insert them from lowest to highest level
          invisibleProtoParameterNodes.insert(j, n);
          added = true;
          break;  // need to break otherwise invisibleProtoParameterNodes grows infinitly
        }
      }

      if (!added)
        invisibleProtoParameterNodes.append(n);

      n = n->protoParameterNode();
    }
  }
  /*
  printf("INVISIBLE PROTO PARAMETER NODES (WHAT WILL BE REMOVED)\n");
  for (int i = 0; i < invisibleProtoParameterNodes.size(); ++i) {
    printf("  [L%d] %s [%p]\n", invisibleProtoParameterNodes[i]->level(),
           invisibleProtoParameterNodes[i]->usefulName().toUtf8().constData(), invisibleProtoParameterNodes[i]);
  }
  */
  if (invisibleProtoParameterNodes.size() == 0)
    return;

  // break link between [field] -> [parameter] and [internal node] -> [parameter node] (from internal node side)
  for (int i = 0; i < internalProtoNodes.size(); ++i) {
    QVector<WbField *> fields = internalProtoNodes[i]->fields();

    for (int j = 0; j < fields.size(); j++)
      fields[j]->setParameter(NULL);

    // break link with proto parameter node
    internalProtoNodes[i]->setProtoParameterNode(NULL);
  }

  // break link [parameter] -> [internal field] and [parameter node] -> [internal node] (from parameter node side)
  for (int i = 0; i < invisibleProtoParameterNodes.size(); ++i) {
    // clear downward references
    invisibleProtoParameterNodes[i]->clearProtoParameterNodeInstances();

    // clear internal field references
    QVector<WbField *> fields =
      invisibleProtoParameterNodes[i]->fields();  // for protoParameterNodes, reference is kept in its fields
    for (int i = 0; i < fields.size(); ++i)
      fields[i]->clearInternalFields();
  }

  QVector<WbNode *> todel;

  for (int i = 0; i < invisibleProtoParameterNodes.size(); ++i) {
    WbNode *parameterNode = invisibleProtoParameterNodes[i];
    WbNode *parent = parameterNode->parentNode();

    QVector<WbField *> fieldsOrParameters = parent->parameters();

    for (int j = 0; j < fieldsOrParameters.size(); j++) {
      WbSFNode *sfnode = dynamic_cast<WbSFNode *>(fieldsOrParameters[j]->value());
      WbMFNode *mfnode = dynamic_cast<WbMFNode *>(fieldsOrParameters[j]->value());

      if (sfnode && sfnode->value() == parameterNode) {
        sfnode->setValueNoSignal(NULL);
        parent->removeFromFieldsOrParameters(fieldsOrParameters[j]);
      } else {
        if (mfnode && mfnode->nodeIndex(parameterNode) != -1) {
          mfnode->removeNodeNoSignal(parameterNode);
          parent->removeFromFieldsOrParameters(fieldsOrParameters[j]);
        }
      }
    }

    fieldsOrParameters = parent->fields();
    for (int j = 0; j < fieldsOrParameters.size(); j++) {
      WbSFNode *sfnode = dynamic_cast<WbSFNode *>(fieldsOrParameters[j]->value());
      WbMFNode *mfnode = dynamic_cast<WbMFNode *>(fieldsOrParameters[j]->value());

      if (sfnode && sfnode->value() == parameterNode) {
        sfnode->setValueNoSignal(NULL);
        parent->removeFromFieldsOrParameters(fieldsOrParameters[j]);
      } else {
        if (mfnode && mfnode->nodeIndex(parameterNode) != -1) {
          mfnode->removeNodeNoSignal(parameterNode);
          parent->removeFromFieldsOrParameters(fieldsOrParameters[j]);
        }
      }
    }
    printf("will delete (%s) [%p]\n", parameterNode->usefulName().toUtf8().constData(), parameterNode);
    todel.append(parameterNode);
  }

  /*
  for (int i = 1; i < todel.size(); i++) {
    if (todel[i] == todel[i - 1])
      continue;
    else {
      printf(">> del (%s) [%p]\n", todel[i - 1]->usefulName().toUtf8().constData(), todel[i - 1]);
      delete todel[i - 1];

      if (i == todel.size() - 1) {
        printf(">> del (%s) [%p]\n", todel[i]->usefulName().toUtf8().constData(), todel[i]);
        delete todel[i];
      }
    }
  }
  */

  // printNodeStructure(root);
}

void WbTemplateManager::regenerateNode(WbNode *node, bool restarted) {
  printf("regenerateNode %s\n", node->usefulName().toUtf8().constData());
  assert(node);

  if (mBlockRegeneration) {
    node->setRegenerationRequired(true);  // will be regenerated when deblocking this manager
    return;
  } else
    node->setRegenerationRequired(false);

  // 1. get stuff
  WbNode *parent = node->parentNode();
  WbProtoModel *proto = node->proto();
  assert(parent && proto);
  if (!parent || !proto)
    return;
  const bool isInBoundingObject = dynamic_cast<WbBaseNode *>(node)->isInBoundingObject();

  QList<WbField *> previousParentRedirections;
  WbField *parentField = node->parentField();
  QVector<WbField *> parameters;
  WbNode::setRestoreUniqueIdOnClone(true);
  foreach (WbField *parameter, node->parameters()) {
    parameters << new WbField(*parameter, NULL);
    if (parameter->parameter() != NULL)
      previousParentRedirections.append(parameter->parameter());
  }
  WbNode::setRestoreUniqueIdOnClone(false);
  int uniqueId = node->uniqueId();
  const WbSolid *solid = dynamic_cast<const WbSolid *>(node);
  WbVector3 translationFromFile;
  WbRotation rotationFromFile;
  if (solid) {
    translationFromFile = solid->translationFromFile();
    rotationFromFile = solid->rotationFromFile();
  }

  WbWorld *world = WbWorld::instance();
  WbGroup *root = WbWorld::instance()->root();
  bool isWorldInitialized = root && root->isPostFinalizedCalled();

  WbViewpoint *viewpoint = world->viewpoint();
  WbSolid *followedSolid = viewpoint == NULL ? NULL : viewpoint->followedSolid();
  bool isFollowedSolid = followedSolid == node;
  QString followedSolidName;
  if (followedSolid)
    followedSolidName = followedSolid->name();

  // 2. regenerate the new node
  WbNode *upperTemplateNode = WbNodeUtilities::findUpperTemplateNeedingRegeneration(node);
  bool nested = upperTemplateNode && upperTemplateNode != node;
  cRegeneratingNodeCount++;
  if (isWorldInitialized && !restarted)
    // signal is not emitted in case a node has been regenerated twice in a row (`restart` == TRUE)
    // to preserve the scene tree selection
    emit preNodeRegeneration(node, nested);

  WbNode::setGlobalParentNode(parent);

  WbNode *newNode = WbNode::regenerateProtoInstanceFromParameters(proto, parameters, node->isTopLevel(),
                                                                  WbWorld::instance()->fileName(), true, uniqueId);

  if (!newNode) {
    WbLog::error(tr("Template regeneration failed. The node cannot be generated."), false, WbLog::PARSING);
    delete newNode;
    if (isWorldInitialized)
      emit abortNodeRegeneration();
    return;
  }

  newNode->setDefName(node->defName());
  WbNode::setGlobalParentNode(NULL);

  WbNodeUtilities::validateInsertedNode(parentField, newNode, parent, isInBoundingObject);

  subscribe(newNode);

  bool ancestorTemplateRegeneration = upperTemplateNode != NULL;
  if (node->isProtoParameterNode()) {
    const QVector<WbField *> &parentFields = parent->fieldsOrParameters();
    foreach (WbField *const parentField, parentFields) {
      if (parentField->type() == WB_SF_NODE) {
        WbSFNode *sfnode = static_cast<WbSFNode *>(parentField->value());
        if (sfnode->value() == node) {
          if (ancestorTemplateRegeneration)
            sfnode->blockSignals(true);

          sfnode->setValue(newNode);

          if (ancestorTemplateRegeneration) {
            sfnode->blockSignals(false);
            regenerateNode(upperTemplateNode);
            return;
          }
        }
      } else if (parentField->type() == WB_MF_NODE) {
        WbMFNode *mfnode = static_cast<WbMFNode *>(parentField->value());
        for (int i = 0; i < mfnode->size(); ++i) {
          WbNode *n = mfnode->item(i);
          if (n == node) {
            if (ancestorTemplateRegeneration)
              mfnode->blockSignals(true);

            mfnode->removeItem(i);
            mfnode->insertItem(i, newNode);

            if (ancestorTemplateRegeneration) {
              mfnode->blockSignals(false);
              regenerateNode(upperTemplateNode);
              return;
            }
            break;
          }
        }
      }
    }
  } else {
    // reassign pointer in parent
    WbGroup *const parentGroup = dynamic_cast<WbGroup *>(parent);
    WbBasicJoint *const parentJoint = dynamic_cast<WbBasicJoint *>(parent);
    WbShape *const parentShape = dynamic_cast<WbShape *>(parent);
    WbSkin *const parentSkin = dynamic_cast<WbSkin *>(parent);
    WbSlot *const parentSlot = dynamic_cast<WbSlot *>(parent);
    WbAppearance *const newAppearance = dynamic_cast<WbAppearance *>(newNode);
    WbPbrAppearance *const newPbrAppearance = dynamic_cast<WbPbrAppearance *>(newNode);
    WbGeometry *const newGeometry = dynamic_cast<WbGeometry *>(newNode);
    WbSlot *const newSlot = dynamic_cast<WbSlot *>(newNode);
    WbSolid *const newSolid = dynamic_cast<WbSolid *>(newNode);
    WbSolidReference *const newSolidReference = dynamic_cast<WbSolidReference *>(newNode);

    if (parentGroup) {
      int i = parentGroup->nodeIndex(node);
      assert(i != -1);

      // TODO: The 3 following lines could be simplified by using WbGroup::setChild(),
      //       but this function has to be fixed first (similar problem in WbSceneTree::transform
      // remove currentNode
      parentGroup->removeChild(node);
      // insert just after currentNode
      parentGroup->insertChild(i, newNode);
      delete node;  // In the other cases the setter function will take care of deleting the node
    } else if (parentSkin && parentSkin->appearanceField() && newAppearance) {
      int i = parentSkin->appearanceField()->nodeIndex(node);
      assert(i != -1);

      // TODO: WbMFNode::setItem doesn't work here either. Fix this along with WbGroup::setChild()
      parentSkin->appearanceField()->removeItem(i);
      parentSkin->appearanceField()->insertItem(i, newAppearance);
    } else if (parentShape && newGeometry)
      parentShape->setGeometry(newGeometry);
    else if (parentShape && newAppearance)
      parentShape->setAppearance(newAppearance);
    else if (parentShape && newPbrAppearance)
      parentShape->setPbrAppearance(newPbrAppearance);
    else if (parentSlot)
      parentSlot->setEndPoint(newNode);
    else if (parentJoint && newSolid)
      parentJoint->setSolidEndPoint(newSolid);
    else if (parentJoint && newSolidReference)
      parentJoint->setSolidEndPoint(newSolidReference);
    else if (parentJoint && newSlot)
      parentJoint->setSolidEndPoint(newSlot);
    else {
      WbLog::error(tr("Template regeneration failed. Unsupported node type."), false, WbLog::PARSING);
      delete newNode;
      emit abortNodeRegeneration();
      return;
    }
  }

  // let the supervisor set field functions work as if the node has not been deleted
  newNode->setUniqueId(uniqueId);

  // restore translation and rotation loaded from file
  WbSolid *newSolid = dynamic_cast<WbSolid *>(newNode);
  if (solid && newSolid) {
    newSolid->setTranslationFromFile(translationFromFile);
    newSolid->setRotationFromFile(rotationFromFile);
  }

  // update nested proto flag of current PROTO node and his instances if any
  newNode->updateNestedProtoFlag();

  // redirect parent parameters
  if (!previousParentRedirections.isEmpty()) {
    foreach (WbField *parentParameter, previousParentRedirections) {
      foreach (WbField *newParam, newNode->parameters()) {
        if (parentParameter->name() == newParam->alias())
          newParam->redirectTo(parentParameter);
      }
    }
  }

  mBlockRegeneration = true;  // prevent regenerating `newNode` in the finalization step due to field checks

  WbBaseNode *base = dynamic_cast<WbBaseNode *>(newNode);
  if (isWorldInitialized) {
    assert(base);
    base->finalize();
  }

  mBlockRegeneration = false;
  if (newNode->isRegenerationRequired()) {  // if needed, trigger `newNode` regeneration with finalized fields values
    regenerateNode(newNode, true);
    return;
  }

  // if the viewpoint is being re-generated we need to re-get the correct pointer, not the old dangling pointer from before
  // the node was regenerated
  viewpoint = world->viewpoint();
  if (isFollowedSolid)
    viewpoint->startFollowUp(newSolid, true);
  else if (!followedSolidName.isEmpty() && viewpoint->followedSolid() == NULL)
    // restore follow solid
    viewpoint->startFollowUp(WbSolid::findSolidFromUniqueName(followedSolidName), true);

  removeInvisibleProtoNodes(newNode);

  cRegeneratingNodeCount--;
  assert(cRegeneratingNodeCount >= 0);
  if (isWorldInitialized)
    emit postNodeRegeneration(newNode);
}

void WbTemplateManager::nodeNeedRegeneration() {
  mTemplatesNeedRegeneration = true;
}
