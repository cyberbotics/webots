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

#include "WbVrmlNodeUtilities.hpp"

#include "WbField.hpp"
#include "WbMFNode.hpp"
#include "WbNode.hpp"
#include "WbSFNode.hpp"
#include "WbTokenizer.hpp"
#include "WbWriter.hpp"

#include <QtCore/QQueue>

namespace {
  bool checkForUseOrDefNode(const WbNode *node, const QString &useName, const QString &previousUseName, bool &useOverlap,
                            bool &defOverlap, bool &abortSearch);

  bool checkForUseOrDefNode(WbField *field, const QString &useName, const QString &previousUseName, bool &useOverlap,
                            bool &defOverlap, bool &abortSearch) {
    WbValue *const value = field->value();
    const WbMFNode *const mfnode = dynamic_cast<WbMFNode *>(value);
    if (mfnode) {
      const int size = mfnode->size();
      for (int i = 0; i < size; ++i) {
        const WbNode *const n = mfnode->item(i);
        if (!n)
          continue;

        if (!previousUseName.isEmpty() && n->defName() == previousUseName) {
          abortSearch = true;
          return false;
        }

        if (defOverlap) {
          if (!previousUseName.isEmpty() && n->useName() == previousUseName)
            return true;
        } else if (n->defName() == useName) {
          defOverlap = true;
        } else if (n->useName() == useName) {
          useOverlap = true;
        }

        if (checkForUseOrDefNode(n, useName, previousUseName, useOverlap, defOverlap, abortSearch))
          return true;

        if (abortSearch)
          return false;
      }
      return false;
    } else {
      const WbSFNode *const sfnode = dynamic_cast<WbSFNode *>(value);
      if (sfnode) {
        const WbNode *const n = sfnode->value();
        if (n) {
          if (!previousUseName.isEmpty() && n->defName() == previousUseName) {
            abortSearch = true;
            return false;
          }

          if (defOverlap) {
            if (!previousUseName.isEmpty() && n->useName() == previousUseName)
              return true;
          } else if (n->defName() == useName) {
            defOverlap = true;
          } else if (n->useName() == useName) {
            useOverlap = true;
          }

          if (checkForUseOrDefNode(n, useName, previousUseName, useOverlap, defOverlap, abortSearch))
            return true;
        }
      }
    }

    return false;
  }

  bool checkForUseOrDefNode(const WbNode *node, const QString &useName, const QString &previousUseName, bool &useOverlap,
                            bool &defOverlap, bool &abortSearch) {
    // Check fields and parameters
    const QVector<WbField *> &fields = node->fieldsOrParameters();
    for (int i = 0, size = fields.size(); i < size; ++i) {
      if (checkForUseOrDefNode(fields[i], useName, previousUseName, useOverlap, defOverlap, abortSearch))
        return true;
      if (abortSearch)
        return false;
    }
    return false;
  }
}  // namespace

const WbNode *WbVrmlNodeUtilities::findTopNode(const WbNode *node) {
  if (node == NULL || node->isWorldRoot())
    return NULL;

  const WbNode *n = node;
  const WbNode *parent = n->parentNode();
  while (parent) {
    if (parent->isWorldRoot())
      return n;

    n = parent;
    parent = n->parentNode();
  }
  return NULL;
}

bool WbVrmlNodeUtilities::isVisible(const WbNode *node) {
  if (node == NULL)
    return false;

  const WbNode *n = node;
  const WbNode *p = n->parentNode();
  while (n && p && !n->isTopLevel()) {
    if (p->isProtoInstance()) {
      if (p->fields().contains(n->parentField(true)))
        // internal node of a PROTO
        return false;
    }
    n = p;
    p = p->parentNode();
  }
  return true;
}

bool WbVrmlNodeUtilities::isVisible(const WbField *target) {
  if (target == NULL)
    return false;

  const WbField *parameter = target;
  const WbField *parentParameter = target->parameter();

  while (parentParameter) {
    parameter = parentParameter;
    parentParameter = parentParameter->parameter();
  }
  const WbNode *parentNode = parameter->parentNode();
  assert(parentNode);
  if (parentNode->fieldsOrParameters().contains(const_cast<WbField *>(parameter)))
    return isVisible(parentNode);
  return false;
}

bool WbVrmlNodeUtilities::isFieldDescendant(const WbNode *node, const QString &fieldName) {
  if (node == NULL)
    return false;

  WbNode *n = node->parentNode();
  WbField *field = node->parentField(true);
  while (n && !n->isWorldRoot() && field) {
    if (field->name() == fieldName)
      return true;

    field = n->parentField(true);
    n = n->parentNode();
  }

  return false;
}

WbField *WbVrmlNodeUtilities::findFieldParent(const WbField *target, bool internal) {
  const WbNode *const nodeParent = target->parentNode();
  assert(nodeParent);
  bool valid = false;
  if (internal)
    valid = nodeParent->fieldsOrParameters().contains(const_cast<WbField *>(target));
  else
    valid = nodeParent->fields().contains(const_cast<WbField *>(target));
  return valid ? nodeParent->parentField() : NULL;
}

WbProtoModel *WbVrmlNodeUtilities::findContainingProto(const WbNode *node) {
  const WbNode *n = node;
  do {
    WbProtoModel *proto = n->proto();
    if (proto)
      return proto;
    else {
      const WbNode *const protoParameterNode = n->protoParameterNode();
      proto = protoParameterNode ? protoParameterNode->proto() : NULL;
      if (proto)
        return proto;

      n = n->parentNode();
    }
  } while (n);
  return NULL;
}

const WbNode *WbVrmlNodeUtilities::findFieldProtoScope(const WbField *field, const WbNode *proto) {
  assert(field);

  if (!proto)
    return NULL;

  const WbNode *node;
  const WbField *candidate = field;
  const WbField *parameter = NULL;
  while (candidate) {
    node = candidate->parentNode();
    if (!node->parentField() && node->protoParameterNode())
      node = node->protoParameterNode();

    parameter = findClosestParameterInProto(candidate, proto);
    if (parameter)
      break;

    candidate = node->parentField();
  }

  if (parameter && !parameter->isDefault())
    return findFieldProtoScope(parameter, proto->containingProto(true));
  else
    return proto;
}

const WbField *WbVrmlNodeUtilities::findClosestParameterInProto(const WbField *field, const WbNode *proto) {
  if (!field || !proto || !proto->proto())
    return NULL;

  const WbNode *parameterNode = proto;
  while (parameterNode) {
    const WbField *parameter = field;
    const QVector<WbField *> parameterList = parameterNode->parameters();

    while (parameter) {
      if (parameterList.contains(const_cast<WbField *>(parameter)))
        return parameter;

      parameter = parameter->parameter();
    }

    parameterNode = parameterNode->protoParameterNode();
  }

  return NULL;
}

WbNode *WbVrmlNodeUtilities::findRootProtoNode(WbNode *const node) {
  WbNode *n = node;
  do {
    WbProtoModel *proto = n->proto();
    if (proto)
      return n;
    n = n->parentNode();
  } while (n);
  return NULL;
}

QList<const WbNode *> WbVrmlNodeUtilities::protoNodesInWorldFile(const WbNode *root) {
  QList<const WbNode *> result;
  QQueue<const WbNode *> queue;
  queue.append(root);
  while (!queue.isEmpty()) {
    const WbNode *node = queue.dequeue();
    if (node->isProtoInstance())
      result.append(node);
    QVector<WbField *> fields = node->fieldsOrParameters();
    QVectorIterator<WbField *> it(fields);
    while (it.hasNext()) {
      const WbField *field = it.next();
      if (field->isDefault())
        continue;  // ignore default fields that will not be written to file
      const QList<WbNode *> children(node->subNodes(field, false, false, false));
      foreach (WbNode *child, children)
        queue.enqueue(child);
    }
  }

  return result;
}

bool WbVrmlNodeUtilities::existsVisibleProtoNodeNamed(const QString &modelName, WbNode *root) {
  if (!root)
    return false;

  QQueue<WbNode *> queue;
  queue.append(root->subNodes(false, false, false));
  while (!queue.isEmpty()) {
    const WbNode *node = queue.dequeue();
    if (node->modelName() == modelName)
      return true;
    QVector<WbField *> fields = node->fieldsOrParameters();
    QVectorIterator<WbField *> it(fields);
    while (it.hasNext()) {
      const WbField *field = it.next();
      if (field->isDefault())
        continue;  // ignore default fields that will not be written to file
      queue.append(node->subNodes(field, false, false, false));
    }
  }
  return false;
}

WbNode *WbVrmlNodeUtilities::findUpperTemplateNeedingRegenerationFromField(WbField *modifiedField, WbNode *parentNode) {
  if (parentNode == NULL || modifiedField == NULL)
    return NULL;

  if (parentNode->isTemplate() && modifiedField->isTemplateRegenerator())
    return parentNode;

  return findUpperTemplateNeedingRegeneration(parentNode);
}

WbNode *WbVrmlNodeUtilities::findUpperTemplateNeedingRegeneration(WbNode *modifiedNode) {
  if (modifiedNode == NULL)
    return NULL;

  WbField *field = modifiedNode->parentField();
  WbNode *node = modifiedNode->parentNode();
  while (node && field && !node->isWorldRoot()) {
    if (node->isTemplate() && field->isTemplateRegenerator())
      return node;

    field = node->parentField();
    node = node->parentNode();
  }

  return NULL;
}

bool WbVrmlNodeUtilities::hasAUseNodeAncestor(const WbNode *node) {
  const WbNode *p = node;
  while (p) {
    if (p->isUseNode())
      return true;
    p = p->parentNode();
  }

  return false;
}

bool WbVrmlNodeUtilities::hasASubsequentUseOrDefNode(const WbNode *defNode, const QString &defName,
                                                     const QString &previousDefName, bool &useOverlap, bool &defOverlap) {
  if (defName.isEmpty())
    return false;

  useOverlap = false;
  defOverlap = false;
  bool abortSearch = false;

  if (checkForUseOrDefNode(defNode, defName, previousDefName, useOverlap, defOverlap, abortSearch))
    return true;

  if (abortSearch) {
    defOverlap = false;
    return useOverlap;
  }

  const WbNode *node = defNode;
  const WbNode *parentNode = node->parentNode();

  while (parentNode) {
    WbField *const parentField = node->parentField();
    const WbMFNode *const mfnode = dynamic_cast<WbMFNode *>(parentField->value());
    if (mfnode) {
      const int index = mfnode->nodeIndex(node) + 1;
      const int size = mfnode->size();
      for (int i = index; i < size; ++i) {
        const WbNode *const n = mfnode->item(i);
        if (!previousDefName.isEmpty() && n->defName() == previousDefName) {
          defOverlap = false;
          return useOverlap;
        }

        if (defOverlap) {
          if (!previousDefName.isEmpty() && n->useName() == previousDefName)
            return true;
        } else if (n->defName() == defName) {
          defOverlap = true;
        } else if (n->useName() == defName) {
          useOverlap = true;
        }

        if (checkForUseOrDefNode(n, defName, previousDefName, useOverlap, defOverlap, abortSearch))
          return true;

        if (abortSearch) {
          defOverlap = false;
          return useOverlap;
        }
      }
    }

    const int fieldIndex = parentNode->fieldIndex(parentField) + 1;
    const QVector<WbField *> &fields = parentNode->fieldsOrParameters();
    const int size = fields.size();
    for (int i = fieldIndex; i < size; ++i) {
      WbField *const f = fields[i];
      if (checkForUseOrDefNode(f, defName, previousDefName, useOverlap, defOverlap, abortSearch))
        return true;
      if (abortSearch) {
        defOverlap = false;
        return useOverlap;
      }
    }

    node = parentNode;
    parentNode = parentNode->parentNode();
  }

  return useOverlap;
}

bool WbVrmlNodeUtilities::hasAreferredDefNodeDescendant(const WbNode *node, const WbNode *root) {
  const WbNode *rootNode = root ? root : node;
  const int count = node->useCount();
  const QList<WbNode *> &useNodes = node->useNodes();
  for (int i = 0; i < count; ++i) {
    if (!rootNode->isAnAncestorOf(useNodes.at(i)))
      return true;
  }

  foreach (WbField *field, node->fieldsOrParameters()) {
    WbValue *value = field->value();
    const WbSFNode *const sfnode = dynamic_cast<WbSFNode *>(value);
    if (sfnode && sfnode->value()) {
      const WbNode *childNode = sfnode->value();
      const int nodeCount = childNode->useCount();
      const QList<WbNode *> &nodeUseNodes = childNode->useNodes();
      for (int i = 0; i < nodeCount; ++i) {
        if (!rootNode->isAnAncestorOf(nodeUseNodes.at(i)))
          return true;
      }
      const bool subtreeHasDef = hasAreferredDefNodeDescendant(childNode, rootNode);
      if (subtreeHasDef)
        return subtreeHasDef;
    } else {
      const WbMFNode *const mfnode = dynamic_cast<WbMFNode *>(value);
      if (mfnode) {
        const int size = mfnode->size();
        for (int i = 0; i < size; ++i) {
          const WbNode *childNode = mfnode->item(i);
          const int nodeCount = childNode->useCount();
          const QList<WbNode *> &nodeUseNodes = childNode->useNodes();
          for (int j = 0; j < nodeCount; ++j) {
            if (!rootNode->isAnAncestorOf(nodeUseNodes.at(j)))
              return true;
          }
          const bool subtreeHasDef = hasAreferredDefNodeDescendant(childNode, rootNode);
          if (subtreeHasDef)
            return subtreeHasDef;
        }
      }
    }
  }

  return false;
}

QList<WbNode *> WbVrmlNodeUtilities::findUseNodeAncestors(WbNode *node) {
  QList<WbNode *> list;

  if (node == NULL)
    return list;

  WbNode *n = node;
  while (n && !n->isWorldRoot()) {
    if (n->isUseNode())
      list.prepend(n);
    n = n->parentNode();
  }

  return list;
}

QString WbVrmlNodeUtilities::exportNodeToString(WbNode *node) {
  QString nodeString;
  WbWriter writer(&nodeString, ".wbt");
  node->write(writer);
  return nodeString;
}

// Return true if we can convert the Transform to a Pose.
bool WbVrmlNodeUtilities::transformBackwardCompatibility(WbTokenizer *tokenizer) {
  if (!tokenizer)
    return true;

  const int initalIndex = tokenizer->pos();
  bool inChildren = false;
  int bracketCount = 0;
  while (tokenizer->hasMoreTokens()) {
    const QString token = tokenizer->nextWord();
    if (inChildren) {
      if (token == "[")
        bracketCount++;
      else if (token == "]") {
        bracketCount--;
        if (bracketCount == 0)
          inChildren = false;
      }
    } else if (token == "children") {
      inChildren = true;
    } else if (token == "scale") {
      for (int i = 0; i < 3; i++) {
        if (tokenizer->nextWord().toFloat() != 1.0f) {
          tokenizer->seek(initalIndex);
          return false;
        }
      }
      // We have identified that the scale is the default one.
      break;
      // End of the Transform
    } else if (token == "}")
      break;
  }
  tokenizer->seek(initalIndex);

  return true;
}
