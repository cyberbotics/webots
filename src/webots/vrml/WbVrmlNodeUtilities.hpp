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

#ifndef WB_VRML_NODE_UTILTIES_HPP
#define WB_VRML_NODE_UTILTIES_HPP

//
// Description: utility class allowing to query nodes in their tree context through static functions.
//              Only generic node properties are used in this class.
//              For advanced functionalities based on the node type please refer to WbNodeUtilities namespace.
//

#include <QtCore/QList>

class WbField;
class WbNode;
class WbProtoModel;
class WbTokenizer;

namespace WbVrmlNodeUtilities {
  //////////////////////////
  // Permanent properties //
  //////////////////////////
  // find the ancestor node directly attached to the root node
  const WbNode *findTopNode(const WbNode *node);

  // is the target field or node visible in the Scene Tree (possibly as a nested proto parameter)
  bool isVisible(const WbNode *node);
  bool isVisible(const WbField *target);

  // is this node located directly or indirectly in the given field
  bool isFieldDescendant(const WbNode *node, const QString &fieldName);
  // find the field parent of the target field, i.e. the closest upper field in the tree hierarchy
  WbField *findFieldParent(const WbField *target, bool internal = false);

  // find (innermost) enclosing PROTO if any
  WbProtoModel *findContainingProto(const WbNode *node);
  const WbNode *findFieldProtoScope(const WbField *field, const WbNode *proto);
  const WbField *findClosestParameterInProto(const WbField *field, const WbNode *proto);

  // find root PROTO node if any
  WbNode *findRootProtoNode(WbNode *const node);

  // return the list of PROTO nodes "visible" in the world (skipping default PROTO parameters)
  QList<const WbNode *> protoNodesInWorldFile(const WbNode *root);

  // checks whether a node of specific model name exists in the node tree and returns true if it is visible
  // default fields that won't be written to the WBT file are skipped
  bool existsVisibleProtoNodeNamed(const QString &modelName, WbNode *root);

  // find the closest template ancestor in which the modified node is contained in template field
  // which requires a template instance regeneration
  WbNode *findUpperTemplateNeedingRegeneration(WbNode *modifiedNode);

  // find the closest template ancestor of given field in which the modified field is contained
  // in template field which requires a template instance regeneration
  WbNode *findUpperTemplateNeedingRegenerationFromField(WbField *modifiedField, WbNode *parentNode);

  //////////////////////////////
  // Non-permanent properties //
  //////////////////////////////

  // has this node a USE node ancestor
  bool hasAUseNodeAncestor(const WbNode *node);
  // find all ancestor USE nodes
  QList<WbNode *> findUseNodeAncestors(WbNode *node);
  // has this DEF node a subsequent USE or DEF node using its new definition
  bool hasASubsequentUseOrDefNode(const WbNode *defNode, const QString &defName, const QString &previousDefName,
                                  bool &useOverlap, bool &defOverlap);

  // has this node a referred DEF node descendant, i.e. a descendant with positive use count
  // which is moreover referred outside the subtree below node
  bool hasAreferredDefNodeDescendant(const WbNode *node, const WbNode *root = NULL);

  ///////////
  // Other //
  ///////////
  QString exportNodeToString(WbNode *node);

  bool transformBackwardCompatibility(WbTokenizer *tokenizer);
}  // namespace WbVrmlNodeUtilities
#endif
