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

#ifndef WB_NODE_OPERATIONS_HPP
#define WB_NODE_OPERATIONS_HPP

//
// Description: class managing the insertion and deletion of nodes in the
//              current world.
//

#include <QtCore/QObject>

class WbBaseNode;
class WbField;
class WbNode;

class WbNodeOperations : public QObject {
  Q_OBJECT

public:
  static WbNodeOperations *instance();
  static void cleanup();

  enum OperationResult { FAILURE = 0, SUCCESS, REGENERATION_REQUIRED };
  enum ImportType { DEFAULT = 0, FROM_ADD_NEW, FROM_SUPERVISOR, FROM_PASTE };

  // import an object in the specified node and field
  // if 'filename' is an empty string, import the node defined by 'nodeString' instead
  OperationResult importNode(int nodeId, int fieldId, int itemIndex, ImportType origin, const QString &nodeString = "");
  // return if imported node is going to be regenerated
  OperationResult importNode(WbNode *parentNode, WbField *field, int itemIndex, ImportType origin,
                             const QString &nodeString = "", bool avoidIntersections = false);

  OperationResult initNewNode(WbNode *newNode, WbNode *parentNode, WbField *field, int newNodeIndex = -1,
                              bool subscribe = false, bool finalize = true);

  bool deleteNode(WbNode *node, bool fromSupervisor = false);

  void notifyNodeAdded(WbNode *node);
  void notifyNodeDeleted(WbNode *node);
  void notifyNodeRegenerated();

  bool updateDictionary(bool load, WbBaseNode *protoRoot);  // called after every modification of the Scene Tree

  bool areNodesAboutToBeInserted() { return mNodesAreAboutToBeInserted; }
  bool isSkipUpdates() { return mSkipUpdates; }
  bool isFromSupervisor() { return mFromSupervisor; }

  // EXTERNPROTO declarations
  void purgeUnusedExternProtoDeclarations();

  void enableSolidNameClashCheckOnNodeRegeneration(bool enabled) const;

public slots:
  void requestUpdateDictionary();
  void requestUpdateSceneDictionary(WbNode *node, bool fromUseToDef);

  // add missing EXTERNPROTO declarations after modifying parameters
  void updateExternProtoDeclarations(WbField *modifiedField);

signals:
  void nodeAdded(WbNode *node);
  void nodeDeleted(WbNode *node);

private:
  static WbNodeOperations *cInstance;
  WbNodeOperations();
  ~WbNodeOperations() {}

  bool mNodesAreAboutToBeInserted;
  bool mSkipUpdates;
  bool mFromSupervisor;

  void setFromSupervisor(bool value);

private slots:
  void resolveSolidNameClashIfNeeded(WbNode *node) const;
};

#endif
