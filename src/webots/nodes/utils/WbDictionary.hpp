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

#ifndef WB_DICTIONARY_HPP
#define WB_DICTIONARY_HPP

//
// Description: Handles the updates of the DEF names Dictionary
//

#include <QtCore/QMultiMap>
#include <QtCore/QStringList>

class WbBaseNode;
class WbField;
class WbNode;
class WbMFNode;
class WbSFNode;

class WbDictionary {
public:
  static WbDictionary *instance();
  static void cleanup();

  // Recompute all DEF-USE dependencies (but protos' dependencies) and update
  // them if needed according to the VRML rule:
  // a USE node refers to its closest previous DEF node if it exists (otherwise it is turned into a DEF node)
  bool update(bool load = false);
  void updateProtosPrivateDef(WbBaseNode *&node);

  // If dictionary update is called after PROTO regeneration, then store the upper template PROTO node to prevent infinite
  // DEF/USE updates
  void setRegeneratedNode(const WbNode *node);

  // Loop through current world and return all the available DEF nodes
  // that could be used in the specified field.
  // If 'suitableOnly' is true, then only the DEF nodes that can be inserted in
  // the field are returned.
  QList<WbNode *> computeDefForInsertion(WbNode *const targetNode, WbField *const targetField, int targetIndex,
                                         bool suitableOnly = true);

  // Get node from DEF name
  WbNode *getNodeFromDEF(const QString &defName) const;
  void updateNodeDefName(WbNode *node, bool fromUseToDef);
  void removeNodeFromDictionary(WbNode *node);

private:
  static WbDictionary *cInstance;
  WbDictionary();
  ~WbDictionary();

  bool updateDef(WbBaseNode *&node, WbSFNode *sfNode, WbMFNode *mfNode, int index, bool isTemplateRegenerator,
                 bool &regenerationRequired);
  void updateProtosDef(WbBaseNode *&node, WbSFNode *sfNode = NULL, WbMFNode *mfNode = NULL, int index = -1);
  void updateForInsertion(const WbNode *const node, bool suitableOnly, QList<WbNode *> &defNodes);
  void makeDefNodeAndUpdateDictionary(WbBaseNode *node, bool updateSceneDictionary);
  QList<QMultiMap<QString, WbNode *>> mNestedDictionaries;
  void clearNestedDictionaries() {
    mNestedUseNodes.clear();
    mNestedDictionaries.clear();
    mNestedDictionaries << QMultiMap<QString, WbNode *>();
  }
  WbNode *mTargetNode;
  WbField *mTargetField;
  int mTargetIndex;
  QList<WbBaseNode *> mNestedProtos;
  QList<WbBaseNode *> mNestedUseNodes;
  bool mStopUpdate;
  bool mLoad;                      // true if the update occurs right after world is loaded
  bool mCurrentProtoRegeneration;  // true if the update occurs due to a PROTO regeneration
  const WbNode *mCurrentProtoRegenerationNode;
  bool isSuitable(const WbNode *defNode, const QString &type) const;
  static bool checkChargerAndLedConstraints(WbNode *useNodeParent, const WbBaseNode *defNode, QString &deviceModelName,
                                            bool isFirstChild);
  static bool checkBoundingObjectConstraints(const WbBaseNode *defNode, QString &errorMessage);

  // List of DEF nodes visible in the scene tree
  QList<std::pair<WbNode *, QString>> mSceneDictionary;
};

#endif
