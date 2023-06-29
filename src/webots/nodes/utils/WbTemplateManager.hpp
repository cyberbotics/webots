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

#ifndef WB_TEMPLATE_MANAGER_HPP
#define WB_TEMPLATE_MANAGER_HPP

//
// Description:    template manager
// Responsability: manage the upates of the templates
//

#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QSet>

class WbNode;
class WbField;

class WbTemplateManager : public QObject {
  Q_OBJECT

public:
  static WbTemplateManager *instance();

  void clear();
  void subscribe(WbNode *node, bool subscribedDescendant);

  // when unblocked, all the templates which have required
  // a regeneration inbetween are regenerated
  void blockRegeneration(bool block);

  static bool isRegenerating() { return cRegeneratingNodeCount > 0; }
  static bool isNodeChangeTriggeringRegeneration(const WbNode *node) {
    return WbTemplateManager::instance()->mNodesSubscribedForRegeneration.contains(node);
  }

signals:
  void preNodeRegeneration(WbNode *node, bool nested);
  void abortNodeRegeneration();
  void postNodeRegeneration(WbNode *node);

private slots:
  void unsubscribe(QObject *node);
  void regenerateNodeFromFieldChange(WbField *field);
  void regenerateNodeFromParameterChange(WbField *field);
  void regenerateNode(WbNode *node, bool restarted = false);
  void nodeNeedRegeneration();

private:
  static void cleanup();
  static WbTemplateManager *cInstance;
  static int cRegeneratingNodeCount;

  WbTemplateManager();
  virtual ~WbTemplateManager();

  bool nodeNeedsToSubscribe(WbNode *node);
  void recursiveFieldSubscribeToRegenerateNode(WbNode *node, bool subscribedNode, bool subscribedDescendant);
  void regenerateNodeFromField(WbNode *templateNode, WbField *field, bool isParameter);

  QList<WbNode *> mTemplates;
  QSet<const WbNode *> mNodesSubscribedForRegeneration;

  bool mBlockRegeneration;
  bool mTemplatesNeedRegeneration;
  WbNode *mRegeneratingUpperTemplateNode;
};

#endif
