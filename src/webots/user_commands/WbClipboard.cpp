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

#include "WbClipboard.hpp"

#include "WbBaseNode.hpp"
#include "WbDictionary.hpp"
#include "WbNodeUtilities.hpp"
#include "WbRgb.hpp"
#include "WbRotation.hpp"
#include "WbVector2.hpp"
#include "WbVector3.hpp"
#include "WbVrmlNodeUtilities.hpp"

#include "../../../include/controller/c/webots/supervisor.h"

#include <QtGui/QClipboard>
#include <QtWidgets/QApplication>
#include <cassert>

static WbClipboard *gInstance = NULL;

WbClipboard *WbClipboard::instance() {
  if (!gInstance)
    gInstance = new WbClipboard();

  return gInstance;
}

void WbClipboard::deleteInstance() {
  if (!gInstance)
    return;
  gInstance->clear();
  delete gInstance;
  gInstance = NULL;
}

WbClipboard::WbClipboard() : WbVariant(), mNodeInfo(NULL), mSystemClipboard(QApplication::clipboard()) {
  update();
}

WbClipboard &WbClipboard::operator=(const WbVariant &other) {
  mSystemClipboard->blockSignals(true);
  WbVariant::setValue(other);
  mSystemClipboard->blockSignals(false);
  return *this;
}

void WbClipboard::update() {
  const QString systemClipboardValue = mSystemClipboard->text();
  if (systemClipboardValue != mStringValue) {
    WbVariant::setString(systemClipboardValue);
    mStringValue = systemClipboardValue;
  }
}

QString WbClipboard::stringValue() {
  update();
  return mStringValue;
}

void WbClipboard::setBool(bool b) {
  WbVariant::setBool(b);
  mStringValue = b ? "true" : "false";

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::setInt(int i) {
  WbVariant::setInt(i);
  mStringValue.setNum(i);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::setDouble(double d) {
  WbVariant::setDouble(d);
  mStringValue.setNum(d);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::setString(const QString &s) {
  WbVariant::setString(s);
  mStringValue = s;

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::setVector2(const WbVector2 &v) {
  WbVariant::setVector2(v);
  mStringValue = v.toString(WbPrecision::DOUBLE_MAX);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::setVector3(const WbVector3 &v) {
  WbVariant::setVector3(v);
  mStringValue = v.toString(WbPrecision::DOUBLE_MAX);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::setColor(const WbRgb &c) {
  WbVariant::setColor(c);
  mStringValue = c.toString(WbPrecision::DOUBLE_MAX);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::setRotation(const WbRotation &r) {
  WbVariant::setRotation(r);
  mStringValue = r.toString(WbPrecision::DOUBLE_MAX);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::setNode(WbNode *n, bool persistent) {
  if (!n)
    return;

  WbVariant::setNode(NULL, persistent);
  assert(!mNodeInfo);
  mNodeInfo = new WbClipboardNodeInfo();
  mNodeInfo->modelName = n->modelName();
  mNodeInfo->nodeModelName = n->nodeModelName();
  mNodeInfo->slotType = WbNodeUtilities::slotType(n);
  mNodeInfo->hasADeviceDescendant = WbNodeUtilities::hasADeviceDescendant(n, true);
  mNodeInfo->hasAConnectorDescendant = mNodeInfo->hasADeviceDescendant || WbNodeUtilities::hasADeviceDescendant(n, false);
  WbNode::enableDefNodeTrackInWrite(false);
  mNodeExportString = WbVrmlNodeUtilities::exportNodeToString(n);
  QList<std::pair<WbNode *, int>> externalDefNodes(*WbNode::externalUseNodesPositionsInWrite());
  WbNode::disableDefNodeTrackInWrite();
  // store all the required external DEF nodes data in order to work correctly
  // independently if other nodes are deleted
  for (int i = 0; i < externalDefNodes.size(); ++i) {
    WbBaseNode *node = dynamic_cast<WbBaseNode *>(externalDefNodes[i].first);
    LinkedDefNodeDefinitions *data = new LinkedDefNodeDefinitions();
    data->position = externalDefNodes[i].second;
    data->type = node->nodeType();
    data->defName = node->defName();
    WbNode::enableDefNodeTrackInWrite(false);
    data->definition = WbVrmlNodeUtilities::exportNodeToString(node);
    WbNode::disableDefNodeTrackInWrite();
    mLinkedDefNodeDefinitions.append(data);
  }

  mStringValue = n->fullName();
  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue, QClipboard::Clipboard);
  mSystemClipboard->blockSignals(false);
}

void WbClipboard::clear() {
  WbVariant::clear();
  delete mNodeInfo;
  mNodeInfo = NULL;
  mNodeExportString.clear();
  for (int i = 0; i < mLinkedDefNodeDefinitions.size(); ++i)
    delete mLinkedDefNodeDefinitions[i];
  mLinkedDefNodeDefinitions.clear();
}

int WbClipboard::replaceExternalNodeDefinitionInString(QString &nodeString, int index) const {
  int position = mLinkedDefNodeDefinitions[index]->position;
  int oldStringSize = mLinkedDefNodeDefinitions[index]->defName.size() + 4;
  nodeString.remove(position, oldStringSize);
  const QString &newString = mLinkedDefNodeDefinitions[index]->definition;
  nodeString.insert(position, newString);
  return newString.size() - oldStringSize;
}

void WbClipboard::replaceAllExternalDefNodesInString() {
  while (!mLinkedDefNodeDefinitions.isEmpty()) {
    replaceExternalNodeDefinitionInString(mNodeExportString, mLinkedDefNodeDefinitions.size() - 1);
    mLinkedDefNodeDefinitions.removeLast();
  }
}

QString WbClipboard::computeNodeExportStringForInsertion(WbNode *parentNode, WbField *field, int fieldIndex) const {
  QString nodeString(mNodeExportString);
  QList<WbNode *> existingDefNodes = WbDictionary::instance()->computeDefForInsertion(parentNode, field, fieldIndex, false);
  const int existingDefNodesSize = existingDefNodes.size();
  for (int i = mLinkedDefNodeDefinitions.size() - 1; i >= 0; --i) {
    bool found = false;
    for (int j = 0; j < existingDefNodesSize && !found; ++j) {
      WbBaseNode *node = dynamic_cast<WbBaseNode *>(existingDefNodes[j]);
      found =
        node->defName() == mLinkedDefNodeDefinitions[i]->defName && node->nodeType() == mLinkedDefNodeDefinitions[i]->type;
    }
    if (!found)
      replaceExternalNodeDefinitionInString(nodeString, i);
  }
  return nodeString;
}
