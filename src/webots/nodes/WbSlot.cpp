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

#include "WbSlot.hpp"

#include "WbBoundingSphere.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"

void WbSlot::init() {
  // user fields
  mEndPoint = findSFNode("endPoint");
  mSlotType = findSFString("type");
}

WbSlot::WbSlot(WbTokenizer *tokenizer) : WbBaseNode("Slot", tokenizer) {
  init();
}

WbSlot::WbSlot(const WbSlot &other) : WbBaseNode(other) {
  init();
}

WbSlot::WbSlot(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbSlot::~WbSlot() {
}

void WbSlot::validateProtoNode() {
  WbSlot *slot = slotEndPoint();
  if (slot) {
    slot->validateProtoNode();
    return;
  }

  WbSolid *solid = solidEndPoint();
  if (solid)
    solid->validateProtoNode();
}

void WbSlot::downloadAssets() {
  WbBaseNode::downloadAssets();
  if (hasEndPoint())
    static_cast<WbBaseNode *>(endPoint())->downloadAssets();
}

void WbSlot::preFinalize() {
  WbBaseNode::preFinalize();

  connect(mEndPoint, &WbSFString::changed, this, &WbSlot::endPointChanged);
  WbGroup *pg = dynamic_cast<WbGroup *>(parentNode());
  if (pg)  // parent is a group
    connect(this, &WbSlot::endPointInserted, pg, &WbGroup::insertChildFromSlotOrJoint);
  WbSlot *ps = dynamic_cast<WbSlot *>(parentNode());
  if (ps)  // parent is another slot
    connect(this, &WbSlot::endPointInserted, ps, &WbSlot::endPointInserted);

  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e && !e->isPreFinalizedCalled())
    e->preFinalize();
}

void WbSlot::postFinalize() {
  WbBaseNode::postFinalize();

  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e && !e->isPostFinalizedCalled())
    e->postFinalize();

  connect(mSlotType, &WbSFString::changed, this, &WbSlot::updateType);
}

void WbSlot::updateType() {
  QString connectedType;
  const WbSlot *const parentSlot = dynamic_cast<WbSlot *>(parentNode());
  const WbSlot *const childSlot = slotEndPoint();
  if (parentSlot)
    connectedType = parentSlot->slotType();
  else if (childSlot)
    connectedType = childSlot->slotType();
  else
    return;

  QString errorMessage;
  if (!WbNodeUtilities::isSlotTypeMatch(slotType(), connectedType, errorMessage))
    parsingWarn(tr("Invalid 'type' changed to '%1': %2").arg(slotType()).arg(errorMessage));
}

WbSolid *WbSlot::solidEndPoint() const {
  return dynamic_cast<WbSolid *>(mEndPoint->value());
}

WbSolidReference *WbSlot::solidReferenceEndPoint() const {
  return dynamic_cast<WbSolidReference *>(mEndPoint->value());
}

WbSlot *WbSlot::slotEndPoint() const {
  return dynamic_cast<WbSlot *>(mEndPoint->value());
}

WbGroup *WbSlot::groupEndPoint() const {
  return dynamic_cast<WbGroup *>(mEndPoint->value());
}

void WbSlot::setEndPoint(WbNode *node) {
  WbBaseNode *const e = static_cast<WbBaseNode *>(node);
  mEndPoint->removeValue();
  mEndPoint->setValue(e);
  endPointChanged();
}

void WbSlot::createOdeObjects() {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->createOdeObjects();
}

void WbSlot::createWrenObjects() {
  WbBaseNode::createWrenObjects();
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->createWrenObjects();
}

void WbSlot::propagateSelection(bool selected) {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->propagateSelection(selected);
}

void WbSlot::setMatrixNeedUpdate() {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->setMatrixNeedUpdate();
}

void WbSlot::setScaleNeedUpdate() {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->setScaleNeedUpdate();
}

void WbSlot::updateCollisionMaterial(bool triggerChange, bool onSelection) {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->updateCollisionMaterial(triggerChange, onSelection);
}

void WbSlot::setSleepMaterial() {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->setSleepMaterial();
}

WbBoundingSphere *WbSlot::boundingSphere() const {
  WbBaseNode *const baseNode = static_cast<WbBaseNode *>(mEndPoint->value());
  if (baseNode)
    return baseNode->boundingSphere();

  return NULL;
}

void WbSlot::endPointChanged() {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e) {
    e->setParentNode(this);
    emit endPointInserted(e);
  }
}

QString WbSlot::endPointName() const {
  if (!mEndPoint->value())
    return QString();

  QString name = mEndPoint->value()->computeName();
  if (name.isEmpty())
    name = mEndPoint->value()->endPointName();
  return name;
}

void WbSlot::reset(const QString &id) {
  WbBaseNode::reset(id);

  WbNode *const e = mEndPoint->value();
  if (e)
    e->reset(id);
}

void WbSlot::save(const QString &id) {
  WbBaseNode::save(id);

  WbNode *const e = mEndPoint->value();
  if (e)
    e->save(id);
}

void WbSlot::updateSegmentationColor(const WbRgb &color) {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->updateSegmentationColor(color);
}

//////////////////////////////////////////////////////////////
//  WREN related methods for resizable WbGeometry children  //
//////////////////////////////////////////////////////////////

void WbSlot::attachResizeManipulator() {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->attachResizeManipulator();
}

void WbSlot::detachResizeManipulator() const {
  WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->detachResizeManipulator();
}

void WbSlot::write(WbWriter &writer) const {
  if (writer.isWebots())
    WbBaseNode::write(writer);
  else {
    if (hasEndPoint())
      mEndPoint->value()->write(writer);
  }
}

QList<const WbBaseNode *> WbSlot::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const WbBaseNode *> list;
  const WbBaseNode *const e = static_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    list << e->findClosestDescendantNodesWithDedicatedWrenNode();
  return list;
}
