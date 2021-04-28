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

#include "WbGroup.hpp"

#include "WbBasicJoint.hpp"
#include "WbBoundingSphere.hpp"
#include "WbGeometry.hpp"
#include "WbOdeContext.hpp"
#include "WbSlot.hpp"

using namespace WbHiddenKinematicParameters;

void WbGroup::init() {
  mOdeSpace = NULL;
  mHasNoSolidAncestor = true;
  mBoundingSphere = NULL;

  mChildren = findMFNode("children");
}

WbGroup::WbGroup(WbTokenizer *tokenizer) : WbBaseNode("Group", tokenizer) {
  init();
}

WbGroup::WbGroup(const QString &modelName, WbTokenizer *tokenizer) : WbBaseNode(modelName, tokenizer) {
  init();
}

WbGroup::WbGroup(const WbGroup &other) : WbBaseNode(other) {
  init();
}

WbGroup::WbGroup(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbGroup::~WbGroup() {
  if (mOdeSpace) {
    dSpaceDestroy(mOdeSpace);
    mOdeSpace = NULL;
  }
  delete mBoundingSphere;
}

void WbGroup::downloadAssets() {
  WbBaseNode::downloadAssets();
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbBaseNode *const n = static_cast<WbBaseNode *>(it.next());
    n->downloadAssets();
  }
}

void WbGroup::preFinalize() {
  WbBaseNode::preFinalize();

  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbBaseNode *const n = static_cast<WbBaseNode *>(it.next());
    if (!mHasNoSolidAncestor) {
      WbGroup *group = dynamic_cast<WbGroup *>(n);
      if (group)
        group->mHasNoSolidAncestor = false;
    }
    n->preFinalize();
  }
}

void WbGroup::postFinalize() {
  WbBaseNode::postFinalize();

  mBoundingSphere = new WbBoundingSphere(this);
  mBoundingSphere->empty();

  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbBaseNode *const n = static_cast<WbBaseNode *>(it.next());
    n->postFinalize();
    if (mBoundingSphere)
      mBoundingSphere->addSubBoundingSphere(n->boundingSphere());
  }

  connect(mChildren, &WbMFNode::changed, this, &WbGroup::childrenChanged);
  connect(mChildren, &WbMFNode::itemInserted, this, &WbGroup::insertChildPrivate);
  // if parent is a slot, it needs to be notified when a new node is inserted
  WbSlot *ps = dynamic_cast<WbSlot *>(parentNode());
  if (ps)
    connect(this, &WbGroup::notifyParentSlot, ps, &WbSlot::endPointInserted);
  // if parent is a joint, it needs to be notified when a new node is inserted
  const WbBasicJoint *pj = dynamic_cast<WbBasicJoint *>(parentNode());
  if (pj)
    connect(this, &WbGroup::notifyParentJoint, pj, &WbBasicJoint::endPointChanged);

  const WbGroup *const parent = dynamic_cast<const WbGroup *const>(parentNode());
  if (parent && parent->mHasNoSolidAncestor) {
    connect(mChildren, &WbMFNode::changed, this, &WbGroup::topLevelListsUpdateRequested);
    connect(this, &WbGroup::topLevelListsUpdateRequested, parent, &WbGroup::topLevelListsUpdateRequested);
  } else if (mHasNoSolidAncestor)
    connect(mChildren, &WbMFNode::changed, this, &WbGroup::topLevelListsUpdateRequested);
}

void WbGroup::insertChild(int index, WbNode *child) {
  child->setParentNode(this);
  mChildren->insertItem(index, child);
}

void WbGroup::setChild(int index, WbNode *child) {
  child->setParentNode(this);
  mChildren->setItem(index, child);
}

void WbGroup::addChild(WbNode *child) {
  child->setParentNode(this);
  mChildren->addItem(child);
}

bool WbGroup::removeChild(WbNode *node) {
  if (mChildren->removeNode(node)) {
    node->setParentNode(NULL);
    return true;
  } else
    return false;
}

void WbGroup::clear() {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbNode *const n = it.next();
    n->setParentNode(NULL);
  }
  mChildren->clear();
}

void WbGroup::deleteAllChildren() {
  mChildren->clear();
}

WbBaseNode *WbGroup::child(int index) const {
  return static_cast<WbBaseNode *>(mChildren->item(index));
}

int WbGroup::nodeIndex(WbNode *child) const {
  return mChildren->nodeIndex(child);
}

void WbGroup::createOdeObjects() {
  WbBaseNode::createOdeObjects();

  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<WbBaseNode *>(it.next())->createOdeObjects();
}

void WbGroup::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  WbMFNode::Iterator it(*mChildren);
  int index = 0;
  while (it.hasNext()) {
    index++;
    static_cast<WbBaseNode *>(it.next())->createWrenObjects();
    emit childFinalizationHasProgressed(index * 100 / childCount());
    if (mFinalizationCanceled)
      return;
  }
}

void WbGroup::cancelFinalization() {
  mFinalizationCanceled = true;
}

void WbGroup::propagateSelection(bool selected) {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<WbBaseNode *>(it.next())->propagateSelection(selected);
}

void WbGroup::setMatrixNeedUpdate() {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<WbBaseNode *>(it.next())->setMatrixNeedUpdate();
}

void WbGroup::setScaleNeedUpdate() {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<WbBaseNode *>(it.next())->setScaleNeedUpdate();
}

void WbGroup::updateCollisionMaterial(bool triggerChange, bool onSelection) {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbBaseNode *const n = static_cast<WbBaseNode *>(it.next());
    n->updateCollisionMaterial(triggerChange, onSelection);
  }
}

void WbGroup::setSleepMaterial() {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<WbBaseNode *>(it.next())->setSleepMaterial();
}

bool WbGroup::isSuitableForInsertionInBoundingObject(bool warning) const {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    const WbBaseNode *const n = static_cast<WbBaseNode *>(it.next());
    if (n->isSuitableForInsertionInBoundingObject(warning) == false)
      return false;
  }

  return true;
}

bool WbGroup::isAValidBoundingObject(bool checkOde, bool warning) const {
  // Checks dimensions of bounding Geometries
  if (isSuitableForInsertionInBoundingObject(warning) == false)
    return false;

  WbMFNode::Iterator it(*mChildren);
  // Checks if there is at least one Geometry (with valid dimensions)
  while (it.hasNext()) {
    const WbBaseNode *const n = static_cast<WbBaseNode *>(it.next());
    if (n->isAValidBoundingObject(checkOde, false))
      return true;
  }

  return false;
}

void WbGroup::descendantNodeInserted(WbBaseNode *decendant) {
  if (!parentNode())
    return;

  WbGroup *pg = dynamic_cast<WbGroup *>(parentNode());
  if (pg) {
    pg->descendantNodeInserted(decendant);
    return;
  }

  if (dynamic_cast<WbBasicJoint *>(parentNode()))
    emit notifyParentJoint(decendant);
  else if (dynamic_cast<WbSlot *>(parentNode()))
    emit notifyParentSlot(decendant);
}

void WbGroup::insertChildFromSlotOrJoint(WbBaseNode *decendant) {
  descendantNodeInserted(decendant);
  emit childAdded(decendant);
}

void WbGroup::insertChildPrivate(int index) {
  WbBaseNode *child = static_cast<WbBaseNode *>(mChildren->item(index));
  if (child->isPostFinalizedCalled() && mBoundingSphere)
    mBoundingSphere->addSubBoundingSphere(child->boundingSphere());
  emit childAdded(child);
  descendantNodeInserted(child);

  if (isPostFinalizedCalled())
    connect(child, &WbBaseNode::finalizationCompleted, this, &WbGroup::monitorChildFinalization);
}

void WbGroup::monitorChildFinalization(WbBaseNode *child) {
  disconnect(child, &WbBaseNode::finalizationCompleted, this, &WbGroup::monitorChildFinalization);
  if (mBoundingSphere)
    mBoundingSphere->addSubBoundingSphere(child->boundingSphere());
  emit finalizedChildAdded(child);
}

bool WbGroup::shallExport() const {
  return !mChildren->isEmpty();
}

void WbGroup::reset(const QString &id) {
  WbBaseNode::reset(id);
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    it.next()->reset(id);
}

void WbGroup::save(const QString &id) {
  WbBaseNode::save(id);
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    it.next()->save(id);
}

void WbGroup::forwardJerk() {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbGroup *const child = dynamic_cast<WbGroup *>(it.next());
    if (child)
      child->forwardJerk();
  }
}

///////////////////
// Hidden fields //
///////////////////

bool WbGroup::restoreHiddenKinematicParameters(const HiddenKinematicParametersMap &map, int &counter) {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbGroup *const g = dynamic_cast<WbGroup *>(it.next());
    if (!g)
      continue;

    if (!g->restoreHiddenKinematicParameters(map, counter))
      return false;
  }

  return true;
}

bool WbGroup::resetHiddenKinematicParameters() {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbGroup *const g = dynamic_cast<WbGroup *>(it.next());
    if (!g)
      continue;
    if (!g->resetHiddenKinematicParameters())
      return false;
  }

  return true;
}

void WbGroup::collectHiddenKinematicParameters(HiddenKinematicParametersMap &map, int &counter) const {
  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    WbGroup *const g = dynamic_cast<WbGroup *>(it.next());
    if (g)
      g->collectHiddenKinematicParameters(map, counter);
  }
}

void WbGroup::writeParameters(WbVrmlWriter &writer) const {
  if (!isProtoParameterNode()) {
    HiddenKinematicParametersMap map;
    int counter = 0;
    collectHiddenKinematicParameters(map, counter);
    const HiddenKinematicParametersMap::const_iterator end = map.constEnd();
    for (HiddenKinematicParametersMap::const_iterator i = map.constBegin(); i != end; ++i) {
      const HiddenKinematicParameters *const hkp = i.value();
      assert(hkp);
      const int solidIndex = i.key();
      const WbVector3 *const t = hkp->translation();
      if (t) {
        writer.writeFieldStart(QString("hidden translation_%1").arg(solidIndex), true);
        writer << *t;
        writer.writeFieldEnd(true);
      }
      const WbRotation *const r = hkp->rotation();
      if (r) {
        writer.writeFieldStart(QString("hidden rotation_%1").arg(solidIndex), true);
        writer << *r;
        writer.writeFieldEnd(true);
      }
      const PositionMap *const m = hkp->positions();
      if (m) {
        const PositionMap::const_iterator hkpEnd = m->constEnd();
        for (PositionMap::const_iterator it = m->constBegin(); it != hkpEnd; ++it) {
          const WbVector3 *const p = it.value();
          assert(p);
          const int jointIndex = it.key();
          for (int j = 0; j < 3; ++j) {
            const double pj = (*p)[j];
            if (!std::isnan(pj)) {
              QString axisIndex;
              if (j > 0)
                axisIndex.setNum(j + 1);
              writer.writeFieldStart(QString("hidden position%1_%2_%3").arg(axisIndex).arg(solidIndex).arg(jointIndex), true);
              writer << WbPrecision::doubleToString(pj, WbPrecision::DOUBLE_MAX);
              writer.writeFieldEnd(true);
            }
          }
        }
      }

      const WbVector3 *const l = hkp->linearVelocity();
      if (l) {
        writer.writeFieldStart(QString("hidden linearVelocity_%1").arg(solidIndex), true);
        writer << *l;
        writer.writeFieldEnd(true);
      }

      const WbVector3 *const a = hkp->angularVelocity();
      if (a) {
        writer.writeFieldStart(QString("hidden angularVelocity_%1").arg(solidIndex), true);
        writer << *a;
        writer.writeFieldEnd(true);
      }
    }
    qDeleteAll(map);
    map.clear();
  }

  WbNode::writeParameters(writer);
}

void WbGroup::readHiddenKinematicParameter(WbField *field) {
  createHiddenKinematicParameter(field, mHiddenKinematicParametersMap);
}

////////////
// Export //
////////////

void WbGroup::exportBoundingObjectToX3D(WbVrmlWriter &writer) const {
  assert(writer.isX3d());

  writer << "<Group>";

  WbMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    const WbNode *const childNode = static_cast<WbNode *>(it.next());
    const WbGeometry *const childGeom = dynamic_cast<const WbGeometry *>(childNode);

    if (childGeom)
      writer << "<Shape>";

    childNode->exportBoundingObjectToX3D(writer);

    if (childGeom)
      writer << "</Shape>";
  }

  writer << "</Group>";
}
