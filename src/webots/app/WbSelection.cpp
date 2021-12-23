// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbSelection.hpp"

#include "WbAbstractPose.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSimulationWorld.hpp"
#include "WbSolid.hpp"
#include "WbWrenRenderingContext.hpp"

WbSelection *WbSelection::cInstance = NULL;

WbSelection::WbSelection() :
  QObject(),
  mSelectedAbstractPose(NULL),
  mSelectedNode(NULL),
  mResizeHandlesEnabledFromSceneTree(false) {
  assert(cInstance == NULL);
  cInstance = this;
}

WbSelection::~WbSelection() {
  cInstance = NULL;
}

WbSolid *WbSelection::selectedSolid() const {
  return dynamic_cast<WbSolid *>(mSelectedAbstractPose);
}

void WbSelection::selectNodeFromSceneTree(WbBaseNode *node) {
  if (mSelectedNode == node)
    return;
  selectNode(node);
  emit selectionChangedFromSceneTree(mSelectedAbstractPose);
}

void WbSelection::selectPoseFromView3D(WbAbstractPose *p, bool handlesDisabled) {
  WbBaseNode *node = p == NULL ? NULL : p->baseNode();
  if (mSelectedNode == node)
    return;
  selectNode(node, handlesDisabled);
  emit selectionChangedFromView3D(mSelectedAbstractPose);
  if (mSelectedAbstractPose)
    updateHandlesScale();
}

void WbSelection::selectNode(WbBaseNode *n, bool handlesDisabled) {
  const bool transformChanged =
    ((n == NULL) != (mSelectedAbstractPose == NULL)) || (n == NULL || n != mSelectedAbstractPose->baseNode());
  if (mSelectedNode) {
    // unselect previously selected node
    updateMatterSelection(false);

    if (!mSelectedNode->isBeingDeleted()) {
      disconnect(mSelectedNode, &WbBaseNode::destroyed, this, &WbSelection::clear);
      setUniformConstraintForResizeHandles(false);
      mSelectedNode->detachResizeManipulator();

      if (mSelectedAbstractPose && transformChanged) {
        mSelectedAbstractPose->detachTranslateRotateManipulator();
        disconnect(mSelectedAbstractPose->baseNode(), &WbBaseNode::isBeingDestroyed, this, &WbSelection::clear);
      }
    }
  }

  mSelectedNode = n;
  if (transformChanged)
    mSelectedAbstractPose = mSelectedNode ? dynamic_cast<WbAbstractPose *>(mSelectedNode) : NULL;
  mResizeHandlesEnabledFromSceneTree = false;

  if (mSelectedNode) {
    // select newly selected node
    connect(mSelectedNode, &WbBaseNode::destroyed, this, &WbSelection::clear, Qt::UniqueConnection);
    updateMatterSelection(true);

    if (mSelectedAbstractPose && transformChanged) {
      connect(mSelectedNode, &WbBaseNode::isBeingDestroyed, this, &WbSelection::clear, Qt::UniqueConnection);
      if (!handlesDisabled && !mSelectedNode->isUseNode() &&
          !WbNodeUtilities::isNodeOrAncestorLocked(mSelectedAbstractPose->baseNode()))
        mSelectedAbstractPose->attachTranslateRotateManipulator();
    }
  }
}

void WbSelection::updateMatterSelection(bool selected) {
  if (mSelectedNode == NULL)
    return;

  WbMatter *matter = dynamic_cast<WbMatter *>(mSelectedNode);
  if (matter == NULL)
    // show bounding objects of parent WbMatter
    matter = WbNodeUtilities::findUpperMatter(mSelectedNode);

  if (matter != NULL && !matter->isBeingDeleted()) {
    matter->select(selected);
    if (selected) {
      // collision and sleep materials update
      if (!WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS))
        matter->propagateBoundingObjectMaterialUpdate(selected);
    }
  }
}

void WbSelection::propagateBoundingObjectMaterialUpdate() {
  if (!mSelectedAbstractPose)
    return;
  WbMatter *const matter = dynamic_cast<WbMatter *>(mSelectedAbstractPose);
  if (matter != NULL)
    matter->propagateBoundingObjectMaterialUpdate();
}

bool WbSelection::isObjectMotionAllowed() const {
  if (!mSelectedAbstractPose)
    return false;

  WbBaseNode *topNode = mSelectedAbstractPose->baseNode();
  WbAbstractPose *topPose = NULL;
  if (!topNode->isTopLevel()) {
    WbSolid *solid = WbNodeUtilities::findUppermostSolid(topNode);
    if (solid)
      topPose = dynamic_cast<WbAbstractPose *>(topNode);
    else
      return false;
  } else
    topPose = dynamic_cast<WbAbstractPose *>(topNode);

  return topPose && !WbNodeUtilities::isNodeOrAncestorLocked(topNode) && topPose->isTranslationFieldVisible();
}

void WbSelection::clear() {
  if (mSelectedNode) {
    selectNode(NULL);
    emit selectionChangedFromSceneTree(NULL);
  }
}

void WbSelection::updateHandlesScale() {
  if (mResizeHandlesEnabledFromSceneTree && mSelectedNode)
    mSelectedNode->updateResizeHandlesSize();
  if (mSelectedAbstractPose)
    mSelectedAbstractPose->updateTranslateRotateHandlesSize();
}

void WbSelection::showResizeManipulatorFromSceneTree(bool enabled) {
  mSelectedNode->showResizeManipulator(enabled);
  mResizeHandlesEnabledFromSceneTree = enabled;
  emit visibleHandlesChanged();
}

void WbSelection::disableActiveManipulator() {
  if (!mSelectedAbstractPose)
    return;

  mSelectedAbstractPose->detachResizeManipulator();
  mSelectedAbstractPose->detachTranslateRotateManipulator();
}

void WbSelection::restoreActiveManipulator() {
  if (!mSelectedAbstractPose || mSelectedNode->isUseNode() ||
      WbNodeUtilities::isNodeOrAncestorLocked(mSelectedAbstractPose->baseNode()))
    return;

  if (mResizeHandlesEnabledFromSceneTree)
    mSelectedAbstractPose->attachResizeManipulator();
  else
    mSelectedAbstractPose->attachTranslateRotateManipulator();
}

bool WbSelection::showResizeManipulatorFromView3D(bool enabled) {
  // only Transform or Solid nodes can be selected from the 3D view
  WbAbstractPose *p = selectedAbstractPose();
  if (!p || mResizeHandlesEnabledFromSceneTree || !mSelectedNode || mSelectedNode->isUseNode() || !p->hasResizeManipulator() ||
      WbNodeUtilities::isNodeOrAncestorLocked(p->baseNode()))
    return false;

  if (enabled) {
    p->detachTranslateRotateManipulator();
    p->attachResizeManipulator();
  } else {
    p->detachResizeManipulator();
    p->attachTranslateRotateManipulator();
  }
  emit visibleHandlesChanged();
  return true;
}

void WbSelection::setUniformConstraintForResizeHandles(bool enable) {
  if (mResizeHandlesEnabledFromSceneTree) {
    mSelectedNode->setUniformConstraintForResizeHandles(enable);
    emit visibleHandlesChanged();
  }
}
