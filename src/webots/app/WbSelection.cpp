// Copyright 1996-2023 Cyberbotics Ltd.
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

#include "WbAbstractTransform.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSimulationWorld.hpp"
#include "WbSolid.hpp"
#include "WbWrenRenderingContext.hpp"

WbSelection *WbSelection::cInstance = NULL;

WbSelection::WbSelection() :
  QObject(),
  mSelectedAbstractTransform(NULL),
  mSelectedNode(NULL),
  mResizeHandlesEnabledFromSceneTree(false) {
  assert(cInstance == NULL);
  cInstance = this;
}

WbSelection::~WbSelection() {
  cInstance = NULL;
}

WbSolid *WbSelection::selectedSolid() const {
  return dynamic_cast<WbSolid *>(mSelectedAbstractTransform);
}

void WbSelection::selectNodeFromSceneTree(WbBaseNode *node) {
  if (mSelectedNode == node)
    return;
  selectNode(node);
  emit selectionChangedFromSceneTree(mSelectedAbstractTransform);
}

void WbSelection::selectTransformFromView3D(WbAbstractTransform *t, bool handlesDisabled) {
  WbBaseNode *node = t == NULL ? NULL : t->baseNode();
  if (mSelectedNode == node)
    return;
  selectNode(node, handlesDisabled);
  emit selectionChangedFromView3D(mSelectedAbstractTransform);
  if (mSelectedAbstractTransform)
    updateHandlesScale();
}

void WbSelection::selectNode(WbBaseNode *n, bool handlesDisabled) {
  const bool transformChanged =
    ((n == NULL) != (mSelectedAbstractTransform == NULL)) || (n == NULL || n != mSelectedAbstractTransform->baseNode());
  if (mSelectedNode) {
    // unselect previously selected node
    updateMatterSelection(false);

    if (!mSelectedNode->isBeingDeleted()) {
      disconnect(mSelectedNode, &WbBaseNode::destroyed, this, &WbSelection::clear);
      setUniformConstraintForResizeHandles(false);
      mSelectedNode->detachResizeManipulator();

      if (mSelectedAbstractTransform && transformChanged) {
        mSelectedAbstractTransform->detachTranslateRotateManipulator();
        disconnect(mSelectedAbstractTransform->baseNode(), &WbBaseNode::isBeingDestroyed, this, &WbSelection::clear);
      }
    }
  }

  mSelectedNode = n;
  if (transformChanged)
    mSelectedAbstractTransform = mSelectedNode ? dynamic_cast<WbAbstractTransform *>(mSelectedNode) : NULL;
  mResizeHandlesEnabledFromSceneTree = false;

  if (mSelectedNode) {
    // select newly selected node
    connect(mSelectedNode, &WbBaseNode::destroyed, this, &WbSelection::clear, Qt::UniqueConnection);
    updateMatterSelection(true);

    if (mSelectedAbstractTransform && transformChanged) {
      connect(mSelectedNode, &WbBaseNode::isBeingDestroyed, this, &WbSelection::clear, Qt::UniqueConnection);
      if (!handlesDisabled && !mSelectedNode->isUseNode() &&
          !WbNodeUtilities::isNodeOrAncestorLocked(mSelectedAbstractTransform->baseNode()))
        mSelectedAbstractTransform->attachTranslateRotateManipulator();
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
  if (!mSelectedAbstractTransform)
    return;
  WbMatter *const matter = dynamic_cast<WbMatter *>(mSelectedAbstractTransform);
  if (matter != NULL)
    matter->propagateBoundingObjectMaterialUpdate();
}

bool WbSelection::isObjectMotionAllowed() const {
  if (!mSelectedAbstractTransform)
    return false;

  WbBaseNode *topNode = mSelectedAbstractTransform->baseNode();
  WbAbstractTransform *topTransform = NULL;
  if (!topNode->isTopLevel()) {
    WbSolid *solid = WbNodeUtilities::findUppermostSolid(topNode);
    if (solid)
      topTransform = dynamic_cast<WbAbstractTransform *>(topNode);
    else
      return false;
  } else
    topTransform = dynamic_cast<WbAbstractTransform *>(topNode);

  return topTransform && !WbNodeUtilities::isNodeOrAncestorLocked(topNode) && topTransform->isTranslationFieldVisible();
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
  if (mSelectedAbstractTransform)
    mSelectedAbstractTransform->updateTranslateRotateHandlesSize();
}

void WbSelection::showResizeManipulatorFromSceneTree(bool enabled) {
  mSelectedNode->showResizeManipulator(enabled);
  mResizeHandlesEnabledFromSceneTree = enabled;
  emit visibleHandlesChanged();
}

void WbSelection::disableActiveManipulator() {
  if (!mSelectedAbstractTransform)
    return;

  mSelectedAbstractTransform->detachResizeManipulator();
  mSelectedAbstractTransform->detachTranslateRotateManipulator();
}

void WbSelection::restoreActiveManipulator() {
  if (!mSelectedAbstractTransform || mSelectedNode->isUseNode() ||
      WbNodeUtilities::isNodeOrAncestorLocked(mSelectedAbstractTransform->baseNode()))
    return;

  if (mResizeHandlesEnabledFromSceneTree)
    mSelectedAbstractTransform->attachResizeManipulator();
  else
    mSelectedAbstractTransform->attachTranslateRotateManipulator();
}

bool WbSelection::showResizeManipulatorFromView3D(bool enabled) {
  // only Transform or Solid nodes can be selected from the 3D view
  WbAbstractTransform *t = selectedAbstractTransform();
  if (!t || mResizeHandlesEnabledFromSceneTree || !mSelectedNode || mSelectedNode->isUseNode() || !t->hasResizeManipulator() ||
      WbNodeUtilities::isNodeOrAncestorLocked(t->baseNode()))
    return false;

  if (enabled) {
    t->detachTranslateRotateManipulator();
    t->attachResizeManipulator();
  } else {
    t->detachResizeManipulator();
    t->attachTranslateRotateManipulator();
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
