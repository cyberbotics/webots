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

#ifndef WB_SELECTION_HPP
#define WB_SELECTION_HPP

//
// Description: classes managing the selection of nodes
//

#include <QtCore/QObject>

class WbAbstractTransform;
class WbBaseNode;
class WbSolid;

class WbSelection : public QObject {
  Q_OBJECT

public:
  // unique selection instance (can be NULL)
  static WbSelection *instance() { return cInstance; }

  WbSelection();
  virtual ~WbSelection();

  // the currently selected transform in the scene tree
  WbAbstractTransform *selectedAbstractTransform() const { return mSelectedAbstractTransform; }

  // the currently selected solid in the scene tree
  WbSolid *selectedSolid() const;

  // the currently selected solid in the scene tree
  WbBaseNode *selectedNode() const { return mSelectedNode; }

  // check that the object containing the selected matter is allowed to be moved by the user
  bool isObjectMotionAllowed() const;

  void disableActiveManipulator();

  // enable or disable the resize handles and return if the action could be successfully completed
  bool showResizeManipulatorFromView3D(bool enabled);

  void showResizeManipulatorFromSceneTree(bool enabled);

  // if true set uniform constraint otherwise resets the default constraint
  void setUniformConstraintForResizeHandles(bool enable);

  bool resizeManipulatorEnabledFromSceneTree() { return mResizeHandlesEnabledFromSceneTree; }

  // the currently selected transform was reselected from the View 3D (we need to make sure the selection switches to the node
  // if a field is selected)
  void confirmSelectedTransformFromView3D() { emit selectionConfirmedFromView3D(mSelectedAbstractTransform); }

public slots:
  // select another node from the scene tree
  void selectNodeFromSceneTree(WbBaseNode *node);

  // select another matter from the 3D view
  void selectTransformFromView3D(WbAbstractTransform *t, bool handlesDisabled = false);

  // update handle size based on viewpoint camera distance
  void updateHandlesScale();

  // re-activate previously disabled manipulator
  void restoreActiveManipulator();

  // update the bounging object material of the selected matter
  void propagateBoundingObjectMaterialUpdate();

signals:
  // the selection has changed from the scene tree
  void selectionChangedFromSceneTree(WbAbstractTransform *t);

  // the selection has changed from the 3D view
  void selectionChangedFromView3D(WbAbstractTransform *t);

  // the same WbAbstractTransform was selected from the 3D view
  void selectionConfirmedFromView3D(WbAbstractTransform *t);

  // the visible handles of the selected node have changed
  void visibleHandlesChanged();

private:
  static WbSelection *cInstance;
  WbAbstractTransform *mSelectedAbstractTransform;
  WbBaseNode *mSelectedNode;  // node selected in the scene tree
  bool mResizeHandlesEnabledFromSceneTree;

  // select another node
  void selectNode(WbBaseNode *n, bool handlesDisabled = false);

  // update matter selection
  // if mSelectedNode is not an instance of WbMatter, then select upper matter
  void updateMatterSelection(bool selected);

private slots:
  void clear();
};

#endif
