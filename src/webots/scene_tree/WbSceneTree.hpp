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

#ifndef WB_SCENE_TREE_HPP
#define WB_SCENE_TREE_HPP

//
// Description: GUI part of the Scene Tree, contains the toolbar to edit the Scene Tree
//

#include "WbActionManager.hpp"

#include <QtCore/QModelIndex>
#include <QtWidgets/QWidget>

class WbAbstractPose;
class WbBaseNode;
class WbClipboard;
class WbField;
class WbFieldEditor;
class WbNode;
class WbSceneTreeModel;
class WbSourceFileEditor;
class WbTreeItem;
class WbTreeView;
class WbWorld;

class QModelIndex;
class QSplitter;
class QPushButton;

struct TreeItemState;

// cppcheck-suppress noConstructor
class WbSceneTree : public QWidget {
  Q_OBJECT
  Q_PROPERTY(int handleWidth MEMBER mHandleWidth READ handleWidth WRITE setHandleWidth)

public:
  explicit WbSceneTree(QWidget *parent = NULL);
  virtual ~WbSceneTree();

  void setWorld(WbWorld *world);
  WbSourceFileEditor *sourceFileEditor() const;

  void cleanup();

  void prepareWorldLoading();
  void applyChanges();

  // save/restore splitter perspective
  QByteArray saveState() const;
  void restoreState(QByteArray state);
  void restoreFactoryLayout();

  int &handleWidth() { return mHandleWidth; }
  void setHandleWidth(const int &handleWidth) { mHandleWidth = handleWidth; }

public slots:
  void selectPose(WbAbstractPose *p);
  void updateValue();
  void updateApplicationActions();
  void updateSelection();

signals:
  void valueChangedFromGui();
  void nodeSelected(WbBaseNode *n);
  void editRequested(const QString &filePath, bool modify = false, bool isRobot = false);
  void documentationRequest(const QString &book, const QString &page, bool visible);

private slots:
  void handleUserCommand(WbAction::WbActionKind actionKind);
  void reset();
  void transform(const QString &modelName);
  void convertToBaseNode();
  void convertRootToBaseNode();
  void moveViewpointToObject();
  void addNew();
  void startWatching(const QModelIndex &index);
  void stopWatching(const QModelIndex &index);
  void handleRowRemoval(const QModelIndex &parentIndex, int start, int end);
  void refreshItems();
  void handleDoubleClickOrEnterPress();
  void editFileFromFieldEditor(const QString &fileName);

  void prepareNodeRegeneration(WbNode *node, bool nested);
  void abortNodeRegeneration();
  void applyNodeRegeneration(WbNode *node);
  void refreshTreeView();

  void help();
  void exportUrdf();
  void openProtoInTextEditor();
  void editProtoInTextEditor();
  void openTemplateInstanceInTextEditor();
  void showFieldEditor(bool force = false);

  void del(WbNode *nodeToDel = NULL);

private:
  QSplitter *mSplitter;
  QString mWorldFileName;
  WbSceneTreeModel *mModel;
  WbTreeItem *mSelectedItem;
  QPushButton *mExternProtoButton;
  WbTreeView *mTreeView;
  WbFieldEditor *mFieldEditor;
  bool mRowsAreAboutToBeRemoved;
  QWidget *mFocusWidgetBeforeNodeRegeneration;

  WbActionManager *mActionManager;
  WbClipboard *mClipboard;
  int mHandleWidth;

  // Stuff about the recovery of the scene tree state after node regeneration.
  bool mSelectionInsideTreeStateRecovery;
  WbTreeItem *mSelectionBeforeTreeStateRegeneration;
  TreeItemState *mTreeItemState;
  void storeTreeItemState(WbTreeItem *treeItem, TreeItemState *treeItemState);
  void restoreTreeItemState(WbTreeItem *treeItem, TreeItemState *treeItemState, WbBaseNode *lastNode);
  void cleanTreeItemState(TreeItemState *item);
  // void printTreeItemState(TreeItemState *item, int indentation = 0); // Debug function to print the stored tree state.

  void showExternProtoPanel();

  void restoreState(WbTreeView *t1, WbTreeView *t2, const QModelIndex &i1, const QModelIndex &i2);
  void updateToolbar();
  bool isPasteAllowed();
  void pasteInSFValue();
  void pasteInMFValue();
  void clearSelection();
  bool isIndexAncestorOfCurrentIndex(const QModelIndex &index, int start, int end);
  void convertProtoToBaseNode(bool rootOnly);
  bool insertInertiaMatrix(const WbField *selectedField);
  void cut();
  void copy();
  void paste();
  void enableObjectViewActions(bool enabled);
};

#endif
