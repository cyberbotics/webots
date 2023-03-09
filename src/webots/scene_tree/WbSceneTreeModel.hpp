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

#ifndef WB_SCENE_TREE_MODEL_HPP
#define WB_SCENE_TREE_MODEL_HPP

//
// Description: QAbstractItemModel for the WbSceneTree: defines how the object must be organized in the Scene tree
//

#include <QtCore/QAbstractItemModel>
#include <QtCore/QMap>
#include <QtCore/QModelIndex>
#include <QtCore/QVariant>

class WbTreeItem;
class WbGroup;
class WbNode;
class WbField;

class WbSceneTreeModel : public QAbstractItemModel {
  Q_OBJECT

public:
  explicit WbSceneTreeModel(WbGroup *worldRoot);
  virtual ~WbSceneTreeModel();

  // inherited from QAbstractItemModel
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
  Qt::ItemFlags flags(const QModelIndex &index) const override;
  QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
  QModelIndex parent(const QModelIndex &index) const override;
  int rowCount(const QModelIndex &parent) const override;
  int columnCount(const QModelIndex &parent) const override { return 1; }
  bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex()) override;
  bool insertRows(int row, int count, const QModelIndex &parent) override;

  void startWatching(const QModelIndex &index);
  void stopWatching(const QModelIndex &index);

  // Index mappings
  WbTreeItem *indexToItem(const QModelIndex &index) const;
  int itemToTreeIndex(WbTreeItem *item) const;
  int modelIndexToTreeIndex(const QModelIndex &index) const { return itemToTreeIndex(indexToItem(index)); }
  QModelIndex treeIndexToModelIndex(int index) const { return itemToIndex(treeIndexToItem(index)); }
  QModelIndex itemToIndex(const WbTreeItem *item) const;

  WbTreeItem *rootItem() const { return mRootItem; }

  WbTreeItem *findUpperNodeItem(const WbTreeItem *item) const;
  QModelIndex findModelIndexFromNode(WbNode *node, WbTreeItem *current) const;
  QModelIndex findModelIndexFromNode(WbNode *node) const { return findModelIndexFromNode(node, mRootItem); }
  static WbTreeItem *findTreeItemFromNode(WbNode *node, WbTreeItem *current);

  QModelIndex findModelIndexFromField(WbField *field, WbTreeItem *current) const;
  QModelIndex findModelIndexFromField(WbField *field) const { return findModelIndexFromField(field, mRootItem); }

  void createChildrenItemForNode(WbNode *node);

  void emitLayoutChanged();
  void updateAllSceneTreeValues();
  void updateItem(WbTreeItem *item);

signals:
  void itemInserted(const QModelIndex &index);
  void rowsAboutToBeRemovedSoon(const QModelIndex &parent, int start, int end);

private slots:
  void updateData();
  void removeItems(int row, int count);
  void insertItems(int position, int count);
  void updateItemAndChildren(WbNode *node, bool createChildren);

private:
  WbTreeItem *mRootItem;
  WbTreeItem *treeIndexToItem(int targetIndex) const;
  static WbTreeItem *treeIndexToItem(WbTreeItem *currentItem, int &index);
  static void treeIndex(const WbTreeItem *const currentItem, const WbTreeItem *const targetItem, bool &itemFound, int &index);

  WbTreeItem *createItemForNode(WbNode *node);
  WbTreeItem *createItemForField(WbField *field);
};

#endif
