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

#include "WbSceneTreeModel.hpp"

#include "WbField.hpp"
#include "WbGuiRefreshOracle.hpp"
#include "WbMFNode.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSFNode.hpp"
#include "WbSolid.hpp"
#include "WbTreeItem.hpp"
#include "WbWorld.hpp"

#include <QtGui/QPixmap>

#include <cassert>

WbSceneTreeModel::WbSceneTreeModel(WbGroup *worldRoot) : mRootItem(createItemForField(worldRoot->findField("children"))) {
}

WbSceneTreeModel::~WbSceneTreeModel() {
  delete mRootItem;
}

WbTreeItem *WbSceneTreeModel::createItemForNode(WbNode *node) {
  WbTreeItem *const item = new WbTreeItem(node);

  if (node)
    connect(node, &WbNode::defUseNameChanged, this, &WbSceneTreeModel::updateItemAndChildren);

  // Solid, Device, Joint and JointParameters USE nodes are made expandable and turned into non-USE nodes during dictionary
  // update
  if (node && (!node->isUseNode() || !WbNodeUtilities::isAValidUseableNode(node))) {
    const int n = node->numFields();
    const QVector<WbField *> &fields = node->fieldsOrParameters();
    for (int i = 0; i < n; ++i) {
      WbTreeItem *const child = createItemForField(fields[i]);
      if (child)
        item->appendChild(child);
    }
  }

  return item;
}

void WbSceneTreeModel::createChildrenItemForNode(WbNode *node) {
  assert(node);
  WbTreeItem *const item = findTreeItemFromNode(node, mRootItem);
  assert(item);
  const int n = node->numFields();
  const QVector<WbField *> &fields = node->fieldsOrParameters();
  for (int i = 0; i < n; ++i) {
    WbTreeItem *const child = createItemForField(fields[i]);
    if (child)
      item->appendChild(child);
  }
}

WbTreeItem *WbSceneTreeModel::createItemForField(WbField *field) {
  if (!field || field->isHidden())
    return NULL;

  WbTreeItem *const item = new WbTreeItem(field);

  const WbSFNode *const sfnode = dynamic_cast<WbSFNode *>(field->value());
  if (sfnode) {
    connect(item, &WbTreeItem::rowsInserted, this, &WbSceneTreeModel::insertItems);
    connect(item, &WbTreeItem::childrenNeedDeletion, this, &WbSceneTreeModel::removeItems);
    WbNode *const node = sfnode->value();
    if (node) {
      connect(node, &WbNode::defUseNameChanged, this, &WbSceneTreeModel::updateItemAndChildren);
      if (!node->isUseNode()) {
        const int n = node->numFields();
        const QVector<WbField *> &fields = node->fieldsOrParameters();
        for (int i = 0; i < n; ++i) {
          WbTreeItem *const child = createItemForField(fields[i]);
          if (child)
            item->appendChild(child);
        }
      }
    }
    return item;
  }

  connect(item, &WbTreeItem::rowsInserted, this, &WbSceneTreeModel::insertItems);
  connect(item, &WbTreeItem::childrenNeedDeletion, this, &WbSceneTreeModel::removeItems);

  const WbMFNode *const mfnode = dynamic_cast<WbMFNode *>(field->value());
  if (mfnode) {
    const int n = mfnode->size();
    for (int i = 0; i < n; ++i)
      item->appendChild(createItemForNode(mfnode->item(i)));
    return item;
  }

  const WbMultipleValue *const mvalue = dynamic_cast<WbMultipleValue *>(field->value());
  if (mvalue) {
    const int n = mvalue->size();
    for (int i = 0; i < n; ++i)
      item->appendChild(new WbTreeItem(field, 0));
    return item;
  }

  return item;
}

QVariant WbSceneTreeModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid())
    return QVariant();

  const WbTreeItem *const item = indexToItem(index);

  switch (role) {
    case Qt::DisplayRole:
      return QVariant(item->data());
    case Qt::DecorationRole:
      return QVariant(item->pixmap());
    case Qt::ToolTipRole:
      return QVariant(item->info());
    default:
      return QVariant();
  }
}

Qt::ItemFlags WbSceneTreeModel::flags(const QModelIndex &index) const {
  if (!index.isValid())
    return Qt::ItemFlags();

  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

QModelIndex WbSceneTreeModel::index(int row, int column, const QModelIndex &parent) const {
  if (!hasIndex(row, column, parent))
    return QModelIndex();

  const WbTreeItem *const parentItem = parent.isValid() ? indexToItem(parent) : mRootItem;
  WbTreeItem *const childItem = parentItem->child(row);

  return childItem ? createIndex(row, column, childItem) : QModelIndex();
}

QModelIndex WbSceneTreeModel::parent(const QModelIndex &index) const {
  if (!index.isValid())
    return QModelIndex();

  WbTreeItem *const childItem = indexToItem(index);
  WbTreeItem *const parentItem = childItem->parent();
  if (parentItem == mRootItem)
    return QModelIndex();

  return createIndex(parentItem->row(), 0, parentItem);
}

int WbSceneTreeModel::rowCount(const QModelIndex &parent) const {
  if (parent.column() > 0)
    return 0;

  const WbTreeItem *const parentItem = parent.isValid() ? indexToItem(parent) : mRootItem;

  return parentItem ? parentItem->childCount() : 0;
}

void WbSceneTreeModel::startWatching(const QModelIndex &index) {
  const WbTreeItem *const item = indexToItem(index);

  if (!item)
    return;

  const int count = item->childCount();

  for (int i = 0; i < count; ++i) {
    const WbTreeItem *const childItem = item->child(i);
    connect(childItem, &WbTreeItem::dataChanged, this, &WbSceneTreeModel::updateData);
  }
}

void WbSceneTreeModel::stopWatching(const QModelIndex &index) {
  const WbTreeItem *const item = indexToItem(index);

  const int count = item->childCount();
  for (int i = 0; i < count; ++i) {
    const WbTreeItem *const childItem = item->child(i);
    disconnect(childItem, &WbTreeItem::dataChanged, this, &WbSceneTreeModel::updateData);
  }
}

WbTreeItem *WbSceneTreeModel::indexToItem(const QModelIndex &index) const {
  return index.isValid() ? static_cast<WbTreeItem *>(index.internalPointer()) : mRootItem;
}

QModelIndex WbSceneTreeModel::itemToIndex(const WbTreeItem *item) const {
  if (item == mRootItem)
    return QModelIndex();

  // create a path from the root item to the specified item
  QList<const WbTreeItem *> ancestors;
  const WbTreeItem *parentItem = item;
  while (parentItem) {
    ancestors.prepend(parentItem);
    parentItem = parentItem->parent();
  }

  // traverse the model from the root index, using the above path
  QModelIndex parentIndex;
  const int as = ancestors.size();
  for (int i = 0; i < as; ++i) {
    const int rows = rowCount(parentIndex);
    for (int j = 0; j < rows; ++j) {
      QModelIndex childIndex = index(j, 0, parentIndex);
      const WbTreeItem *const childItem = indexToItem(childIndex);

      // see if the child item is in the path
      if (childItem == ancestors[i]) {
        // enter this branch from the model
        parentIndex = childIndex;
        break;
      }
    }
  }

  return parentIndex;
}

void WbSceneTreeModel::updateData() {
  WbTreeItem *const item = static_cast<WbTreeItem *>(sender());

  // don't update scene tree item view at each step
  if (!WbGuiRefreshOracle::instance()->canRefreshNow()) {
    item->setDataRefreshNeeded(true);
    return;
  }

  item->setDataRefreshNeeded(false);
  QModelIndex modelIndex = itemToIndex(item);
  emit dataChanged(modelIndex, modelIndex);

  // Without the following line, some values such as MFVector2
  // were not updated correctly when modified from editor
  emit layoutChanged();
}

void WbSceneTreeModel::updateAllSceneTreeValues() {
  const int nRows = mRootItem->childCount();
  QModelIndex topLeft = index(0, 0);
  QModelIndex bottomRight = index(nRows - 1, 0);
  emit dataChanged(topLeft, bottomRight);
}

void WbSceneTreeModel::updateItem(WbTreeItem *item) {
  QModelIndex itemModelIndex = itemToIndex(item);
  emit dataChanged(itemModelIndex, itemModelIndex);
}

void WbSceneTreeModel::updateItemAndChildren(WbNode *node, bool createChildren) {
  if (createChildren)
    createChildrenItemForNode(node);
  QModelIndex itemModelIndex = findModelIndexFromNode(node);
  emit dataChanged(itemModelIndex, itemModelIndex);
}

void WbSceneTreeModel::removeItems(int row, int count) {
  const WbTreeItem *const item = static_cast<WbTreeItem *>(sender());
  const QModelIndex &modelIndex = itemToIndex(item);
  emit rowsAboutToBeRemovedSoon(modelIndex, row, row + count - 1);
  removeRows(row, count, modelIndex);
}

bool WbSceneTreeModel::removeRows(int row, int count, const QModelIndex &parent) {
  beginRemoveRows(parent, row, row + count - 1);

  WbTreeItem *const item = indexToItem(parent);
  if (item->isSFNode())
    item->deleteAllChildren();
  else
    item->deleteChild(row);

  endRemoveRows();

  return true;
}

void WbSceneTreeModel::insertItems(int position, int count) {
  const WbTreeItem *const item = static_cast<WbTreeItem *>(sender());
  const QModelIndex &modelIndex = itemToIndex(item);
  insertRows(position, count, modelIndex);
}

bool WbSceneTreeModel::insertRows(int row, int count, const QModelIndex &parent) {
  beginInsertRows(parent, row, row + count - 1);

  WbTreeItem *const parentItem = indexToItem(parent);

  const WbSFNode *const sfnode = dynamic_cast<WbSFNode *>(parentItem->field()->value());
  if (sfnode) {
    WbNode *const node = sfnode->value();
    if (node) {
      if (!node->isUseNode()) {
        const int n = node->numFields();
        for (int i = 0; i < n; ++i) {
          WbTreeItem *const childItem = createItemForField(node->field(i));
          if (childItem) {
            parentItem->appendChild(childItem);
            connect(childItem, &WbTreeItem::dataChanged, this, &WbSceneTreeModel::updateData);
          }
        }
      }

      connect(node, &WbNode::defUseNameChanged, this, &WbSceneTreeModel::updateItemAndChildren, Qt::UniqueConnection);
    }

    // signal WbTreeView
    emit itemInserted(parent);
  } else {
    const WbMFNode *const mfnode = dynamic_cast<WbMFNode *>(parentItem->field()->value());

    for (int pos = row; pos < count + row; ++pos) {
      WbTreeItem *const childItem = mfnode ? createItemForNode(mfnode->item(pos)) : new WbTreeItem(parentItem->field(), 0);

      parentItem->insertChild(pos, childItem);
      connect(childItem, &WbTreeItem::dataChanged, this, &WbSceneTreeModel::updateData);

      // signal WbTreeView
      QModelIndex childIndex = itemToIndex(childItem);
      emit itemInserted(childIndex);
    }
  }

  endInsertRows();

  emit dataChanged(parent, parent);
  return true;
}

QModelIndex WbSceneTreeModel::findModelIndexFromNode(WbNode *node, WbTreeItem *current) const {
  if (current->isNode() && current->node() == node)
    return itemToIndex(current);

  if (current->isField() && current->field()->type() == WB_SF_NODE) {
    const WbSFNode *const n = static_cast<WbSFNode *>(current->field()->value());
    if (n->value() == node)
      return itemToIndex(current);
  }

  const int nChild = current->childCount();
  for (int i = 0; i < nChild; ++i) {
    QModelIndex modelIndex = findModelIndexFromNode(node, current->child(i));
    if (modelIndex.isValid())
      return modelIndex;
  }

  return QModelIndex();
}

WbTreeItem *WbSceneTreeModel::findTreeItemFromNode(WbNode *node, WbTreeItem *current) {
  if (current->isNode() && current->node() == node)
    return current;

  if (current->isSFNode()) {
    const WbNode *const candidate = static_cast<WbSFNode *>(current->field()->value())->value();
    if (node == candidate)
      return current;
  }

  const int n = current->childCount();
  for (int i = 0; i < n; ++i) {
    WbTreeItem *const item = findTreeItemFromNode(node, current->child(i));
    if (item)
      return item;
  }

  return NULL;
}

WbTreeItem *WbSceneTreeModel::findUpperNodeItem(const WbTreeItem *item) const {
  const WbTreeItem *i = item;
  while (!i->isNode() && !i->isSFNode()) {
    i = i->parent();
    if (!i)
      return NULL;
  }
  return const_cast<WbTreeItem *>(i);
}

QModelIndex WbSceneTreeModel::findModelIndexFromField(WbField *field, WbTreeItem *current) const {
  if (current->isField() && current->field() == field)
    return itemToIndex(current);

  const int nChild = current->childCount();
  for (int i = 0; i < nChild; ++i) {
    QModelIndex modelIndex = findModelIndexFromField(field, current->child(i));
    if (modelIndex.isValid())
      return modelIndex;
  }

  return QModelIndex();
}

/////////////////////////////////////////////
// Utility functions related to item index //
/////////////////////////////////////////////

int WbSceneTreeModel::itemToTreeIndex(WbTreeItem *item) const {
  WbTreeItem *const targetItem = item;
  bool itemFound = false;
  int itemIndex = 0;
  const int n = mRootItem->childCount();

  for (int i = 0; !itemFound && i < n; ++i)
    treeIndex(mRootItem->child(i), targetItem, itemFound, itemIndex);

  return itemIndex;
}

void WbSceneTreeModel::treeIndex(const WbTreeItem *const currentItem, const WbTreeItem *const targetItem, bool &itemFound,
                                 int &index) {
  ++index;

  if (currentItem == targetItem) {
    itemFound = true;
    return;
  }

  const int n = currentItem->childCount();
  for (int i = 0; !itemFound && i < n; ++i)
    treeIndex(currentItem->child(i), targetItem, itemFound, index);
}

WbTreeItem *WbSceneTreeModel::treeIndexToItem(int targetIndex) const {
  int itemIndex = targetIndex;
  const int n = mRootItem->childCount();

  for (int i = 0; (itemIndex > 0) && i < n; ++i) {
    WbTreeItem *currentItem = treeIndexToItem(mRootItem->child(i), itemIndex);
    if (itemIndex == 0)
      return currentItem;
  }

  return NULL;
}

WbTreeItem *WbSceneTreeModel::treeIndexToItem(WbTreeItem *currentItem, int &index) {
  --index;

  if (index == 0)
    return currentItem;

  const int n = currentItem->childCount();
  for (int i = 0; (index > 0) && i < n; ++i) {
    WbTreeItem *item = treeIndexToItem(currentItem->child(i), index);
    if (index == 0)
      return item;
  }

  return NULL;
}

// Update the scene tree layout
void WbSceneTreeModel::emitLayoutChanged() {
  emit layoutChanged();
}
