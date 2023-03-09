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

#ifndef WB_TREE_ITEM_HPP
#define WB_TREE_ITEM_HPP

//
// Description: an object that represent a single line in the Scene Tree
//   A Scene Tree line can be:
//     1. a node, e.g. "WorldInfo"
//     2. a field, e.g. "boundingObject NULL", "color 0 1 0"
//     3. an item, e.g. "Author: first name last name <e-mail>"
//

#include <QtCore/QObject>
#include <QtCore/QVector>

class WbNode;
class WbField;
class QPixmap;

class WbTreeItem : public QObject {
  Q_OBJECT

public:
  explicit WbTreeItem(WbNode *node);
  explicit WbTreeItem(WbField *field);
  WbTreeItem(WbField *field, int index);
  virtual ~WbTreeItem();

  WbTreeItem *parent() const { return mParent; }
  void appendChild(WbTreeItem *const child) {
    mChildren.append(child);
    child->mParent = this;
  }
  void insertChild(int index, WbTreeItem *const child) {
    mChildren.insert(index, child);
    child->mParent = this;
  }
  WbTreeItem *child(int row) const { return mChildren.value(row); }
  int childCount() const { return mChildren.count(); }
  void deleteChild(int row);
  void deleteAllChildren();
  QString data() const;
  const QPixmap &pixmap() const;
  const QString &info() const;
  bool isDefault() const;
  bool canDelete() const;
  void del();
  bool canInsert() const;
  bool canCopy() const;
  bool canCut() const;
  int row() const;
  bool isNode() const { return mType == NODE; }
  bool hasNode() const { return mType == NODE || isSFNode(); }
  bool isField() const { return mType == FIELD; }
  bool isSFNode() const;
  bool isItem() const { return mType == ITEM; }
  bool isInvalid() const { return mType == INVALID; }
  WbField *field() const { return (mType == FIELD || mType == ITEM) ? mField : NULL; }
  WbNode *node() const;
  int itemIndex(const WbTreeItem *item) const;
  WbTreeItem *lastChild() const;

  bool isDataRefreshNeeded() const { return mIsDataRefreshNeeded; }
  void setDataRefreshNeeded(bool b) { mIsDataRefreshNeeded = b; }
  void refreshData();

  static void enableUpdates(bool enabled);

signals:
  void dataChanged();
  void childrenNeedDeletion(int row, int count);
  void rowsInserted(int row, int count);

public slots:
  void propagateDataChange();
  int makeInvalid();

private slots:
  void sfnodeChanged();
  void emitChildNeedsDeletion(int row);
  void emitDeleteAllChildren();
  void addChild(int row);

private:
  enum Type { ROOT, NODE, FIELD, ITEM, INVALID };

  Type mType;
  WbTreeItem *mParent;
  QVector<WbTreeItem *> mChildren;
  union {
    WbNode *mNode;
    WbField *mField;
  };
  bool mIsDataRefreshNeeded;

  static const QStringList FIXED_ROWS_MFFIELD;
  bool isFixedRowsMFitem() const;
  bool isNonEmptyFixedRowsMFfield() const;
};

#endif
