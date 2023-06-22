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

#include "WbTreeItem.hpp"

#include "WbBackground.hpp"
#include "WbField.hpp"
#include "WbGroup.hpp"
#include "WbMFNode.hpp"
#include "WbMultipleValue.hpp"
#include "WbNode.hpp"
#include "WbNodeModel.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSFNode.hpp"
#include "WbSFRotation.hpp"
#include "WbSFVector2.hpp"
#include "WbViewpoint.hpp"
#include "WbWorldInfo.hpp"

#include <QtGui/QPixmap>

#include <cassert>

static const QString EMPTY_STRING;
// MFFields with a fixed number of rows
const QStringList WbTreeItem::FIXED_ROWS_MFFIELD = QStringList() << "inertiaMatrix"
                                                                 << "centerOfMass";

static bool gUpdatesEnabled = true;

void WbTreeItem::enableUpdates(bool enabled) {
  gUpdatesEnabled = enabled;
}

WbTreeItem::WbTreeItem(WbNode *node) {
  mIsDataRefreshNeeded = false;
  mType = NODE;
  mParent = NULL;
  mNode = node;
  connect(mNode, &QObject::destroyed, this, &WbTreeItem::makeInvalid);
}

WbTreeItem::WbTreeItem(WbField *field) {
  mType = FIELD;
  mParent = NULL;
  mField = field;

  connect(mNode, &QObject::destroyed, this, &WbTreeItem::makeInvalid);

  WbValue *const value = mField->value();
  WbSingleValue *const singleValue = dynamic_cast<WbSingleValue *>(value);
  if (singleValue) {
    const WbSFNode *const sfnode = dynamic_cast<WbSFNode *>(value);
    if (sfnode) {
      connect(sfnode, &WbSFNode::changed, this, &WbTreeItem::sfnodeChanged);
      if (sfnode->value())
        connect(sfnode->value(), &WbNode::defUseNameChanged, this, &WbTreeItem::propagateDataChange, Qt::UniqueConnection);
    } else {
      // Main signal
      connect(singleValue, &WbSFNode::changed, this, &WbTreeItem::propagateDataChange);
      // Signal used by translation and rotation fields of Solids and position fields of Joints only
      const QString &fieldName = field->name();
      if (fieldName == "translation") {
        const WbSFVector3 *const translation = dynamic_cast<WbSFVector3 *>(singleValue);
        if (translation)
          connect(translation, &WbSFVector3::changedByOde, this, &WbTreeItem::propagateDataChange);
        else {
          const WbSFVector2 *const translation2 = dynamic_cast<WbSFVector2 *>(singleValue);
          if (translation2)
            connect(translation2, &WbSFVector2::changedByWebots, this, &WbTreeItem::propagateDataChange);
        }
      } else if (fieldName == "rotation") {
        const WbSFRotation *const rotation = dynamic_cast<WbSFRotation *>(singleValue);
        if (rotation)
          connect(rotation, &WbSFRotation::changedByOde, this, &WbTreeItem::propagateDataChange);
      } else if (fieldName == "position") {
        const WbSFDouble *const position = dynamic_cast<WbSFDouble *>(singleValue);
        if (position)
          connect(position, &WbSFDouble::changedByOde, this, &WbTreeItem::propagateDataChange);
      }
    }
    return;
  }

  const WbMultipleValue *const multipleValue = static_cast<WbMultipleValue *>(value);
  // slots are executed in the order they have been connected
  if (mField->type() == WB_MF_NODE) {
    connect(multipleValue, &WbMultipleValue::itemChanged, this, &WbTreeItem::emitChildNeedsDeletion);
    connect(multipleValue, &WbMultipleValue::itemChanged, this, &WbTreeItem::addChild);
  } else
    // otherwise there is no need to recreate the item when the value changes
    connect(multipleValue, &WbMultipleValue::itemChanged, this, &WbTreeItem::propagateDataChange);
  connect(multipleValue, &WbMultipleValue::itemRemoved, this, &WbTreeItem::emitChildNeedsDeletion);
  connect(multipleValue, &WbMultipleValue::cleared, this, &WbTreeItem::emitDeleteAllChildren);
  connect(multipleValue, &WbMultipleValue::itemInserted, this, &WbTreeItem::addChild);
}

WbTreeItem::WbTreeItem(WbField *field, int index) {
  mType = ITEM;
  mParent = NULL;
  mField = field;

  connect(mNode, &QObject::destroyed, this, &WbTreeItem::makeInvalid);
}

WbTreeItem::~WbTreeItem() {
  qDeleteAll(mChildren);
}

void WbTreeItem::propagateDataChange() {
  if (gUpdatesEnabled)
    emit dataChanged();
}

void WbTreeItem::refreshData() {
  if (gUpdatesEnabled) {
    mIsDataRefreshNeeded = false;
    emit dataChanged();
  }
}

QString WbTreeItem::data() const {
  if (!gUpdatesEnabled)
    return QString();

  switch (mType) {
    case NODE:
      return mNode->usefulName();
    case FIELD: {
      if (mField->isSingle())
        return QString("%1 %2").arg(mField->name(), mField->value()->toString(WbPrecision::GUI_LOW));
      else
        return mField->name();
    }
    case ITEM: {
      const WbMultipleValue *const value = dynamic_cast<WbMultipleValue *>(mField->value());
      int r = row();
      if (r >= 0 && r < value->size())
        return value->itemToString(r, WbPrecision::GUI_LOW);
      return EMPTY_STRING;
    }
    case INVALID:
      return EMPTY_STRING;
    default:
      assert(false);
      return EMPTY_STRING;
  }
}

const QPixmap &WbTreeItem::pixmap() const {
  static const QPixmap nodePixmap("enabledIcons:node.png");
  static const QPixmap fieldPixmap("enabledIcons:field.png");
  static const QPixmap protoPixmap("enabledIcons:proto.png");
  static const QPixmap nullPixmap;

  switch (mType) {
    case NODE: {
      if (mNode->isProtoInstance())
        return protoPixmap;
      else
        return nodePixmap;
    }
    case FIELD:
      if (isSFNode()) {
        if (node() && node()->isProtoInstance())
          return protoPixmap;
        else
          return nodePixmap;
      } else
        return fieldPixmap;
    case ITEM:
    case INVALID:
      return nullPixmap;
    default:
      assert(false);
      return nullPixmap;
  }
}

const QString &WbTreeItem::info() const {
  switch (mType) {
    case NODE:
      return mNode->info();
    case FIELD: {
      const WbSFNode *const sfnode = dynamic_cast<WbSFNode *>(mField->value());
      if (sfnode && sfnode->value())
        return sfnode->value()->info();
    }
    default:
      return EMPTY_STRING;
  }
}

bool WbTreeItem::isDefault() const {
  switch (mType) {
    case NODE:
      return mNode->isDefault();
    case FIELD:
    case ITEM:
      return mField->isDefault();
    case INVALID:
      return true;
    default:
      assert(false);
      return false;
  }
}

int WbTreeItem::row() const {
  if (!mParent)
    return 0;

  return mParent->mChildren.indexOf(const_cast<WbTreeItem *>(this));
}

bool WbTreeItem::isFixedRowsMFitem() const {
  assert(isItem());

  const WbTreeItem *const p = parent();
  if (p == NULL || !p->isField())
    return false;

  const WbField *f = p->field();
  if (f == NULL)
    return false;

  foreach (const QString name, FIXED_ROWS_MFFIELD) {
    if (f->name() == name)
      return true;
  }

  if (f->internalFields().size() > 0) {
    f = f->internalFields().at(0);
    if (f == NULL)
      return false;
    foreach (const QString name, FIXED_ROWS_MFFIELD) {
      if (f->name() == name)
        return true;
    }
  }

  return false;
}

bool WbTreeItem::isNonEmptyFixedRowsMFfield() const {
  assert(isField());

  const WbField *f = field();
  if (f == NULL || !f->isMultiple())
    return false;

  foreach (const QString name, FIXED_ROWS_MFFIELD) {
    if (f->name() == name)
      return !dynamic_cast<WbMultipleValue *>(f->value())->isEmpty();
  }

  if (f->internalFields().size() > 0) {
    f = f->internalFields().at(0);
    if (f == NULL)
      return false;

    foreach (const QString name, FIXED_ROWS_MFFIELD) {
      if (f->name() == name)
        return !dynamic_cast<WbMultipleValue *>(f->value())->isEmpty();
    }
  }

  return false;
}

bool WbTreeItem::canInsert() const {
  switch (mType) {
    case NODE:
      if (dynamic_cast<WbWorldInfo *>(mNode))
        return false;
      return true;
    case FIELD: {
      const WbSFNode *const sfnode = dynamic_cast<WbSFNode *>(mField->value());
      if (sfnode)
        return !sfnode->value();
      else {
        if (isNonEmptyFixedRowsMFfield())
          return false;
        return mField->isMultiple();
      }
    }
    case ITEM:
      if (isFixedRowsMFitem())
        return false;
      return mParent->isField() && mParent->mField->isMultiple();
    case INVALID:
      return false;
    default:
      assert(false);
      return false;
  }

  return false;
}

bool WbTreeItem::canCopy() const {
  switch (mType) {
    case NODE:
      if (dynamic_cast<WbWorldInfo *>(mNode))
        return false;
      if (dynamic_cast<WbViewpoint *>(mNode))
        return false;
      return true;
    case FIELD: {
      const WbSFNode *const sfnode = dynamic_cast<WbSFNode *>(mField->value());
      if (sfnode)
        return sfnode->value() && !sfnode->value()->isUseNode();
      else
        return mField->isSingle();
    }
    case ITEM:
      return true;
    case INVALID:
      return false;
    default:
      assert(false);
      return false;
  }

  return false;
}

bool WbTreeItem::canCut() const {
  switch (mType) {
    case NODE:
      if (dynamic_cast<WbWorldInfo *>(mNode))
        return false;
      if (dynamic_cast<WbViewpoint *>(mNode))
        return false;
      return true;
    case FIELD: {
      WbSFNode *sfnode = dynamic_cast<WbSFNode *>(mField->value());
      return sfnode && sfnode->value() && !sfnode->value()->isUseNode();
    }
    case ITEM:
      return true;
    case INVALID:
      return false;
    default:
      assert(false);
      return false;
  }

  return false;
}

bool WbTreeItem::canDelete() const {
  switch (mType) {
    case NODE: {
      if (dynamic_cast<WbWorldInfo *>(mNode))
        return false;
      if (dynamic_cast<WbViewpoint *>(mNode))
        return false;
      return true;
    }
    case FIELD: {
      WbSFNode *sfnode = dynamic_cast<WbSFNode *>(mField->value());
      return sfnode && sfnode->value() != NULL;
    }
    case ITEM: {
      if (isFixedRowsMFitem())
        return false;

      return true;
    }
    case INVALID:
      return false;
    default:
      assert(false);
      return false;
  }
}

void WbTreeItem::del() {
  switch (mType) {
    case NODE:
    case ITEM: {
      WbMultipleValue *mvalue = static_cast<WbMultipleValue *>(mParent->mField->value());
      mvalue->removeItem(row());
      break;
    }
    case FIELD: {
      WbSFNode *sfnode = static_cast<WbSFNode *>(mField->value());
      sfnode->setValue(NULL);
      break;
    }
    case INVALID:
    default:
      assert(false);
  }
}

// invalidate item and sub-items and return the total number of lines (item) to be removed in the Scene Tree
int WbTreeItem::makeInvalid() {
  mType = INVALID;
  mNode = NULL;

  int count = 1;
  foreach (WbTreeItem *c, mChildren)
    count += c->makeInvalid();

  return count;
}

void WbTreeItem::emitChildNeedsDeletion(int row) {
  mChildren.at(row)->makeInvalid();
  emit childrenNeedDeletion(row, 1);
}

void WbTreeItem::emitDeleteAllChildren() {
  for (int i = mChildren.size() - 1; i >= 0; --i)
    mChildren.at(i)->makeInvalid();

  deleteAllChildren();
}

void WbTreeItem::addChild(int row) {
  emit rowsInserted(row, 1);
}

void WbTreeItem::deleteChild(int row) {
  delete mChildren.at(row);
  mChildren.remove(row);
}

void WbTreeItem::deleteAllChildren() {
  qDeleteAll(mChildren);
  mChildren.clear();
}

void WbTreeItem::sfnodeChanged() {
  assert(mType == FIELD);
  WbSFNode *sfnode = static_cast<WbSFNode *>(mField->value());
  WbNode *nodeObject = sfnode->value();

  // delete previous children items
  int count = 0;
  foreach (WbTreeItem *c, mChildren)
    count += c->makeInvalid();
  if (count)
    emit childrenNeedDeletion(0, count);

  if (nodeObject) {
    emit rowsInserted(0, 1);
    connect(sfnode->value(), &WbNode::defUseNameChanged, this, &WbTreeItem::propagateDataChange, Qt::UniqueConnection);
  }
}

bool WbTreeItem::isSFNode() const {
  return mType == FIELD && (dynamic_cast<WbSFNode *>(mField->value()) != NULL);
}

WbNode *WbTreeItem::node() const {
  if (mType == NODE)
    return mNode;

  WbSFNode *sfNode = dynamic_cast<WbSFNode *>(mField->value());
  if (!sfNode)
    return NULL;

  return sfNode->value();
}

int WbTreeItem::itemIndex(const WbTreeItem *item) const {
  return mChildren.indexOf(const_cast<WbTreeItem *>(item));
}

WbTreeItem *WbTreeItem::lastChild() const {
  if (mChildren.isEmpty())
    return NULL;
  return mChildren.last();
}
