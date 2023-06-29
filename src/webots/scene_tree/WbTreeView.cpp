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

#include "WbTreeView.hpp"

#include "WbActionManager.hpp"
#include "WbContextMenuGenerator.hpp"
#include "WbNodeOperations.hpp"
#include "WbTreeItem.hpp"

#include <QtWidgets/QHeaderView>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QStyle>
#include <QtWidgets/QStyledItemDelegate>

class WbTreeItemDelegate : public QStyledItemDelegate {
public:
  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override {
    QStyleOptionViewItem itemOption(option);

    WbTreeItem *item = static_cast<WbTreeItem *>(index.internalPointer());
    if (item->isDefault())
      // paint unmodified tree items in black
      itemOption.palette.setColor(QPalette::Text, mDefaultColor);
    else
      // paint modified tree items in dark cyan
      itemOption.palette.setColor(QPalette::Text, mModifiedColor);

    // call the base class method for drawing the normal
    // parts of the item of the QTreeView.
    QStyledItemDelegate::paint(painter, itemOption, index);
  }

  void setDefaultColor(const QColor &color) { mDefaultColor = color; }

  void setModifiedColor(const QColor &color) { mModifiedColor = color; }

private:
  QColor mDefaultColor, mModifiedColor;
};

WbTreeView::WbTreeView(QWidget *parent) : QTreeView(parent) {
  setObjectName("TreeView");

  mIsScrollActive = false;
  mTreeItemDelegate = new WbTreeItemDelegate();
  style()->polish(this);
  mTreeItemDelegate->setDefaultColor(defaultColor());
  mTreeItemDelegate->setModifiedColor(modifiedColor());
  setItemDelegate(mTreeItemDelegate);

  // display an horizontal scroll bar rather than
  // cutting strings with '...' characters
  header()->setSectionResizeMode(QHeaderView::ResizeToContents);
  header()->setStretchLastSection(false);
  header()->setDefaultSectionSize(10000);

  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, &QTreeView::customContextMenuRequested, this, &WbTreeView::showMenu);
}

WbTreeView::~WbTreeView() {
  delete mTreeItemDelegate;
}

void WbTreeView::focusInEvent(QFocusEvent *event) {
  QTreeView::focusInEvent(event);
  WbActionManager::instance()->enableTextEditActions(false, true);
  WbActionManager::instance()->setFocusObject(this);

  // when this widget gets keyboard focus the higlighted color of the current
  // item does not change unless we force Qt to redraw the tree view, so do
  // this by refreshing the item model
  emit refreshRequested();

  emit focusIn();
}

void WbTreeView::focusOutEvent(QFocusEvent *event) {
  if (WbActionManager::instance()->focusObject() == this)
    WbActionManager::instance()->setFocusObject(NULL);
  emit refreshRequested();
}

void WbTreeView::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Left) {
    if (currentIndex().parent() != rootIndex() && !isExpanded(currentIndex()))
      setCurrentIndex(currentIndex().parent());
    else if (isExpanded(currentIndex()))
      collapse(currentIndex());
  } else if (event->key() == Qt::Key_Right && isExpanded(currentIndex()))
    setCurrentIndex(currentIndex().model()->index(0, 0, currentIndex()));
  else if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)
    emit doubleClickOrEnterPressed();
  else
    QTreeView::keyPressEvent(event);

  scrollToSelection();
}

void WbTreeView::currentChanged(const QModelIndex &current, const QModelIndex &previous) {
  emit selectionHasChanged();
}

void WbTreeView::itemInserted(const QModelIndex &index) {
  if (!WbNodeOperations::instance()->isFromSupervisor())
    // select new tree item
    setCurrentIndex(index);
}

void WbTreeView::showMenu(const QPoint &position) {
  emit beforeContextMenuShowed();
  const QModelIndexList indexes = selectionModel()->selectedIndexes();
  if (indexes.isEmpty())
    return;
  const WbTreeItem *item = static_cast<WbTreeItem *>(indexes.at(0).internalPointer());
  assert(item);
  WbContextMenuGenerator::generateContextMenu(mapToGlobal(position), item->node(), NULL);
}

void WbTreeView::scrollToModelIndex(const QModelIndex &index) {
  mIsScrollActive = true;
  scrollTo(index);
  mIsScrollActive = false;
}

void WbTreeView::scrollTo(const QModelIndex &index, QTreeView::ScrollHint hint) {
  if (!mIsScrollActive)
    return;

  QTreeView::scrollTo(index, hint);

  if (!index.isValid())
    return;

  // compute improved horizontal scroll position
  int level = -1;
  QModelIndex parentIndex = index.parent();
  while (parentIndex.isValid()) {
    ++level;
    parentIndex = parentIndex.parent();
  }
  horizontalScrollBar()->setValue(indentation() * level);
}

void WbTreeView::mouseDoubleClickEvent(QMouseEvent *event) {
  emit doubleClickOrEnterPressed();
}
