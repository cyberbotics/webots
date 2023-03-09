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

#ifndef WB_TREE_VIEW_HPP
#define WB_TREE_VIEW_HPP

//
// Description: A regular QTreeView, just specialized for displaying non-default tree items in a different color
//

#include <QtGui/QKeyEvent>
#include <QtWidgets/QTreeView>

class WbTreeItemDelegate;

// cppcheck-suppress noConstructor
class WbTreeView : public QTreeView {
  Q_OBJECT
  Q_PROPERTY(QColor defaultColor MEMBER mDefaultColor READ defaultColor WRITE setDefaultColor)
  Q_PROPERTY(QColor modifiedColor MEMBER mModifiedColor READ modifiedColor WRITE setModifiedColor)

public:
  explicit WbTreeView(QWidget *parent = NULL);
  virtual ~WbTreeView();

  void scrollToSelection() { scrollToModelIndex(currentIndex()); }
  void scrollToModelIndex(const QModelIndex &index);
  void scrollTo(const QModelIndex &index, QTreeView::ScrollHint hint = QTreeView::EnsureVisible) override;

  const QColor &defaultColor() const { return mDefaultColor; }
  const QColor &modifiedColor() const { return mModifiedColor; }

  void setDefaultColor(const QColor &color) { mDefaultColor = color; }
  void setModifiedColor(const QColor &color) { mModifiedColor = color; }

public slots:
  void itemInserted(const QModelIndex &index);
  void showMenu(const QPoint &position);

signals:
  void doubleClickOrEnterPressed();
  void selectionHasChanged();
  void focusIn();
  void refreshRequested();
  void beforeContextMenuShowed();

protected:
  void currentChanged(const QModelIndex &current, const QModelIndex &previous) override;
  void focusInEvent(QFocusEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;

private:
  QColor mDefaultColor, mModifiedColor;
  WbTreeItemDelegate *mTreeItemDelegate;
  bool mIsScrollActive;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
};

#endif
