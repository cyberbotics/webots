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

#ifndef WB_NODE_EDITOR_HPP
#define WB_NODE_EDITOR_HPP

//
// Description: editor for editing a WbSFNode or a WbMFNode item
//

#include "WbValueEditor.hpp"

class WbFieldLineEdit;
class WbNode;

class QCheckBox;
class QLabel;
class QStackedWidget;
class QPushButton;

class WbNodeEditor : public WbValueEditor {
  Q_OBJECT

public:
  explicit WbNodeEditor(QWidget *parent = NULL);

  void recursiveBlockSignals(bool block) override;

  void edit(bool copyOriginalValue) override;
  void stopEditing() override;
  void resetFocus() override;

  void update();

  QWidget *lastEditorWidget() override { return NULL; }

signals:
  void dictionaryUpdateRequested();

public slots:
  void apply() override;
  void cleanValue() override;

protected:
  enum PaneType { DEF_PANE, EMPTY_PANE };

private:
  WbNode *mNode;
  WbFieldLineEdit *mDefEdit;
  QLabel *mUseCount;
  QPushButton *mPrintUrl;
  QLabel *mNbTriangles;
  QStackedWidget *mStackedWidget;
  bool mMessageBox;

  // actions buttons
  QLabel *mShowResizeHandlesLabel;
  QCheckBox *mShowResizeHandlesCheckBox;

  void setTransformActionVisibile(bool visible);
  void takeKeyboardFocus() override {}
  void printUrl();
};

#endif
