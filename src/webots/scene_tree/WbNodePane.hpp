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

#ifndef WB_NODE_PANE_HPP
#define WB_NODE_PANE_HPP

//
// Description: pane containing viewers and editors for a WbSFNode or a WbMFNode item
//

#include "WbValueEditor.hpp"

class WbNodeEditor;
class WbPhysicsViewer;
class WbPositionViewer;
class WbVelocityViewer;

class QHBoxLayout;
class QTabWidget;

class WbNodePane : public WbValueEditor {
  Q_OBJECT

public:
  explicit WbNodePane(QWidget *parent = NULL);
  virtual ~WbNodePane();

  void recursiveBlockSignals(bool block) override;

  void edit(WbNode *node, WbField *field, int index) override;
  void edit(bool copyOriginalValue) override;
  void stopEditing() override;

  const WbNodeEditor *nodeEditor() const { return mNodeEditor; }

  QWidget *lastEditorWidget() override { return NULL; }

public slots:
  void cleanValue() override;

protected:
  void resetFocus() override;

protected slots:
  void apply() override;

private:
  enum TabIndex { NODE_TAB = 0, PHYSICS_TAB, POSITION_TAB, VELOCITY_TAB };

  // Tab widgets
  QTabWidget *mTabs;
  WbNodeEditor *mNodeEditor;
  WbPhysicsViewer *mPhysicsViewer;
  WbPositionViewer *mPositionViewer;
  WbVelocityViewer *mVelocityViewer;
  // save the selected tab name to restore it when a different node is selected
  QString mPreviousTabName;

  void update();
  void enableTab(int index, QWidget *widget, bool enabled);
  void takeKeyboardFocus() override {}

private slots:
  void updateSelectedTab();
};

#endif
