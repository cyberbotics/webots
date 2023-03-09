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

#ifndef WB_VECTOR2_EDITOR_HPP
#define WB_VECTOR2_EDITOR_HPP

//
// Description: editor for editing a WbSFVector2 or a WbMFVector2 item
//

#include "WbValueEditor.hpp"
#include "WbVector2.hpp"

class WbFieldDoubleSpinBox;
class QLabel;

class WbVector2Editor : public WbValueEditor {
  Q_OBJECT

public:
  explicit WbVector2Editor(QWidget *parent = NULL);
  virtual ~WbVector2Editor();

  void recursiveBlockSignals(bool block) override;

  QWidget *lastEditorWidget() override;

public slots:
  void applyIfNeeded() override;

protected:
  void edit(bool copyOriginalValue) override;
  void resetFocus() override;

protected slots:
  void apply() override;

private:
  WbVector2 mVector2;
  WbFieldDoubleSpinBox *mSpinBoxes[2];
  QLabel *mLabel[2];
  QLabel *mUnitLabel[2];
  void takeKeyboardFocus() override;

  void updateSpinBoxes();
};

#endif
