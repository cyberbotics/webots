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

#ifndef WB_VECTOR3_EDITOR_HPP
#define WB_VECTOR3_EDITOR_HPP

//
// Description: editor for editing a WbSFVector3 or a WbMFVector3 item
//

#include "WbValueEditor.hpp"
#include "WbVector3.hpp"

class WbFieldDoubleSpinBox;
class QLabel;

class WbVector3Editor : public WbValueEditor {
  Q_OBJECT

public:
  explicit WbVector3Editor(QWidget *parent = NULL);
  virtual ~WbVector3Editor();

  void recursiveBlockSignals(bool block) override;

  QWidget *lastEditorWidget() override;

protected:
  void edit(bool copyOriginalValue) override;
  void resetFocus() override;

public slots:
  void applyIfNeeded() override;

protected slots:
  void apply() override;

private slots:
  void updateSpinBoxes();

private:
  WbVector3 mVector3;
  WbFieldDoubleSpinBox *mSpinBoxes[3];
  QLabel *mLabel[3];
  QLabel *mUnitLabel[3];
  void takeKeyboardFocus() override;

  bool mApplied;
};

#endif
