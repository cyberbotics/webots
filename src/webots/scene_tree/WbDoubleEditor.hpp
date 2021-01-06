// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_DOUBLE_EDITOR_HPP
#define WB_DOUBLE_EDITOR_HPP

//
// Description: editor for editing a WbSFDouble or a WbMFDouble item
//

#include "WbValueEditor.hpp"

class WbFieldDoubleSpinBox;

class WbDoubleEditor : public WbValueEditor {
  Q_OBJECT

public:
  explicit WbDoubleEditor(QWidget *parent = NULL);
  virtual ~WbDoubleEditor();

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
  double mDouble;
  WbFieldDoubleSpinBox *mSpinBox;

  void takeKeyboardFocus() override;
};

#endif
