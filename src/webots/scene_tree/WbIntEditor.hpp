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

#ifndef WB_INT_EDITOR_HPP
#define WB_INT_EDITOR_HPP

//
// Description: editor for editing a WbSFInt or a WbMFInt item
//

#include "WbValueEditor.hpp"

class WbFieldIntSpinBox;

class WbIntEditor : public WbValueEditor {
  Q_OBJECT

public:
  explicit WbIntEditor(QWidget *parent = NULL);
  virtual ~WbIntEditor();

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
  int mInt;
  WbFieldIntSpinBox *mSpinBox;
  void takeKeyboardFocus() override;
};

#endif
