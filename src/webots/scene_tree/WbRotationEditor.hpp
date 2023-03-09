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

#ifndef WB_ROTATION_EDITOR_HPP
#define WB_ROTATION_EDITOR_HPP

//
// Description: editor for editing a WbSFRotation or a WbMFRotation item
//

#include "WbRotation.hpp"
#include "WbValueEditor.hpp"

class WbFieldDoubleSpinBox;
class QLabel;
class QPushButton;

class WbRotationEditor : public WbValueEditor {
  Q_OBJECT

public:
  explicit WbRotationEditor(QWidget *parent = NULL);
  virtual ~WbRotationEditor();

  enum RotationType { AXIS_ANGLE = 0, QUATERNIONS };

  void recursiveBlockSignals(bool block) override;

  QWidget *lastEditorWidget() override;

public slots:
  // cppcheck-suppress virtualCallInConstructor
  void applyIfNeeded() override;

protected:
  void edit(bool copyOriginalValue) override;
  void resetFocus() override;

protected slots:
  void apply() override;
  void normalize();
  void updateRotationType(int index);

private:
  void updateSpinBoxes();
  void takeKeyboardFocus() override;
  WbRotation computeRotation();

  WbRotation mRotation;
  QLabel *mRotationTypeLabel;
  QComboBox *mRotationTypeComboBox;
  int mCurrentRotationType;
  WbFieldDoubleSpinBox *mSpinBoxes[4];
  QLabel *mLabel[4];
  QLabel *mUnitLabel[4];
  QPushButton *mNormalizeButton;

  bool mApplied;
};

#endif
