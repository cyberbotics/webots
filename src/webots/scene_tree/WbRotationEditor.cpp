// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbRotationEditor.hpp"

#include "WbField.hpp"
#include "WbFieldDoubleSpinBox.hpp"
#include "WbMFRotation.hpp"
#include "WbSFRotation.hpp"
#include "WbSimulationState.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

WbRotationEditor::WbRotationEditor(QWidget *parent) : WbValueEditor(parent), mApplied(false) {
  static const QStringList LABELS(QStringList() << "x:"
                                                << "y:"
                                                << "z:" << tr("angle:"));
  static const QStringList UNITS(QStringList() << "m"
                                               << "m"
                                               << "m"
                                               << "rad");
  for (int i = 0; i < 4; ++i) {
    mLabel[i] = new QLabel(LABELS[i], this);
    mLayout->addWidget(mLabel[i], i + 1, 0, Qt::AlignRight);
    mSpinBoxes[i] = new WbFieldDoubleSpinBox(this, i == 3 ? WbFieldDoubleSpinBox::RADIANS : WbFieldDoubleSpinBox::AXIS);
    connect(mSpinBoxes[i], &WbFieldDoubleSpinBox::valueApplied, this, &WbRotationEditor::apply);
    connect(mSpinBoxes[i], &WbFieldDoubleSpinBox::focusLeft, this, &WbRotationEditor::applyIfNeeded);
    mLayout->addWidget(mSpinBoxes[i], i + 1, 1);
    mUnitLabel[i] = new QLabel(UNITS[i], this);
    mLayout->addWidget(mUnitLabel[i], i + 1, 2);
  }
}

WbRotationEditor::~WbRotationEditor() {
}

void WbRotationEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  for (int i = 0; i < 4; ++i)
    mSpinBoxes[i]->blockSignals(block);
}

QWidget *WbRotationEditor::lastEditorWidget() {
  return mSpinBoxes[3];
}

void WbRotationEditor::edit(bool copyOriginalValue) {
  // don't show the results if the rotation editor
  // is edited manually (applied) in order to prevent
  // automatic normalization
  if (mApplied)
    return;

  if (copyOriginalValue) {
    if (singleValue())
      mRotation = static_cast<WbSFRotation *>(singleValue())->value();
    else if (multipleValue())
      mRotation = static_cast<WbMFRotation *>(multipleValue())->item(index());
    updateSpinBoxes();
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  for (int i = 0; i < 4; ++i) {
    mLabel[i]->setVisible(!hasRetrictedValues);
    mSpinBoxes[i]->setVisible(!hasRetrictedValues);
    mUnitLabel[i]->setVisible(!hasRetrictedValues);
  }
}

void WbRotationEditor::updateSpinBoxes() {
  for (int i = 0; i < 4; ++i)
    if (WbSimulationState::instance()->isPaused() || !mSpinBoxes[i]->hasFocus())
      mSpinBoxes[i]->setValueNoSignals(mRotation[i]);
}

void WbRotationEditor::resetFocus() {
  for (int i = 0; i < 4; ++i)
    mSpinBoxes[i]->clearFocus();
}

void WbRotationEditor::takeKeyboardFocus() {
  mSpinBoxes[0]->setFocus();
  mSpinBoxes[0]->selectAll();
}

void WbRotationEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mRotation != WbRotation(mComboBox->currentText())) ||
                  (!field()->hasRestrictedValues() &&
                   (mRotation.x() != mSpinBoxes[0]->value() || mRotation.y() != mSpinBoxes[1]->value() ||
                    mRotation.z() != mSpinBoxes[2]->value() || mRotation.angle() != mSpinBoxes[3]->value()))))
    apply();
}

void WbRotationEditor::apply() {
  mRotation.setAxisAngle(mSpinBoxes[0]->value(), mSpinBoxes[1]->value(), mSpinBoxes[2]->value(), mSpinBoxes[3]->value());

  if (field()->hasRestrictedValues())
    mRotation = WbRotation(mComboBox->currentText());

  if (singleValue()) {
    WbSFRotation *const sfRotation = static_cast<WbSFRotation *>(singleValue());
    if (sfRotation->value() == mRotation)
      return;

    mPreviousValue->setRotation(sfRotation->value());

  } else if (multipleValue()) {
    WbMFRotation *const mfRotation = static_cast<WbMFRotation *>(multipleValue());
    if (mfRotation->item(index()) == mRotation)
      return;

    mPreviousValue->setRotation(mfRotation->item(index()));
  }

  mNewValue->setRotation(mRotation);
  mApplied = true;
  WbValueEditor::apply();
  mApplied = false;
}
