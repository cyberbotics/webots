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

#include "WbDoubleEditor.hpp"

#include "WbField.hpp"
#include "WbFieldDoubleSpinBox.hpp"
#include "WbMFDouble.hpp"
#include "WbSFDouble.hpp"
#include "WbSimulationState.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

WbDoubleEditor::WbDoubleEditor(QWidget *parent) : WbValueEditor(parent), mSpinBox(new WbFieldDoubleSpinBox(this)) {
  connect(mSpinBox, &WbFieldDoubleSpinBox::valueApplied, this, &WbDoubleEditor::apply);
  connect(mSpinBox, &WbFieldDoubleSpinBox::focusLeft, this, &WbDoubleEditor::applyIfNeeded);
  mLayout->addWidget(mSpinBox, 1, 1);
}

WbDoubleEditor::~WbDoubleEditor() {
}

void WbDoubleEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mSpinBox->blockSignals(block);
}

QWidget *WbDoubleEditor::lastEditorWidget() {
  return mSpinBox;
}

void WbDoubleEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mDouble = static_cast<WbSFDouble *>(singleValue())->value();
    else if (multipleValue())
      mDouble = static_cast<WbMFDouble *>(multipleValue())->item(index());
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  mSpinBox->setVisible(!hasRetrictedValues);

  if (hasRetrictedValues) {
    mLayout->setColumnStretch(0, 0);
    mLayout->setColumnStretch(2, 0);
  } else {
    mLayout->setColumnStretch(0, 1);
    mLayout->setColumnStretch(2, 1);
  }

  if (!hasRetrictedValues)
    mSpinBox->setValueNoSignals(mDouble);
}

void WbDoubleEditor::resetFocus() {
  mSpinBox->clearFocus();
}

void WbDoubleEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mDouble != mComboBox->currentText().toDouble()) ||
                  (!field()->hasRestrictedValues() && mDouble != mSpinBox->value())))
    apply();
}

void WbDoubleEditor::takeKeyboardFocus() {
  mSpinBox->setFocus();
  mSpinBox->selectAll();
}

void WbDoubleEditor::apply() {
  mDouble = field()->hasRestrictedValues() ? mComboBox->currentText().toDouble() :
                                             WbPrecision::roundValue(mSpinBox->value(), WbPrecision::GUI_MEDIUM);
  if (singleValue()) {
    WbSFDouble *const sfDouble = static_cast<WbSFDouble *>(singleValue());
    if (sfDouble->value() == mDouble)
      return;

    mPreviousValue->setDouble(sfDouble->value());

  } else if (multipleValue()) {
    WbMFDouble *const mfDouble = static_cast<WbMFDouble *>(multipleValue());
    if (mfDouble->item(index()) == mDouble)
      return;

    mPreviousValue->setDouble(mfDouble->item(index()));
  }

  mNewValue->setDouble(mDouble);
  WbValueEditor::apply();
}
