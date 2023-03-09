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

#include "WbIntEditor.hpp"

#include "WbField.hpp"
#include "WbFieldIntSpinBox.hpp"
#include "WbMFInt.hpp"
#include "WbSFInt.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

WbIntEditor::WbIntEditor(QWidget *parent) : WbValueEditor(parent), mInt(-1), mSpinBox(new WbFieldIntSpinBox(this)) {
  connect(mSpinBox, &WbFieldIntSpinBox::valueApplied, this, &WbIntEditor::apply);
  connect(mSpinBox, &WbFieldIntSpinBox::focusLeft, this, &WbIntEditor::applyIfNeeded);
  mLayout->addWidget(mSpinBox, 1, 1);
}

WbIntEditor::~WbIntEditor() {
}

void WbIntEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mSpinBox->blockSignals(block);
}

QWidget *WbIntEditor::lastEditorWidget() {
  return mSpinBox;
}

void WbIntEditor::takeKeyboardFocus() {
  mSpinBox->setFocus();
  mSpinBox->selectAll();
}

void WbIntEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mInt = static_cast<WbSFInt *>(singleValue())->value();
    else if (multipleValue())
      mInt = static_cast<WbMFInt *>(multipleValue())->item(index());
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
    mSpinBox->setValueNoSignals(mInt);
}

void WbIntEditor::resetFocus() {
  mSpinBox->clearFocus();
}

void WbIntEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mInt != mComboBox->currentText().toInt()) ||
                  (!field()->hasRestrictedValues() && mInt != mSpinBox->value())))
    apply();
}

void WbIntEditor::apply() {
  mInt = field()->hasRestrictedValues() ? mComboBox->currentText().toInt() : mSpinBox->value();

  if (singleValue()) {
    WbSFInt *const sfInt = static_cast<WbSFInt *>(singleValue());
    if (sfInt->value() == mInt)
      return;

    mPreviousValue->setInt(sfInt->value());

  } else if (multipleValue()) {
    WbMFInt *const mfInt = static_cast<WbMFInt *>(multipleValue());
    if (mfInt->item(index()) == mInt)
      return;

    mPreviousValue->setInt(mfInt->item(index()));
  }

  mNewValue->setInt(mInt);
  WbValueEditor::apply();
}
