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

#include "WbStringEditor.hpp"

#include "WbField.hpp"
#include "WbFieldLineEdit.hpp"
#include "WbMFString.hpp"
#include "WbSFString.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

WbStringEditor::WbStringEditor(QWidget *parent) : WbValueEditor(parent), mLineEdit(new WbFieldLineEdit(this)) {
  connect(mLineEdit, &WbFieldLineEdit::returnPressed, this, &WbStringEditor::apply);
  connect(mLineEdit, &WbFieldLineEdit::focusLeft, this, &WbStringEditor::applyIfNeeded);
  mLayout->addWidget(mLineEdit, 1, 1);
}

WbStringEditor::~WbStringEditor() {
}

void WbStringEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mLineEdit->blockSignals(block);
}

QWidget *WbStringEditor::lastEditorWidget() {
  return mLineEdit;
}

void WbStringEditor::takeKeyboardFocus() {
  mLineEdit->setFocus();
  mLineEdit->selectAll();
}

void WbStringEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mString = static_cast<WbSFString *>(singleValue())->value();
    else if (multipleValue())
      mString = static_cast<WbMFString *>(multipleValue())->item(index());
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  mLineEdit->setVisible(!hasRetrictedValues);

  if (!hasRetrictedValues)
    mLineEdit->setText(mString);
}

void WbStringEditor::resetFocus() {
  mLineEdit->clearFocus();
}

void WbStringEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mString != mComboBox->currentText()) ||
                  (!field()->hasRestrictedValues() && mString != mLineEdit->text())))
    apply();
}

void WbStringEditor::apply() {
  mString = field()->hasRestrictedValues() ? mComboBox->currentText() : mLineEdit->text();

  if (singleValue()) {
    WbSFString *const sfString = static_cast<WbSFString *>(singleValue());
    if (sfString->value() == mString)
      return;

    mPreviousValue->setString(sfString->value());

  } else if (multipleValue()) {
    WbMFString *const mfString = static_cast<WbMFString *>(multipleValue());
    if (mfString->item(index()) == mString)
      return;

    mPreviousValue->setString(mfString->item(index()));
  }

  mNewValue->setString(mString);
  WbValueEditor::apply();
}
