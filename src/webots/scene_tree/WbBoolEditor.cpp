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

#include "WbBoolEditor.hpp"

#include "WbField.hpp"
#include "WbMFBool.hpp"
#include "WbSFBool.hpp"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

WbBoolEditor::WbBoolEditor(QWidget *parent) : WbValueEditor(parent), mCheckBox(new QCheckBox(this)) {
  mCheckBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  connect(mCheckBox, &QCheckBox::stateChanged, this, &WbBoolEditor::apply);
  mLayout->addWidget(mCheckBox, 1, 1);
}

WbBoolEditor::~WbBoolEditor() {
}

void WbBoolEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mCheckBox->blockSignals(block);
}

QWidget *WbBoolEditor::lastEditorWidget() {
  return mCheckBox;
}

void WbBoolEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mBool = static_cast<WbSFBool *>(singleValue())->value();
    else if (multipleValue())
      mBool = static_cast<WbMFBool *>(multipleValue())->item(index());
  }

  mCheckBox->setVisible(!field()->hasRestrictedValues());

  if (mBool)
    mCheckBox->setCheckState(Qt::Checked);
  else
    mCheckBox->setCheckState(Qt::Unchecked);

  updateText();
}

void WbBoolEditor::takeKeyboardFocus() {
  mCheckBox->setFocus();
}

void WbBoolEditor::updateText() {
  switch (mCheckBox->checkState()) {
    case Qt::Checked:
      mCheckBox->setText(tr("TRUE"));
      break;

    case Qt::Unchecked:
      mCheckBox->setText(tr("FALSE"));
      break;

    default:
      Q_ASSERT(0);
  }
}

void WbBoolEditor::resetFocus() {
  mCheckBox->clearFocus();
}

void WbBoolEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mBool != (mComboBox->currentText() == "TRUE")) ||
                  (!field()->hasRestrictedValues() && mBool != mCheckBox->checkState())))
    apply();
}

void WbBoolEditor::apply() {
  mBool = field()->hasRestrictedValues() ? mComboBox->currentText() == "TRUE" : mCheckBox->checkState();

  if (singleValue()) {
    WbSFBool *const sfBool = static_cast<WbSFBool *>(singleValue());
    if (sfBool->value() == mBool)
      return;
    mPreviousValue->setBool(sfBool->value());
  } else if (multipleValue()) {
    WbMFBool *const mfBool = static_cast<WbMFBool *>(multipleValue());
    if (mfBool->item(index()) == mBool)
      return;
    mPreviousValue->setBool(mfBool->item(index()));
  }

  mNewValue->setBool(mBool);
  WbValueEditor::apply();

  updateText();
}
