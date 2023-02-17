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

#include "WbVector3Editor.hpp"

#include "WbField.hpp"
#include "WbFieldDoubleSpinBox.hpp"
#include "WbMFVector3.hpp"
#include "WbSFVector3.hpp"
#include "WbSimulationState.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

WbVector3Editor::WbVector3Editor(QWidget *parent) : WbValueEditor(parent), mApplied(false) {
  static const QStringList LABELS(QStringList() << "x:"
                                                << "y:"
                                                << "z:");
  for (int i = 0; i < 3; ++i) {
    mLabel[i] = new QLabel(LABELS[i], this);
    mLayout->addWidget(mLabel[i], i + 1, 0, Qt::AlignRight);
    mSpinBoxes[i] = new WbFieldDoubleSpinBox(this);
    connect(mSpinBoxes[i], &WbFieldDoubleSpinBox::valueApplied, this, &WbVector3Editor::apply);
    connect(mSpinBoxes[i], &WbFieldDoubleSpinBox::focusLeft, this, &WbVector3Editor::applyIfNeeded);
    mLayout->addWidget(mSpinBoxes[i], i + 1, 1);
    mUnitLabel[i] = new QLabel("m", this);
    mLayout->addWidget(mUnitLabel[i], i + 1, 2);
  }
}

WbVector3Editor::~WbVector3Editor() {
}

void WbVector3Editor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  for (int i = 0; i < 3; ++i)
    mSpinBoxes[i]->blockSignals(block);
}

QWidget *WbVector3Editor::lastEditorWidget() {
  return mSpinBoxes[2];
}

void WbVector3Editor::edit(bool copyOriginalValue) {
  // don't show the results if the Vector3 editor
  // is edited manually (applied) in order to prevent
  // automatic normalization or default value restoration
  if (mApplied)
    return;

  if (copyOriginalValue) {
    if (singleValue())
      mVector3 = static_cast<WbSFVector3 *>(singleValue())->value();
    else if (multipleValue())
      mVector3 = static_cast<WbMFVector3 *>(multipleValue())->item(index());
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  for (int i = 0; i < 3; ++i) {
    mLabel[i]->setVisible(!hasRetrictedValues);
    mSpinBoxes[i]->setVisible(!hasRetrictedValues);
    mUnitLabel[i]->setVisible(!hasRetrictedValues);
  }

  if (!hasRetrictedValues)
    updateSpinBoxes();
}

void WbVector3Editor::takeKeyboardFocus() {
  mSpinBoxes[0]->setFocus();
  mSpinBoxes[0]->selectAll();
}

void WbVector3Editor::updateSpinBoxes() {
  for (int i = 0; i < 3; ++i)
    if (WbSimulationState::instance()->isPaused() || !mSpinBoxes[i]->hasFocus())  // in order to prevent updating while editing
      mSpinBoxes[i]->setValueNoSignals(mVector3[i]);
}

void WbVector3Editor::resetFocus() {
  for (int i = 0; i < 3; ++i)
    mSpinBoxes[i]->clearFocus();
}

void WbVector3Editor::applyIfNeeded() {
  if (field() &&
      ((field()->hasRestrictedValues() && mVector3 != WbVector3(mComboBox->currentText())) ||
       (!field()->hasRestrictedValues() && (mVector3.x() != mSpinBoxes[0]->value() || mVector3.y() != mSpinBoxes[1]->value() ||
                                            mVector3.z() != mSpinBoxes[2]->value()))))
    apply();
}

void WbVector3Editor::apply() {
  if (field()->hasRestrictedValues())
    mVector3 = WbVector3(mComboBox->currentText());
  else
    mVector3.setXyz(WbPrecision::roundValue(mSpinBoxes[0]->value(), WbPrecision::GUI_MEDIUM),
                    WbPrecision::roundValue(mSpinBoxes[1]->value(), WbPrecision::GUI_MEDIUM),
                    WbPrecision::roundValue(mSpinBoxes[2]->value(), WbPrecision::GUI_MEDIUM));
  mVector3.clamp();
  if (singleValue()) {
    WbSFVector3 *const sfVector3 = static_cast<WbSFVector3 *>(singleValue());
    if (sfVector3->value() == mVector3)
      return;

    mPreviousValue->setVector3(sfVector3->value());

  } else if (multipleValue()) {
    WbMFVector3 *const mfVector3 = static_cast<WbMFVector3 *>(multipleValue());
    if (mfVector3->item(index()) == mVector3)
      return;

    mPreviousValue->setVector3(mfVector3->item(index()));
  }

  mNewValue->setVector3(mVector3);
  mApplied = true;
  WbValueEditor::apply();
  mApplied = false;
}
