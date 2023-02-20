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

#include "WbColorEditor.hpp"

#include "WbField.hpp"
#include "WbFieldDoubleSpinBox.hpp"
#include "WbMFColor.hpp"
#include "WbSFColor.hpp"
#include "WbSimulationState.hpp"

#include <QtWidgets/QColorDialog>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QToolButton>

WbColorEditor::WbColorEditor(QWidget *parent) : WbValueEditor(parent), mColorButton(new QToolButton(this)) {
  static const QStringList LABELS(QStringList() << "red:"
                                                << "green:"
                                                << "blue:");
  QGridLayout *const gridLayout = new QGridLayout();

  for (int i = 0; i < 3; ++i) {
    mLabel[i] = new QLabel(LABELS[i], this);
    gridLayout->addWidget(mLabel[i], i, 0, Qt::AlignRight);
    mSpinBoxes[i] = new WbFieldDoubleSpinBox(this, WbFieldDoubleSpinBox::RGB);
    mSpinBoxes[i]->setMinimumWidth(80);
    connect(mSpinBoxes[i], &WbFieldDoubleSpinBox::valueApplied, this, &WbColorEditor::apply);
    connect(mSpinBoxes[i], &WbFieldDoubleSpinBox::focusLeft, this, &WbColorEditor::applyIfNeeded);
    gridLayout->addWidget(mSpinBoxes[i], i, 1);
  }

  setTabOrder(mSpinBoxes[2], mColorButton);

  mColorButton->setObjectName("colorToolButton");
  mColorButton->setFixedSize(64, 64);
  connect(mColorButton, &QToolButton::clicked, this, &WbColorEditor::openColorChooser);

  mLayout->setColumnStretch(0, 1);
  mLayout->addLayout(gridLayout, 1, 1);
  mLayout->addWidget(mColorButton, 1, 2);
  mLayout->setColumnStretch(3, 1);
}

WbColorEditor::~WbColorEditor() {
}

void WbColorEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mColorButton->blockSignals(block);
  for (int i = 0; i < 3; ++i)
    mSpinBoxes[i]->blockSignals(block);
}

QWidget *WbColorEditor::lastEditorWidget() {
  return mColorButton;
}

void WbColorEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mRgb = static_cast<WbSFColor *>(singleValue())->value();
    else if (multipleValue())
      mRgb = static_cast<WbMFColor *>(multipleValue())->item(index());
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  for (int i = 0; i < 3; ++i) {
    mLabel[i]->setVisible(!hasRetrictedValues);
    mSpinBoxes[i]->setVisible(!hasRetrictedValues);
  }
  mColorButton->setVisible(!hasRetrictedValues);

  if (!hasRetrictedValues) {
    updateSpinBoxes();
    updateButton();
  }
}

void WbColorEditor::updateSpinBoxes() {
  mSpinBoxes[0]->setValueNoSignals(mRgb.red());
  mSpinBoxes[1]->setValueNoSignals(mRgb.green());
  mSpinBoxes[2]->setValueNoSignals(mRgb.blue());
}

void WbColorEditor::resetFocus() {
  mSpinBoxes[0]->clearFocus();
  mSpinBoxes[1]->clearFocus();
  mSpinBoxes[2]->clearFocus();
}

void WbColorEditor::updateButton() {
  const int r = mRgb.redByte();
  const int g = mRgb.greenByte();
  const int b = mRgb.blueByte();

  mColorButton->setStyleSheet(QString("QToolButton {"
                                      "background-color: rgb(%1,%2,%3);"
                                      "}"
                                      "QToolButton:pressed {"
                                      "background-color: rgb(%4,%5,%6);"
                                      "}")
                                .arg(r)
                                .arg(g)
                                .arg(b)
                                .arg(r * 0.8)
                                .arg(g * 0.8)
                                .arg(b * 0.8));
}

void WbColorEditor::takeKeyboardFocus() {
  mSpinBoxes[0]->setFocus();
  mSpinBoxes[0]->selectAll();
}

void WbColorEditor::openColorChooser() {
  WbSimulationState::instance()->pauseSimulation();
  const int r = mRgb.redByte();
  const int g = mRgb.greenByte();
  const int b = mRgb.blueByte();

  QColor color(r, g, b);
  QColorDialog dialog(color, this);
  const int result = dialog.exec();
  if (result == QDialog::Rejected) {
    WbSimulationState::instance()->resumeSimulation();
    return;
  }

  color = dialog.currentColor();
  mRgb.setValue(color.redF(), color.greenF(), color.blueF());

  updateSpinBoxes();
  apply();
  WbSimulationState::instance()->resumeSimulation();
}

void WbColorEditor::applyIfNeeded() {
  if (field() &&
      ((field()->hasRestrictedValues() && mRgb != WbRgb(mComboBox->currentText())) ||
       (!field()->hasRestrictedValues() && (mRgb.red() != mSpinBoxes[0]->value() || mRgb.green() != mSpinBoxes[1]->value() ||
                                            mRgb.blue() != mSpinBoxes[2]->value()))))
    apply();
}

void WbColorEditor::apply() {
  mRgb.setValue(WbPrecision::roundValue(mSpinBoxes[0]->value(), WbPrecision::GUI_MEDIUM),
                WbPrecision::roundValue(mSpinBoxes[1]->value(), WbPrecision::GUI_MEDIUM),
                WbPrecision::roundValue(mSpinBoxes[2]->value(), WbPrecision::GUI_MEDIUM));
  if (field()->hasRestrictedValues())
    mRgb = WbRgb(mComboBox->currentText());
  updateButton();

  if (singleValue()) {
    WbSFColor *const sfColor = static_cast<WbSFColor *>(singleValue());
    if (sfColor->value() == mRgb)
      return;

    mPreviousValue->setColor(sfColor->value());

  } else if (multipleValue()) {
    WbMFColor *const mfColor = static_cast<WbMFColor *>(multipleValue());
    if (mfColor->item(index()) == mRgb)
      return;

    mPreviousValue->setColor(mfColor->item(index()));
  }

  mNewValue->setColor(mRgb);
  WbValueEditor::apply();
}
