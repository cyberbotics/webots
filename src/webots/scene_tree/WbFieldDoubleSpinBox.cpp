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

#include "WbFieldDoubleSpinBox.hpp"

#include "WbUndoStack.hpp"

#include <QtGui/QKeyEvent>
#include <QtWidgets/QLineEdit>

#include <cfloat>
#include <cmath>
#include <limits>

WbFieldDoubleSpinBox::WbFieldDoubleSpinBox(QWidget *parent, int mode) : WbDoubleSpinBox(parent) {
  mDecimals = 0;
  mMode = mode;

  setDecimals(DBL_MAX_10_EXP + DBL_DIG);
  setRange(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

  connect(lineEdit(), &QLineEdit::textEdited, this, &WbFieldDoubleSpinBox::findDecimals);

  if (mMode == RGB)
    setRange(0.0, 1.0);
}

WbFieldDoubleSpinBox::~WbFieldDoubleSpinBox() {
}

void WbFieldDoubleSpinBox::setValueNoSignals(double value) {
  if (value == this->value())
    return;

  blockSignals(true);
  setValue(value);
  findDecimals(text());
  blockSignals(false);
}

void WbFieldDoubleSpinBox::setMode(int mode) {
  mDecimals = 0;
  mMode = mode;
  if (mMode == RGB)
    setRange(0.0, 1.0);
  findDecimals(text());
}

void WbFieldDoubleSpinBox::stepBy(int steps) {
  double value = text().toDouble();

  switch (mMode) {
    case NORMAL:
      // normal double change the last decimal
      value += pow(10, -mDecimals) * steps;
      break;
    case RADIANS: {
      // angle in radians: change by pi/24 and snap to the nearest multiple of PI
      value += M_PI * steps / 24.0;
      double decpart = fmod(fabs(value) / M_PI, 1.0);
      if (decpart < 0.01 || decpart > 0.99)
        value = M_PI * round(value / M_PI);
      break;
    }
    case AXIS:
      // rotation axis: change the last decimal and bound between -1, 0, 1
      value += pow(10, -mDecimals) * steps;
      value = qBound(-1.0, value, 1.0);
      break;
    case RGB:
      // change the last decimal but only between -1.0 and 1.0
      if (mDecimals < 1)
        mDecimals = 1;
      value += pow(10, -mDecimals) * steps;
      value = qBound(0.0, value, 1.0);
      break;
  }

  setValue(value);
  emit valueApplied();
}

void WbFieldDoubleSpinBox::findDecimals(const QString &text) {
  if (mMode == RADIANS)
    return;

  // find current decimal position
  int index = text.indexOf('.');
  if (index == -1)
    mDecimals = 0;
  else
    mDecimals = text.length() - index - 1;
}

QString WbFieldDoubleSpinBox::textFromValue(double value) const {
  return QString::number(value);
}

void WbFieldDoubleSpinBox::keyPressEvent(QKeyEvent *event) {
  if (event->matches(QKeySequence::Undo))
    WbUndoStack::instance()->undo();
  else if (event->matches(QKeySequence::Redo))
    WbUndoStack::instance()->redo();
  else
    WbDoubleSpinBox::keyPressEvent(event);
}

void WbFieldDoubleSpinBox::keyReleaseEvent(QKeyEvent *event) {
  QDoubleSpinBox::keyReleaseEvent(event);

  int key = event->key();
  if (key == Qt::Key_Return || key == Qt::Key_Enter)
    emit valueApplied();
}

void WbFieldDoubleSpinBox::focusOutEvent(QFocusEvent *event) {
  QDoubleSpinBox::focusOutEvent(event);
  emit focusLeft();
}
