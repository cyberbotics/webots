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

#include "WbFieldIntSpinBox.hpp"

#include "WbUndoStack.hpp"

#include <QtGui/QKeyEvent>

#include <limits>

WbFieldIntSpinBox::WbFieldIntSpinBox(QWidget *parent) : WbIntSpinBox(parent) {
  setRange(std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
}

WbFieldIntSpinBox::~WbFieldIntSpinBox() {
}

void WbFieldIntSpinBox::stepBy(int steps) {
  QSpinBox::stepBy(steps);
  emit valueApplied();
}

void WbFieldIntSpinBox::setValueNoSignals(int value) {
  if (value == this->value())
    return;

  blockSignals(true);
  setValue(value);
  blockSignals(false);
}

void WbFieldIntSpinBox::keyPressEvent(QKeyEvent *event) {
  if (event->matches(QKeySequence::Undo))
    WbUndoStack::instance()->undo();
  else if (event->matches(QKeySequence::Redo))
    WbUndoStack::instance()->redo();
  else
    WbIntSpinBox::keyPressEvent(event);
};

void WbFieldIntSpinBox::keyReleaseEvent(QKeyEvent *event) {
  QSpinBox::keyReleaseEvent(event);

  int key = event->key();
  if (key == Qt::Key_Return || key == Qt::Key_Enter)
    emit valueApplied();
}

void WbFieldIntSpinBox::focusOutEvent(QFocusEvent *event) {
  QSpinBox::focusOutEvent(event);
  emit focusLeft();
}
