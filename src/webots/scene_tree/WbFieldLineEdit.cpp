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

#include "WbFieldLineEdit.hpp"

#include "WbUndoStack.hpp"

#include <QtGui/QKeyEvent>

WbFieldLineEdit::WbFieldLineEdit(QWidget *parent) : WbLineEdit(parent) {
}

WbFieldLineEdit::~WbFieldLineEdit() {
}

void WbFieldLineEdit::keyPressEvent(QKeyEvent *event) {
  if (event->matches(QKeySequence::Undo))
    WbUndoStack::instance()->undo();
  else if (event->matches(QKeySequence::Redo))
    WbUndoStack::instance()->redo();
  else
    WbLineEdit::keyPressEvent(event);
};

void WbFieldLineEdit::focusOutEvent(QFocusEvent *event) {
  QLineEdit::focusOutEvent(event);
  emit focusLeft();
}
