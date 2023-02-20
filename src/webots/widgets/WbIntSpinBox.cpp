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

#include "WbIntSpinBox.hpp"

#include "WbClipboard.hpp"

#include <QtGui/QKeyEvent>
#include <QtWidgets/QLineEdit>

#include <limits>

WbIntSpinBox::WbIntSpinBox(QWidget *parent) : QSpinBox(parent), mClipboard(WbClipboard::instance()) {
  setButtonSymbols(PlusMinus);
  setMinimumHeight(sizeHint().height());
}

WbIntSpinBox::~WbIntSpinBox() {
}

void WbIntSpinBox::keyPressEvent(QKeyEvent *event) {
  if (event->matches(QKeySequence::Cut))
    cut();
  else if (event->matches(QKeySequence::Copy))
    copy();
  else if (event->matches(QKeySequence::Paste))
    paste();
  else
    QSpinBox::keyPressEvent(event);
};

void WbIntSpinBox::cut() {
  if (lineEdit()->hasSelectedText()) {
    copy();
    lineEdit()->backspace();  // remove selected text
  }
}

void WbIntSpinBox::copy() const {
  if (lineEdit()->hasSelectedText()) {
    bool ok;
    const int value = lineEdit()->selectedText().toInt(&ok);
    if (ok)
      mClipboard->setInt(value);
    else
      mClipboard->setString(lineEdit()->selectedText());
  }
}

void WbIntSpinBox::paste() {
  if (mClipboard->isEmpty())
    return;

  QLineEdit *le = lineEdit();

  const QString pastedText = mClipboard->stringValue();
  QString newText = le->text();
  if (le->hasSelectedText())
    // remove selected text
    newText.remove(le->selectionStart(), le->selectedText().length());
  // insert pasted text
  newText.insert(le->cursorPosition(), mClipboard->stringValue());
  // check if valid value
  bool ok;
  // cppcheck-suppress ignoredReturnValue
  newText.toInt(&ok);

  if (ok)
    le->insert(pastedText);
}
