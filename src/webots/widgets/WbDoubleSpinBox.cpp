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

#include "WbDoubleSpinBox.hpp"

#include "WbClipboard.hpp"

#include <QtGui/QKeyEvent>
#include <QtWidgets/QLineEdit>

WbDoubleSpinBox::WbDoubleSpinBox(QWidget *parent) : QDoubleSpinBox(parent), mClipboard(WbClipboard::instance()) {
  setButtonSymbols(PlusMinus);
  setMinimumHeight(sizeHint().height());
}

WbDoubleSpinBox::~WbDoubleSpinBox() {
}

void WbDoubleSpinBox::keyPressEvent(QKeyEvent *event) {
  if (event->matches(QKeySequence::Cut))
    cut();
  else if (event->matches(QKeySequence::Copy))
    copy();
  else if (event->matches(QKeySequence::Paste))
    paste();
  else
    QDoubleSpinBox::keyPressEvent(event);
};

void WbDoubleSpinBox::cut() {
  if (lineEdit()->hasSelectedText()) {
    copy();
    lineEdit()->backspace();  // remove selected text
  }
}

void WbDoubleSpinBox::copy() const {
  if (lineEdit()->hasSelectedText()) {
    bool ok;
    const double value = lineEdit()->selectedText().toDouble(&ok);
    if (ok)
      mClipboard->setDouble(value);
    else
      mClipboard->setString(lineEdit()->selectedText());
  }
}

void WbDoubleSpinBox::paste() {
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
  newText.toDouble(&ok);

  if (ok)
    le->insert(pastedText);
}
