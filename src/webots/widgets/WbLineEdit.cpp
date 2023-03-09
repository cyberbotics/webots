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

#include "WbLineEdit.hpp"

#include "WbClipboard.hpp"

#include <QtGui/QKeyEvent>

WbLineEdit::WbLineEdit(QWidget *parent) : QLineEdit(parent), mClipboard(WbClipboard::instance()) {
  setMinimumHeight(sizeHint().height());
}

WbLineEdit::WbLineEdit(const QString &contents, QWidget *parent) :
  QLineEdit(contents, parent),
  mClipboard(WbClipboard::instance()) {
  setMinimumHeight(sizeHint().height());
}

WbLineEdit::~WbLineEdit() {
}

void WbLineEdit::keyPressEvent(QKeyEvent *event) {
  if (event->matches(QKeySequence::Cut))
    cut();
  else if (event->matches(QKeySequence::Copy))
    copy();
  else if (event->matches(QKeySequence::Paste))
    paste();
  else
    QLineEdit::keyPressEvent(event);
};

void WbLineEdit::cut() {
  if (hasSelectedText()) {
    mClipboard->setString(selectedText());
    backspace();  // remove selected text
  }
}

void WbLineEdit::copy() const {
  if (hasSelectedText())
    mClipboard->setString(selectedText());
}

void WbLineEdit::paste() {
  if (mClipboard->isEmpty())
    return;

  const QString text = mClipboard->stringValue();
  if (!text.isEmpty())
    insert(text);
}
