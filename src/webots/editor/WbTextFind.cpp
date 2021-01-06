// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbTextFind.hpp"

#include <QtCore/QRegularExpression>
#include <QtGui/QTextCursor>
#include <QtGui/QTextDocument>
#include <QtWidgets/QPlainTextEdit>

QStringList WbTextFind::cFindStringList;
QStringList WbTextFind::cReplaceStringList;
WbTextFind::FindFlags WbTextFind::cLastFindFlags;
bool WbTextFind::cIsLastStringEmpty = true;

WbTextFind::WbTextFind(QPlainTextEdit *editor) : mEditor(editor) {
}

void WbTextFind::addStringToList(const QString &s, QStringList &list) {
  if (s.isEmpty())
    return;

  // prepend avoiding duplicates
  int index = list.indexOf(s);
  if (index != -1)
    list.removeAt(index);
  list.prepend(s);

  if (list.size() > 10)
    list.removeLast();
}

bool WbTextFind::find(const QString &text, FindFlags flags, bool backwards) {
  return findText(text, mEditor->textCursor().position(), flags, backwards);
}

bool WbTextFind::findFromBegin(const QString &text, FindFlags flags, bool backwards) {
  return findText(text, 0, flags, backwards);
}

bool WbTextFind::findFromEnd(const QString &text, FindFlags flags, bool backwards) {
  return findText(text, mEditor->document()->characterCount(), flags, backwards);
}

bool WbTextFind::findText(const QString &text, int position, FindFlags flags, bool backwards) {
  if (mEditor == NULL)
    return true;
  if (text.isEmpty()) {
    emit findStringChanged(QRegExp());
    cIsLastStringEmpty = true;
    return true;
  }

  QTextDocument *document = mEditor->document();
  QTextDocument::FindFlags findFlags;
  if (backwards)
    findFlags |= QTextDocument::FindBackward;
  if (flags & FIND_CASE_SENSITIVE)
    findFlags |= QTextDocument::FindCaseSensitively;
  if (flags & FIND_WHOLE_WORDS)
    findFlags |= QTextDocument::FindWholeWords;

  QRegExp regExp = computeRegExp(text, flags);
  emit findStringChanged(regExp);
  if (cIsLastStringEmpty || text != cFindStringList.first()) {
    addStringToList(text, cFindStringList);
    cIsLastStringEmpty = false;
  }
  cLastFindFlags = flags;

  if (backwards)
    position -= text.length();

  QTextCursor result = document->find(regExp, position, findFlags);
  if (result.isNull())
    return false;

  mEditor->blockSignals(true);
  mEditor->setTextCursor(result);
  mEditor->ensureCursorVisible();
  mEditor->blockSignals(false);
  return true;
}

void WbTextFind::replace(const QString &before, const QString &after, FindFlags findFlags) {
  if (mEditor == NULL)
    return;
  if (before.isEmpty()) {
    cIsLastStringEmpty = true;
    emit findStringChanged(QRegExp());
    return;
  }

  addStringToList(before, cFindStringList);
  addStringToList(after, cReplaceStringList);

  QTextCursor cursor = mEditor->textCursor();
  QRegExp beforeRegExp = computeRegExp(before, findFlags);
  if (beforeRegExp.exactMatch(cursor.selectedText())) {
    QString realAfter;
    if (findFlags & FIND_REGULAR_EXPRESSION)
      realAfter = expandRegExpReplacement(after, beforeRegExp.capturedTexts());
    else
      realAfter = after;
    cursor.insertText(realAfter);
    mEditor->setTextCursor(cursor);
  }
}

void WbTextFind::replaceAll(const QString &before, const QString &after, FindFlags findFlags) {
  if (mEditor == NULL)
    return;
  if (before.isEmpty()) {
    emit findStringChanged(QRegExp());
    return;
  }

  addStringToList(before, cFindStringList);
  addStringToList(after, cReplaceStringList);

  QTextCursor cursor = mEditor->textCursor();
  cursor.movePosition(QTextCursor::Start);
  cursor.beginEditBlock();

  QTextDocument::FindFlags docFindFlags;
  if (findFlags & FIND_CASE_SENSITIVE)
    docFindFlags |= QTextDocument::FindCaseSensitively;
  if (findFlags & FIND_WHOLE_WORDS)
    docFindFlags |= QTextDocument::FindWholeWords;
  QRegExp beforeRegExp = computeRegExp(before, findFlags);

  QTextDocument *doc = mEditor->document();
  QTextCursor found = doc->find(beforeRegExp, cursor, docFindFlags);
  bool first = true;
  while (!found.isNull()) {
    if (found == cursor && !first) {
      if (cursor.atEnd())
        break;

      // avoid endless loop of regular expressions
      QTextCursor newPosCursor = cursor;
      newPosCursor.movePosition(QTextCursor::NextCharacter);
      found = doc->find(beforeRegExp, newPosCursor, docFindFlags);
    }
    if (first)
      first = false;

    cursor.setPosition(found.selectionStart());
    cursor.setPosition(found.selectionEnd(), QTextCursor::KeepAnchor);
    beforeRegExp.exactMatch(found.selectedText());

    QString realAfter;
    if (findFlags & FIND_REGULAR_EXPRESSION)
      realAfter = expandRegExpReplacement(after, beforeRegExp.capturedTexts());
    else
      realAfter = after;
    cursor.insertText(realAfter);
    found = doc->find(beforeRegExp, cursor, docFindFlags);
  }

  cursor.endEditBlock();
}

QString WbTextFind::expandRegExpReplacement(const QString &replaceText, const QStringList &capturedTexts) {
  // handles \1 \\ \& & \t \n
  QString result;
  const int numCaptures = capturedTexts.size() - 1;
  for (int i = 0; i < replaceText.length(); ++i) {
    QChar c = replaceText.at(i);
    if (c == QLatin1Char('\\') && i < replaceText.length() - 1) {
      c = replaceText.at(++i);
      if (c == QLatin1Char('\\'))
        result += QLatin1Char('\\');
      else if (c == QLatin1Char('&'))
        result += QLatin1Char('&');
      else if (c == QLatin1Char('t'))
        result += QLatin1Char('\t');
      else if (c == QLatin1Char('n'))
        result += QLatin1Char('\n');
      else if (c.isDigit()) {
        int index = c.unicode() - '1';
        if (index < numCaptures)
          result += capturedTexts.at(index + 1);
        else {
          result += QLatin1Char('\\');
          result += c;
        }
      } else {
        result += QLatin1Char('\\');
        result += c;
      }
    } else if (c == QLatin1Char('&'))
      result += capturedTexts.at(0);
    else
      result += c;
  }
  return result;
}

QRegExp WbTextFind::computeRegExp(const QString &pattern, FindFlags flags) {
  QString expPattern = pattern;
  if ((flags & FIND_REGULAR_EXPRESSION) == 0)
    expPattern = QRegularExpression::escape(pattern);
  if (flags & FIND_WHOLE_WORDS)
    expPattern = "\\b" + expPattern + "\\b";
  QRegExp regExp(expPattern);
  regExp.setCaseSensitivity((flags & FIND_CASE_SENSITIVE) ? Qt::CaseSensitive : Qt::CaseInsensitive);
  return regExp;
}
