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

#ifndef WB_TEXT_FIND_HPP
#define WB_TEXT_FIND_HPP

//
// Description: a class for managing find and replace actions in a text document
//

#include <QtCore/QRegularExpression>
#include <QtGui/QTextCursor>
#include <QtGui/QTextDocument>

class QPlainTextEdit;

class WbTextFind : public QObject {
  Q_OBJECT
public:
  // options
  enum FindFlags { FIND_NONE = 0x0, FIND_CASE_SENSITIVE = 0x1, FIND_WHOLE_WORDS = 0x2, FIND_REGULAR_EXPRESSION = 0x4 };

  explicit WbTextFind(QPlainTextEdit *editor);
  void setEditor(QPlainTextEdit *editor) { mEditor = editor; }

  bool find(const QString &text, FindFlags flags, bool backwards);
  bool findFromBegin(const QString &text, FindFlags flags, bool backwards);
  bool findFromEnd(const QString &text, FindFlags flags, bool backwards);

  void replace(const QString &before, const QString &after, FindFlags findFlags);
  void replaceAll(const QString &before, const QString &after, FindFlags findFlags);

  static QStringList findStringList() { return cFindStringList; }
  static QStringList replaceStringList() { return cReplaceStringList; }
  static FindFlags lastFindFlags() { return cLastFindFlags; }

signals:
  void findStringChanged(QRegularExpression regularExpression);

private:
  QPlainTextEdit *mEditor;
  static QStringList cFindStringList;
  static QStringList cReplaceStringList;
  static FindFlags cLastFindFlags;
  static bool cIsLastStringEmpty;

  void addStringToList(const QString &s, QStringList &list);

  bool findText(const QString &text, int position, FindFlags flags, bool backwards);
  QString expandRegularExpressionReplacement(const QString &replaceText, const QStringList &capturedTexts);
  QTextCursor find(QTextDocument *document, const QRegularExpression &expr, int from, QTextDocument::FindFlags options) const;
  static QRegularExpression computeRegularExpression(const QString &pattern, FindFlags flags);
};

#endif
