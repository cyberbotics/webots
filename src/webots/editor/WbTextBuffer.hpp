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

#ifndef WB_TEXT_BUFFER_HPP
#define WB_TEXT_BUFFER_HPP

//
// Description: a text buffer used in a tab of the WbTextEditor
//

#include <QtCore/QDir>
#include <QtWidgets/QPlainTextEdit>

class WbClipboard;

class QCompleter;
class QFileSystemWatcher;
class QRegularExpression;
class QResizeEvent;

class WbSyntaxHighlighter;
class LineNumberArea;
class WbLanguage;

// cppcheck-suppress noConstructor
class WbTextBuffer : public QPlainTextEdit {
  Q_OBJECT
  Q_PROPERTY(QColor apiColor MEMBER mApiColor READ apiColor WRITE setApiColor)
  Q_PROPERTY(QColor commentColor MEMBER mCommentColor READ commentColor WRITE setCommentColor)
  Q_PROPERTY(QColor keywordColor MEMBER mKeywordColor READ keywordColor WRITE setKeywordColor)
  Q_PROPERTY(QColor numberColor MEMBER mNumberColor READ numberColor WRITE setNumberColor)
  Q_PROPERTY(QColor preprocessorColor MEMBER mPreprocessorColor READ preprocessorColor WRITE setPreprocessorColor)
  Q_PROPERTY(QColor quotationColor MEMBER mQuotationColor READ quotationColor WRITE setQuotationColor)

  Q_PROPERTY(
    QColor gutterForegroundColor MEMBER mGutterForegroundColor READ gutterForegroundColor WRITE setGutterForegroundColor)
  Q_PROPERTY(
    QColor gutterBackgroundColor MEMBER mGutterBackgroundColor READ gutterBackgroundColor WRITE setGutterBackgroundColor)

public:
  explicit WbTextBuffer(QWidget *parent = NULL);
  virtual ~WbTextBuffer();

  // absolute name of the file in that buffer
  const QString &fileName() const { return mFileName; }
  void setFileName(const QString &fileName);
  bool isUnnamed() const;

  // short file name to display as tab text, e.g. "my_controller.c"
  const QString &shortName() const { return mShortName; }

  // absolute file path with name and suffix
  QString path() const;

  // absolute file dir
  QDir fileDir() const;

  // language used by this buffer
  WbLanguage *language() const { return mLanguage; }

  // load a file in this buffer
  // if title is not specified it will be computed from the file path
  bool load(const QString &fn, const QString &title = QString());

  // revert to file on disk
  bool revert(bool askUser);
  void ignoreFileChangedEvent();

  // save
  bool save();
  bool saveAs(const QString &newName);

  // cut, copy, paste using Webots clipboard
  void cut();
  void copy() const;
  void paste();

  // buffer was edited
  bool isModified() const;

  // is text selected in the buffer
  bool hasSelection() const;
  bool hasSingleBlockSelection() const;

  // replace a pattern for Find/Replace dialog
  void goToLine();

  // mark error line (or word if column is psecified)
  void markError(int line, int column = -1);
  void unmarkError();

  // toggle line comment
  void toggleLineComment();

  // indentation
  enum IndentMode { INCREASE, DECREASE };
  void indent(IndentMode mode);

  // to be used from class LineNumberArea
  void lineNumberAreaPaintEvent(QPaintEvent *event);
  int lineNumberAreaWidth();

  void updateSearchTextHighlighting(QRegularExpression regularExpression);
  const QColor &apiColor() const { return mApiColor; }
  const QColor &commentColor() const { return mCommentColor; }
  const QColor &keywordColor() const { return mKeywordColor; }
  const QColor &numberColor() const { return mNumberColor; }
  const QColor &preprocessorColor() const { return mPreprocessorColor; }
  const QColor &quotationColor() const { return mQuotationColor; }
  void setApiColor(const QColor &color) { mApiColor = color; }
  void setCommentColor(const QColor &color) { mCommentColor = color; }
  void setKeywordColor(const QColor &color) { mKeywordColor = color; }
  void setNumberColor(const QColor &color) { mNumberColor = color; }
  void setPreprocessorColor(const QColor &color) { mPreprocessorColor = color; }
  void setQuotationColor(const QColor &color) { mQuotationColor = color; }

  const QColor &gutterForegroundColor() { return mGutterForegroundColor; }
  const QColor &gutterBackgroundColor() { return mGutterBackgroundColor; }
  void setGutterForegroundColor(const QColor &color) { mGutterForegroundColor = color; }
  void setGutterBackgroundColor(const QColor &color) { mGutterBackgroundColor = color; }

signals:
  void fileNameChanged();
  void focusIn();
  void showRequested();

protected:
  void resizeEvent(QResizeEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void focusInEvent(QFocusEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;

private slots:
  void updateLineNumberAreaWidth(int newBlockCount);
  void updateLineNumberArea(const QRect &, int);
  void insertCompletion(const QString &completion);
  void matchParentheses();
  void updateFont();
  void askForReverting();
  void resetSearchTextHighlighting();

private:
  QColor mApiColor, mCommentColor, mKeywordColor, mNumberColor, mPreprocessorColor, mQuotationColor;  // Q_PROPERTY
  QColor mGutterBackgroundColor, mGutterForegroundColor;
  QString mFileName;   // e.g. /home/my_self/my_file.c
  QString mShortName;  // e.g. myfile.c
  LineNumberArea *mLineNumberArea;
  WbLanguage *mLanguage;
  QCompleter *mCompleter;
  QList<QTextEdit::ExtraSelection> mExtraSelections;
  QFileSystemWatcher *mWatcher;
  WbSyntaxHighlighter *mSyntaxHighlighter;

  WbClipboard *mClipboard;

  void watch();
  void unwatch();
  bool mFileModifiedBySaveAction;
  bool mReloadPromptExists;

  // current word from start to cursor
  QString currentWordPrefix() const;
  void setLanguage(WbLanguage *lang);
  void createExtraSelections();
  int findMatchingParenthesis(int start, QChar type) const;
  void markParenthesis(int start, int end);
  void unmarkParenthesis();

  // manage indentation
  void addNewLine();
};

#endif
