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

#include "WbTextBuffer.hpp"

#include "WbActionManager.hpp"
#include "WbClipboard.hpp"
#include "WbFileUtil.hpp"
#include "WbLanguage.hpp"
#include "WbMessageBox.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"
#include "WbSyntaxHighlighter.hpp"
#include "WbTextFind.hpp"
#include "WbVariant.hpp"

#include <QtCore/QFileSystemWatcher>
#include <QtCore/QTime>
#include <QtGui/QPainter>
#include <QtGui/QTextDocumentFragment>
#include <QtWidgets/QAbstractItemView>
#include <QtWidgets/QCompleter>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QScrollBar>

class LineNumberArea : public QWidget {
public:
  explicit LineNumberArea(WbTextBuffer *editor) : QWidget(editor) { textEdit = editor; }

  QSize sizeHint() const override { return QSize(textEdit->lineNumberAreaWidth(), 0); }

protected:
  void paintEvent(QPaintEvent *event) override { textEdit->lineNumberAreaPaintEvent(event); }

private:
  WbTextBuffer *textEdit;
};

WbTextBuffer::WbTextBuffer(QWidget *parent) : QPlainTextEdit(parent) {
  setObjectName("TextBuffer");

  // overwrite selection highlight format
  // resetting the automatic format applied when searching for some text
  QPalette p = palette();
  p.setColor(QPalette::Highlight, p.color(QPalette::Highlight));
  p.setColor(QPalette::HighlightedText, p.color(QPalette::HighlightedText));
  setPalette(p);

  mClipboard = WbClipboard::instance();

  mFileName = WbStandardPaths::unnamedTextFile();
  mShortName = mFileName;
  mLanguage = WbLanguage::findByCode(WbLanguage::PLAIN_TEXT);
  mCompleter = NULL;

  createExtraSelections();

  mLineNumberArea = new LineNumberArea(this);
  connect(this, &WbTextBuffer::blockCountChanged, this, &WbTextBuffer::updateLineNumberAreaWidth);
  connect(this, &WbTextBuffer::updateRequest, this, &WbTextBuffer::updateLineNumberArea);
  connect(this, &WbTextBuffer::cursorPositionChanged, this, &WbTextBuffer::matchParentheses);
  updateLineNumberAreaWidth(0);

  setLineWrapMode(QPlainTextEdit::NoWrap);

  updateFont();

  mSyntaxHighlighter = WbSyntaxHighlighter::createForLanguage(WbLanguage::findByCode(WbLanguage::PLAIN_TEXT), document());

  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbTextBuffer::updateFont);

  mFileModifiedBySaveAction = false;
  mReloadPromptExists = false;
  mWatcher = new QFileSystemWatcher(this);
  connect(mWatcher, &QFileSystemWatcher::fileChanged, this, &WbTextBuffer::askForReverting);
  watch();
}

WbTextBuffer::~WbTextBuffer() {
}

void WbTextBuffer::ignoreFileChangedEvent() {
  disconnect(mWatcher, &QFileSystemWatcher::fileChanged, this, &WbTextBuffer::askForReverting);
}

int WbTextBuffer::lineNumberAreaWidth() {
  int digits = 1;
  int max = qMax(1, blockCount());
  while (max >= 10) {
    max /= 10;
    ++digits;
  }

  return 3 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * digits;
}

void WbTextBuffer::updateLineNumberAreaWidth(int) {
  setViewportMargins(lineNumberAreaWidth(), 0, 0, 0);
}

void WbTextBuffer::updateLineNumberArea(const QRect &rect, int dy) {
  if (dy)
    mLineNumberArea->scroll(0, dy);
  else
    mLineNumberArea->update(0, rect.y(), mLineNumberArea->width(), rect.height());

  if (rect.contains(viewport()->rect()))
    updateLineNumberAreaWidth(0);
}

void WbTextBuffer::resizeEvent(QResizeEvent *event) {
  QPlainTextEdit::resizeEvent(event);
  QRect cr = contentsRect();
  mLineNumberArea->setGeometry(QRect(cr.left(), cr.top(), lineNumberAreaWidth(), cr.height()));
}

// Based on: qt/examples/widgets/codeeditor/codeeditor.cpp
void WbTextBuffer::lineNumberAreaPaintEvent(QPaintEvent *event) {
  QPainter painter(mLineNumberArea);
  painter.fillRect(event->rect(), mGutterBackgroundColor);

  QTextBlock block = firstVisibleBlock();
  int blockNumber = block.blockNumber();
  int top = (int)blockBoundingGeometry(block).translated(contentOffset()).top();
  int bottom = top + (int)blockBoundingRect(block).height();

  while (block.isValid() && top <= event->rect().bottom()) {
    if (block.isVisible() && bottom >= event->rect().top()) {
      QString number = QString::number(blockNumber + 1);
      painter.setPen(mGutterForegroundColor);
      painter.drawText(0, top, mLineNumberArea->width(), fontMetrics().height(), Qt::AlignRight, number);
    }

    block = block.next();
    top = bottom;
    bottom = top + (int)blockBoundingRect(block).height();
    ++blockNumber;
  }
}

void WbTextBuffer::setLanguage(WbLanguage *lang) {
  if (lang != mLanguage) {
    mLanguage = lang;
    mSyntaxHighlighter = WbSyntaxHighlighter::createForLanguage(mLanguage, document(), mSyntaxHighlighter);
    delete mCompleter;
    mCompleter = NULL;
  }

  if (mLanguage && !mCompleter && !isReadOnly()) {
    mCompleter = new QCompleter(mLanguage->autoCompletionWords(), this);
    mCompleter->setWrapAround(false);
    mCompleter->setWidget(this);
    mCompleter->setCompletionMode(QCompleter::PopupCompletion);
    void (QCompleter::*completerActivated)(const QString &) = &QCompleter::activated;
    connect(mCompleter, completerActivated, this, &WbTextBuffer::insertCompletion);
  }
}

void WbTextBuffer::setFileName(const QString &fileName) {
  QFileInfo fi(fileName);
  mFileName = fi.canonicalFilePath();
  mShortName = fi.fileName();
  setLanguage(WbLanguage::findByFileName(mShortName));

  watch();

  emit fileNameChanged();
}

void WbTextBuffer::askForReverting() {
  // Fixed Qt bug: QFileSystemWatcher::fileChanged signal is sent twice
  // https://bugreports.qt-project.org/browse/QTBUG-23096
  static bool firstCall = true;
  static QString previousFileName;
  static QTime previousTime;
  if (firstCall)
    firstCall = false;
  else if (mFileName == previousFileName && QTime::currentTime() < previousTime.addSecs(1))
    return;

  if (mFileModifiedBySaveAction) {
    mFileModifiedBySaveAction = false;
    return;
  }

  // show text editor if hidden
  emit showRequested();

  if (QFile::exists(mFileName)) {
    // auto reload file if we risk losing nothing
    if (!isModified())
      revert(false);
    // else make sure only one reload prompt appears
    else if (mReloadPromptExists == false) {
      mReloadPromptExists = true;
      int result =
        WbMessageBox::question(tr("The file \"%1\" open in the Webots text editor was modified by an external action. Would "
                                  "you like to revert the text editor buffer (Any modification will be lost)?")
                                 .arg(mFileName),
                               this, tr("Text editor asks for reverting file"), QMessageBox::Ok);
      if (result == QMessageBox::Ok)
        revert(false);

      mReloadPromptExists = false;
    }
  } else {
    WbMessageBox::info(tr("The file \"%1\" open in the Webots text editor was deleted by an external action.").arg(mFileName),
                       this, tr("Text editor has detected file deletion"));

    // keep name but discard path
    mShortName = QFileInfo(mFileName).fileName();
    mFileName = mShortName;
    emit fileNameChanged();
    document()->setModified(true);
  }

  // Fixed Qt bug: QFileSystemWatcher::fileChanged is sent twice
  // https://bugreports.qt-project.org/browse/QTBUG-23096
  previousFileName = mFileName;
  previousTime = QTime::currentTime();
}

void WbTextBuffer::watch() {
  unwatch();

  if (QFile::exists(mFileName))
    mWatcher->addPath(mFileName);
}

void WbTextBuffer::unwatch() {
  if (mWatcher->files().size() > 0)
    mWatcher->removePaths(mWatcher->files());
}

QDir WbTextBuffer::fileDir() const {
  return QFileInfo(mFileName).absoluteDir();
}

QString WbTextBuffer::path() const {
  return QFileInfo(mFileName).absolutePath();
}

bool WbTextBuffer::load(const QString &fn, const QString &title) {
  QFile file(fn);
  if (!file.open(QFile::ReadOnly))
    return false;

  if (!title.isEmpty() || WbFileUtil::isLocatedInDirectory(fn, WbStandardPaths::webotsTmpPath()) ||
      WbFileUtil::isLocatedInDirectory(fn, WbStandardPaths::cachedAssetsPath())) {
    setReadOnly(true);
    setUndoRedoEnabled(false);
  }

  QByteArray data = file.readAll();
  setPlainText(QString::fromUtf8(data));
  // we only need to set a different title to the tab in case of cached assets
  setFileName(title.isEmpty() ? fn : title);

  return true;
}

bool WbTextBuffer::revert(bool askUser) {
  if (askUser && WbMessageBox::question(tr("Revert to saved file? Changes will be lost."), this) == QMessageBox::Cancel)
    return false;

  return load(fileName());
}

bool WbTextBuffer::saveAs(const QString &newName) {
  unwatch();

  mFileModifiedBySaveAction = true;

  QFile file(newName);
  if (!file.open(QIODevice::WriteOnly)) {
    watch();
    return false;
  }

  file.write(document()->toPlainText().toUtf8());
  if (!file.flush()) {
    watch();
    return false;
  }

  file.close();
  document()->setModified(false);
  setReadOnly(false);
  setUndoRedoEnabled(true);
  setFileName(newName);

  watch();

  return true;
}

bool WbTextBuffer::save() {
  return saveAs(mFileName);
}

void WbTextBuffer::cut() {
  assert(!isReadOnly());
  QTextCursor cursor = textCursor();
  if (cursor.hasSelection()) {
    mClipboard->setString(cursor.selection().toPlainText());
    cursor.removeSelectedText();
  }
}

void WbTextBuffer::copy() const {
  if (textCursor().hasSelection())
    mClipboard->setString(textCursor().selection().toPlainText());
}

void WbTextBuffer::paste() {
  assert(!isReadOnly());
  if (mClipboard->isEmpty())
    return;

  const QString text = mClipboard->stringValue();
  if (!text.isEmpty())
    textCursor().insertText(text);
}

bool WbTextBuffer::isModified() const {
  return document() && document()->isModified();
}

bool WbTextBuffer::hasSelection() const {
  return textCursor().hasSelection();
}

bool WbTextBuffer::hasSingleBlockSelection() const {
  QTextCursor cur = textCursor();
  return cur.hasSelection() && cur.block() == document()->findBlock(cur.anchor());
}

bool WbTextBuffer::isUnnamed() const {
  return mFileName == WbStandardPaths::unnamedTextFile();
}

void WbTextBuffer::insertCompletion(const QString &completion) {
  if (mCompleter->widget() != this)
    return;

  QTextCursor tc = textCursor();
  int extra = completion.length() - mCompleter->completionPrefix().length();
  tc.insertText(completion.right(extra));
  setTextCursor(tc);
}

QString WbTextBuffer::currentWordPrefix() const {
  QTextCursor tc = textCursor();
  int pos = tc.position();
  // move one character left otherwise if cursor is just before a word
  // separator character the word is not correctly selected
  // for the same reason the QTextCursor::WordUnderCursor selection
  // cannot be used
  tc.movePosition(QTextCursor::Left);
  tc.movePosition(QTextCursor::StartOfWord);
  tc.setPosition(pos, QTextCursor::KeepAnchor);
  return tc.selectedText();
}

void WbTextBuffer::focusInEvent(QFocusEvent *event) {
  if (mCompleter)
    mCompleter->setWidget(this);

  QPlainTextEdit::focusInEvent(event);
  WbActionManager::instance()->setFocusObject(this);
  emit focusIn();
}

void WbTextBuffer::focusOutEvent(QFocusEvent *event) {
  if (WbActionManager::instance()->focusObject() == this)
    WbActionManager::instance()->setFocusObject(NULL);
}

void WbTextBuffer::indent(IndentMode mode) {
  assert(!isReadOnly());
  QTextCursor cur = textCursor();
  int initialAnchor = cur.anchor();
  int initialPosition = cur.position();

  // handle backward selection
  if (initialAnchor > initialPosition) {
    initialAnchor = initialPosition;
    initialPosition = cur.anchor();
  }

  int blockStartPosition = initialAnchor;

  // save a new anchor at the beginning of the line of the selected text
  cur.setPosition(blockStartPosition);
  cur.movePosition(QTextCursor::StartOfBlock, QTextCursor::MoveAnchor);
  blockStartPosition = cur.position();

  // set a new selection with the new beginning
  cur.setPosition(blockStartPosition);
  cur.setPosition(initialPosition, QTextCursor::KeepAnchor);

  // get the selected text and split into lines
  QString str = cur.selection().toPlainText();
  QStringList list = str.split("\n");
  const int indentSize = mLanguage->code() == WbLanguage::PYTHON ? 4 : 2;

  if (mode == INCREASE) {
    // insert two or four spaces at the beginning of each line
    initialAnchor += indentSize;
    initialPosition += indentSize * list.count();
    for (int i = 0; i < list.count(); i++)
      if (mLanguage->code() == WbLanguage::PYTHON)
        list[i].insert(0, "    ");
      else
        list[i].insert(0, "  ");
  } else {
    // remove two or four spaces from the beginning of each line
    for (int i = 0; i < list.count(); i++) {
      if ((list[i].startsWith("    ") && mLanguage->code() == WbLanguage::PYTHON) || list[i].startsWith(" ")) {
        list[i].remove(0, indentSize);

        if (i == 0)
          initialAnchor -= indentSize;
        initialPosition -= indentSize;
      }
    }
  }

  if (blockStartPosition > initialAnchor)
    initialAnchor = blockStartPosition;

  // put the new text back
  str = list.join("\n");
  cur.insertText(str);
  // reset initial selection
  cur.setPosition(initialAnchor);
  cur.setPosition(initialPosition, QTextCursor::KeepAnchor);

  // put the whole thing back into the main text
  setTextCursor(cur);
}

void WbTextBuffer::keyPressEvent(QKeyEvent *event) {
  // cut, copy and paste action are handled in WbTextBuffer
  if (event->matches(QKeySequence::Copy)) {
    copy();
    return;
  }

  if (isReadOnly())
    return;

  if (event->matches(QKeySequence::Cut)) {
    cut();
    return;
  }

  if (event->matches(QKeySequence::Paste)) {
    paste();
    return;
  }

  if (event->matches(QKeySequence::Save)) {
    if (!save())
      WbMessageBox::warning(tr("Unable to save '%1'.").arg(mFileName));
    return;
  }

  if (mCompleter && mCompleter->popup()->isVisible()) {
    // The following keys are forwarded by the completer to the widget
    switch (event->key()) {
      case Qt::Key_Enter:
      case Qt::Key_Return:
      case Qt::Key_Escape:
      case Qt::Key_Tab:
      case Qt::Key_Backtab:
        event->ignore();
        return;  // let the completer do default behavior
      default:
        break;
    }
  }

  if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
    addNewLine();
    return;
  }

  if (event->key() == Qt::Key_Backtab) {
    indent(DECREASE);
    return;
  }

  if (event->key() == Qt::Key_Tab) {
    indent(INCREASE);
    return;
  }

  bool isPopupShortcut = ((event->modifiers() & Qt::ControlModifier) && event->key() == Qt::Key_Space);  // CTRL + Space
  if (!mCompleter || !isPopupShortcut)  // dont process the shortcut when we have a completer
    QPlainTextEdit::keyPressEvent(event);

  bool ctrl = event->modifiers() & Qt::ControlModifier;
  if (mCompleter && ctrl && !isPopupShortcut) {
    // hide so that action shortcuts (e.g. CTRL+S) can be automatically handled by the text editor
    mCompleter->popup()->hide();
    return;
  }

  const bool ctrlOrShift = ctrl || (event->modifiers() & Qt::ShiftModifier);
  if (!mCompleter || (ctrlOrShift && event->text().isEmpty()))
    return;

  // static QString eow("~!@#$%^&*()_+{}|:\"<>?,./;'[]\\-="); // end of word
  bool hasModifier = (event->modifiers() != Qt::NoModifier) && !ctrlOrShift;
  QString completionPrefix = currentWordPrefix();

  // if (!isShortcut && (hasModifier || e->text().isEmpty() || completionPrefix.length() < 4 ||
  // eow.contains(e->text().right(1)))) {
  if (!isPopupShortcut && (hasModifier || completionPrefix.length() < 2)) {
    mCompleter->popup()->hide();
    return;
  }

  if (event->text().isEmpty())
    // don't show completer popup window if no key with text is pressed
    return;

  if (completionPrefix != mCompleter->completionPrefix()) {
    mCompleter->setCompletionPrefix(completionPrefix);
    mCompleter->popup()->setCurrentIndex(mCompleter->completionModel()->index(0, 0));
  }

  QRect cr = cursorRect();
  cr.setWidth(mCompleter->popup()->sizeHintForColumn(0) + mCompleter->popup()->verticalScrollBar()->sizeHint().width());
  mCompleter->complete(cr);  // popup it up!
}

void WbTextBuffer::markParenthesis(int start, int end) {
  // start
  QTextCursor cursor = textCursor();
  cursor.setPosition(start);
  cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::KeepAnchor);
  mExtraSelections[0].cursor = cursor;

  // end
  cursor = textCursor();
  cursor.setPosition(end);
  cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::KeepAnchor);
  mExtraSelections[1].cursor = cursor;

  setExtraSelections(mExtraSelections);
}

void WbTextBuffer::unmarkParenthesis() {
  mExtraSelections[0].cursor = QTextCursor();
  mExtraSelections[1].cursor = QTextCursor();

  setExtraSelections(mExtraSelections);
}

static bool isOpeningParenthesis(QChar ch) {
  return ch == '(' || ch == '{' || ch == '[';
}

static bool isClosingParenthesis(QChar ch) {
  return ch == ')' || ch == '}' || ch == ']';
}

static bool isParenthesis(QChar ch) {
  return isOpeningParenthesis(ch) || isClosingParenthesis(ch);
}

static QChar oppositeParenthesis(QChar ch) {
  const QString REG("(){}[]");
  const QString OPP(")(}{][");
  return OPP.at(REG.indexOf(ch));
}

int WbTextBuffer::findMatchingParenthesis(int start, QChar type) const {
  QTextDocument *doc = document();
  int count = doc->characterCount();
  QChar opp = oppositeParenthesis(type);
  int inc = isOpeningParenthesis(type) ? +1 : -1;
  int k = 1;

  for (int pos = start + inc; pos >= 0 && pos < count; pos += inc) {
    QChar ch = doc->characterAt(pos);
    if (ch == type)
      k++;
    else if (ch == opp) {
      k--;
      if (k == 0)
        return pos;
    }
  }

  return -1;
}

void WbTextBuffer::matchParentheses() {
  int position = textCursor().position();
  QTextDocument *doc = document();

  // check first character on the right and left of cursor
  QChar type = doc->characterAt(position - 1);
  int start = -1;
  if (isParenthesis(type))
    start = position - 1;
  else {
    type = doc->characterAt(position);
    if (isParenthesis(type))
      start = position;
  }

  if (start == -1) {
    // start parenthesis not found
    unmarkParenthesis();
    return;
  }

  int end = findMatchingParenthesis(start, type);
  if (end == -1) {
    // end parenthesis not found
    unmarkParenthesis();
    return;
  }

  // mark opening and closing parentheses
  markParenthesis(start, end);
}

void WbTextBuffer::goToLine() {
  int max = document()->lineCount();
  bool ok;
  int line = QInputDialog::getInt(this, "Go To Line", "Enter the line you want to go to:", 1, 1, max, 1, &ok);
  if (!ok)
    return;

  QTextBlock block = document()->findBlockByLineNumber(line - 1);
  setTextCursor(QTextCursor(block));
  ensureCursorVisible();
}

void WbTextBuffer::createExtraSelections() {
  // brace matching format
  QTextEdit::ExtraSelection s1;
  s1.format.setBackground(Qt::magenta);
  mExtraSelections.append(s1);  // opening brace
  mExtraSelections.append(s1);  // closing brace

  // highlighted error line
  QTextEdit::ExtraSelection s2;
  s2.format.setUnderlineStyle(QTextCharFormat::SpellCheckUnderline);
  s2.format.setUnderlineColor(Qt::red);
  mExtraSelections.append(s2);
}

void WbTextBuffer::unmarkError() {
  mExtraSelections[2].cursor = QTextCursor();
  setExtraSelections(mExtraSelections);
}

void WbTextBuffer::markError(int line, int column) {
  // qDebug() << "WbTextBuffer::markError(" << line + 1 << ", " << column + 1 << ")";

  QTextBlock block = document()->findBlockByLineNumber(line);

  // scroll window
  QTextCursor cursor(block);
  setTextCursor(cursor);
  ensureCursorVisible();

  if (column < 0) {
    // column number is not known: select from the beginning of the first word to the end of the line
    if (cursor.movePosition(QTextCursor::EndOfWord))
      cursor.movePosition(QTextCursor::StartOfWord);
    else
      cursor.movePosition(QTextCursor::NextWord);
    cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
  } else {
    // column number is known: select exact word
    cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::MoveAnchor, column);
    cursor.movePosition(QTextCursor::EndOfWord, QTextCursor::KeepAnchor);
  }

  // mark
  mExtraSelections[2].cursor = cursor;
  setExtraSelections(mExtraSelections);
}

void WbTextBuffer::addNewLine() {
  assert(!isReadOnly());
  QTextCursor cursor = textCursor();
  cursor.beginEditBlock();
  cursor.removeSelectedText();

  // compute indentation
  cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
  QString lineContent = cursor.selectedText();
  cursor.setPosition(cursor.anchor());
  int i = 0;
  while (i < lineContent.size() && lineContent.at(i).isSpace())
    i++;

  // add new line with the same indent as previous line
  cursor.insertBlock();
  cursor.insertText(lineContent.left(i));

  setTextCursor(cursor);
  cursor.endEditBlock();
}

void WbTextBuffer::toggleLineComment() {
  assert(!isReadOnly());

  // select the appropriate comment or do nothing in case of unknown an language
  QString comment = WbLanguage::findByFileName(fileName())->commentPrefix() + " ";

  QTextCursor cursor = textCursor();
  cursor.beginEditBlock();
  int selectionEnd = cursor.selectionEnd();
  int selectionStart = cursor.selectionStart();

  cursor.setPosition(selectionEnd);
  cursor.movePosition(QTextCursor::StartOfLine);
  int lastSelectedLineStart = cursor.position();
  int endOffset = selectionEnd - lastSelectedLineStart;

  cursor.setPosition(selectionStart);
  cursor.movePosition(QTextCursor::StartOfLine);
  bool firstLine = true;
  // for each selected line
  while (cursor.position() <= lastSelectedLineStart) {
    cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
    QString lineContent = cursor.selectedText();
    int index = 0;
    while (index < lineContent.size() && lineContent.at(index).isSpace()) {
      index++;
    }

    if (index < lineContent.size()) {
      QString substring = lineContent.mid(index, comment.length());
      if (substring == comment) {
        // remove comment
        lineContent.remove(index, comment.length());
        lastSelectedLineStart -= comment.length();
        if (firstLine) {
          selectionStart -= comment.length();
          firstLine = false;
        }

      } else {
        // add comment
        lineContent.insert(index, comment);
        lastSelectedLineStart += comment.length();
        if (firstLine) {
          selectionStart += comment.length();
          firstLine = false;
        }
      }

      cursor.insertText(lineContent);
    }

    if (cursor.atEnd())
      break;  // end of document

    cursor.movePosition(QTextCursor::Down);
    cursor.movePosition(QTextCursor::StartOfLine);
  }

  // reset selection
  cursor.setPosition(selectionStart);
  cursor.setPosition(lastSelectedLineStart + endOffset, QTextCursor::KeepAnchor);
  setTextCursor(cursor);
  cursor.endEditBlock();
}

void WbTextBuffer::updateFont() {
  WbPreferences *prefs = WbPreferences::instance();
  QFont font;
  font.fromString(prefs->value("Editor/font").toString());
  setFont(font);
  mLineNumberArea->setFont(font);
}

void WbTextBuffer::updateSearchTextHighlighting(QRegularExpression regularExpression) {
  if (regularExpression.pattern().isEmpty())
    disconnect(this, &QPlainTextEdit::selectionChanged, this, &WbTextBuffer::resetSearchTextHighlighting);

  mSyntaxHighlighter->setSearchTextRule(regularExpression);

  if (!regularExpression.pattern().isEmpty())
    connect(this, &QPlainTextEdit::selectionChanged, this, &WbTextBuffer::resetSearchTextHighlighting, Qt::UniqueConnection);
}

void WbTextBuffer::resetSearchTextHighlighting() {
  updateSearchTextHighlighting(QRegularExpression());
}
