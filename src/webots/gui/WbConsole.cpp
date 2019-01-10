// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbConsole.hpp"

#include "WbActionManager.hpp"
#include "WbBuildEditor.hpp"
#include "WbClipboard.hpp"
#include "WbDockTitleBar.hpp"
#include "WbFindReplaceDialog.hpp"
#include "WbLog.hpp"
#include "WbMessageBox.hpp"
#include "WbPreferences.hpp"
#include "WbSyntaxHighlighter.hpp"
#include "WbTextFind.hpp"

#include <QtGui/QTextBlock>
#include <QtGui/QTextDocumentFragment>

#include <QtWidgets/QAction>
#include <QtWidgets/QMenu>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QStyle>

#include <cassert>
#include <iostream>

#include <ode/ode.h>  // for message handlers

// plain text edit with single line highlighting
class ConsoleEdit : public QPlainTextEdit {
public:
  explicit ConsoleEdit(QWidget *parent) : QPlainTextEdit(parent) {
    setObjectName("ConsoleEdit");
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, &QPlainTextEdit::customContextMenuRequested, this, &ConsoleEdit::showCustomContextMenu);

    // overwrite selection highlight format
    // resetting the automatic format applied when searching for some text
    QPalette p = palette();
    p.setColor(QPalette::Highlight, p.color(QPalette::Highlight));
    p.setColor(QPalette::HighlightedText, p.color(QPalette::HighlightedText));
    setPalette(p);

    mSyntaxHighlighter = WbSyntaxHighlighter::createForLanguage(NULL, document());
    connect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting);

    // listen to clear console keyboard shortcut
    addAction(WbActionManager::instance()->action(WbActionManager::CLEAR_CONSOLE));
    document()->setDefaultStyleSheet("span{\n  white-space:pre;\n}\n");
  }

  ~ConsoleEdit() { delete mSyntaxHighlighter; }

  void copy() {
    if (textCursor().hasSelection())
      WbClipboard::instance()->setString(textCursor().selection().toPlainText());
  }

  void mouseDoubleClickEvent(QMouseEvent *event) override {
    if (event->button() != Qt::LeftButton)
      return;

    // find position of double-click
    QTextCursor cursor(cursorForPosition(event->pos()));

    // select line under cursor
    cursor.movePosition(QTextCursor::StartOfLine);
    cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);

    // inform text mEditor
    static_cast<WbConsole *>(parent())->jumpToError(cursor.selectedText());

    // mark line
    QList<QTextEdit::ExtraSelection> selections;
    QTextEdit::ExtraSelection selection;
    selection.format.setBackground(Qt::lightGray);
    selection.cursor = cursor;
    selections.append(selection);
    setExtraSelections(selections);
  }

public slots:
  void updateSearchTextHighlighting(QRegExp regExp) {
    if (regExp.isEmpty())
      disconnect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting);

    mSyntaxHighlighter->setSearchTextRule(regExp);

    if (!regExp.isEmpty())
      connect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting, Qt::UniqueConnection);
  }

protected:
  void keyPressEvent(QKeyEvent *event) override {
    if (event->modifiers() == Qt::ControlModifier) {
      switch (event->key()) {
        case Qt::Key_A:
          selectAll();
          event->accept();
          return;
        case Qt::Key_C:
          copy();
          event->accept();
          return;
        default:
          break;
      }
    }

    QPlainTextEdit::keyPressEvent(event);
  }

  void keyReleaseEvent(QKeyEvent *event) override { event->ignore(); }

  void focusInEvent(QFocusEvent *event) override {
    QPlainTextEdit::focusInEvent(event);

    // update application actions
    WbActionManager *actionManager = WbActionManager::instance();
    actionManager->setFocusObject(this);
    actionManager->enableTextEditActions(false);
    actionManager->setEnabled(WbActionManager::COPY, textCursor().hasSelection());
    actionManager->setEnabled(WbActionManager::SELECT_ALL, true);
    actionManager->setEnabled(WbActionManager::FIND, true);
    actionManager->setEnabled(WbActionManager::FIND_NEXT, true);
    actionManager->setEnabled(WbActionManager::FIND_PREVIOUS, true);
    actionManager->setEnabled(WbActionManager::CUT, false);
    actionManager->setEnabled(WbActionManager::PASTE, false);
    actionManager->setEnabled(WbActionManager::UNDO, false);
    actionManager->setEnabled(WbActionManager::REDO, false);
  }

  void focusOutEvent(QFocusEvent *event) override {
    if (WbActionManager::instance()->focusObject() == this)
      WbActionManager::instance()->setFocusObject(NULL);
  }

private:
  WbSyntaxHighlighter *mSyntaxHighlighter;

private slots:
  void showCustomContextMenu(const QPoint &pt);

  void resetSearchTextHighlighting() { updateSearchTextHighlighting(QRegExp()); }
};

void ConsoleEdit::showCustomContextMenu(const QPoint &pt) {
  QMenu *menu = createStandardContextMenu();
  menu->addAction(WbActionManager::instance()->action(WbActionManager::FIND));
  menu->addSeparator();
  menu->addAction(WbActionManager::instance()->action(WbActionManager::CLEAR_CONSOLE));
  menu->exec(mapToGlobal(pt));
  delete menu;
}

static bool gStdoutTee = false;
static bool gStderrTee = false;
static WbConsole *gInstance = NULL;

void WbConsole::enableStdOutRedirectToTerminal() {
  gStdoutTee = true;
};

void WbConsole::enableStdErrRedirectToTerminal() {
  gStderrTee = true;
};

WbConsole *WbConsole::instance() {
  return gInstance;
}

namespace {
  void odeErrorFunc(int errnum, const char *msg, va_list ap) {
    QString error;
    error.vsprintf(msg, ap);
    emit WbLog::instance()->logEmitted(WbLog::ERROR, QString("ODE Error %1: ").arg(errnum) + error, false);
  }

  void odeDebugFunc(int errnum, const char *msg, va_list ap) {
    QString debug;
    debug.vsprintf(msg, ap);
    emit WbLog::instance()->logEmitted(WbLog::DEBUG, QString("ODE INTERNAL ERROR %1: ").arg(errnum) + debug, false);
  }

  void odeMessageFunc(int errnum, const char *msg, va_list ap) {
    QString message;
    message.vsprintf(msg, ap);
    if (message.startsWith("LCP")) {
      message = QString("The current physics step could not be computed correctly. "
                        "Your world may be too complex. If this problem persists, try simplifying "
                        "your bounding object(s), reducing the number of joints, or reducing "
                        "WorldInfo.basicTimeStep.");

      emit WbLog::instance()->logEmitted(WbLog::WARNING, QString("WARNING: ") + message, false);
    } else
      emit WbLog::instance()->logEmitted(WbLog::WARNING, QString("ODE Message %1: ").arg(errnum) + message, false);
  }
}  // namespace

WbConsole::WbConsole(QWidget *parent) :
  WbDockWidget(parent),
  mEditor(new ConsoleEdit(this)),
  mClearAction(new QAction(this)),
  mErrorPatterns(createErrorMatchingPatterns()),  // patterns for error matching
  mBold(false),
  mIsOverwriteEnabled(false),  // option to overwrite last line
  mFindDialog(NULL),
  mTextFind(new WbTextFind(mEditor)) {
  setWindowTitle("Console");
  setTabbedTitle("Console");
  setObjectName("Console");
  gInstance = this;

  // setup for main window
  QAction *const action = toggleViewAction();
  action->setText("Console");
  action->setStatusTip("Toggle the view of the console.");
  action->setShortcut(Qt::CTRL + Qt::Key_L);

  titleBarWidget()->setObjectName("consoleTitleBar");
  titleBarWidget()->style()->polish(titleBarWidget());

  // create text editor
  mEditor->setReadOnly(true);
  mEditor->setMaximumBlockCount(5000);  // limit the memory usage
  mEditor->setFocusPolicy(Qt::ClickFocus);
  setWidget(mEditor);

  connect(mEditor, &ConsoleEdit::copyAvailable, this, &WbConsole::enableCopyAction);
  connect(WbActionManager::instance(), &WbActionManager::userConsoleEditCommandReceived, this, &WbConsole::handleUserCommand);

  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbConsole::updateFont);
  updateFont();

  connect(WbActionManager::instance()->action(WbActionManager::CLEAR_CONSOLE), &QAction::triggered, this, &WbConsole::clear);

  connect(mTextFind, &WbTextFind::findStringChanged, mEditor, &ConsoleEdit::updateSearchTextHighlighting);

  // listen to WbLog
  connect(WbLog::instance(), SIGNAL(logEmitted(WbLog::Level, const QString &, bool)), this,
          SLOT(appendLog(WbLog::Level, const QString &, bool)));
  connect(WbLog::instance(), SIGNAL(controllerLogEmitted(WbLog::Level, const QString &, const QString &, bool)), this,
          SLOT(appendLog(WbLog::Level, const QString &, const QString &, bool)));
  connect(WbLog::instance(), SIGNAL(cleared()), this, SLOT(clear()));

  // Install ODE message handlers
  dSetErrorHandler(odeErrorFunc);
  dSetDebugHandler(odeDebugFunc);
  dSetMessageHandler(odeMessageFunc);
}

WbConsole::~WbConsole() {
  for (int i = 0; mErrorPatterns[i]; ++i)
    delete mErrorPatterns[i];
  gInstance = NULL;
}

void WbConsole::clear() {
  mEditor->clear();
}

QString WbConsole::htmlSpan(const QString &s, WbLog::Level level) const {
  if (s.isEmpty() || s == "\n")
    return "";

  QString color;
  bool bold;
  if (level == WbLog::ERROR || level == WbLog::FATAL || level == WbLog::STDERR) {
    color = errorColor();
    bold = true;
  } else if (level == WbLog::WARNING || level == WbLog::DEBUG) {
    color = errorColor();
    bold = false;
  } else if (level == WbLog::INFO || level == WbLog::STATUS) {
    color = infoColor();
    bold = false;
  } else {
    assert(level == WbLog::STDOUT);
    color = mColor;
    bold = mBold;
  }
  QString span("<span");
  if (!color.isEmpty() || bold) {
    span += " style=\"";
    if (!color.isEmpty())
      span += "color:" + color + ";";
    if (bold)
      span += "font-weight:bold;";
    span += "\"";
  }
  span += ">" + s.toHtmlEscaped() + "</span>";
  return span;
}

void WbConsole::handleCRAndLF(const QString &msg) {
  // handle CR (\r) and LF (\n) characters
  static bool lastLineEndsWithLF = false;

  QString html(msg);
  if (html.isEmpty())
    return;

#ifdef _WIN32
  html.replace("\r\n", "\n");  // use unix new line syntax
#endif
  html.replace("\r\n", "\n");  // CR has no effect if followed by a new line

  if (html == "<span>\n</span>" && lastLineEndsWithLF) {
    mEditor->appendHtml("");  // add empty line
    lastLineEndsWithLF = true;
    mIsOverwriteEnabled = false;
    return;
  }

  if (html.startsWith("<span>\n") && !lastLineEndsWithLF) {
    html.remove(6, 1);  // remove additional empty line
    mIsOverwriteEnabled = false;
  }

  if (html.endsWith("\n</span>")) {
    html.remove(-8, 1);  // text printed automatically on a new line
    lastLineEndsWithLF = true;
  } else
    lastLineEndsWithLF = false;

  bool lastLineEndsWithCR = false;
  if (html.endsWith("\r</span>")) {
    html.remove(-8, 1);
    lastLineEndsWithCR = !lastLineEndsWithLF;
  }
  html.remove("<span></span>");

  // handle CR in the middle of the text block
  const QStringList linesList(html.split("\r"));
  const int linesCount = linesList.size();
  bool endsWithCR = (linesCount != 1);

  for (int i = 0; i < linesCount; ++i) {
    QString line(linesList.at(i));

    if (i == linesCount - 1)
      endsWithCR = lastLineEndsWithCR;

    if (!line.isEmpty()) {
      if (mIsOverwriteEnabled) {
        // move the cursor to the beginning of last line
        QTextCursor textCursor = mEditor->textCursor();
        textCursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
        textCursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        textCursor.movePosition(QTextCursor::End, QTextCursor::KeepAnchor);
        QString previousLine = textCursor.selectedText();
        textCursor.removeSelectedText();

        if (!mPrefix.isEmpty())
          line.prepend(mPrefix);  // append '[controller_name] '

        // remove span HTML tags to compute current text length
        QString plainLine(line);
        plainLine.remove("</span>");
        int spanIndex = plainLine.indexOf("<span");
        while (spanIndex != -1) {
          int length = 5;
          while (plainLine[spanIndex + length] != '>')
            ++length;
          ++length;

          plainLine.remove(spanIndex, length);
          spanIndex = plainLine.indexOf("<span");
        }
        const int plainLineSize = plainLine.size();

        const int previousLineSize = previousLine.size();
        if (previousLineSize > plainLineSize)
          // append non-overwritten characters from previous line
          line.append(previousLine.mid(plainLineSize, previousLineSize - plainLineSize));

        textCursor.insertHtml(line);

      } else
        mEditor->appendHtml(line);
      mIsOverwriteEnabled = endsWithCR;
    } else
      mIsOverwriteEnabled = mIsOverwriteEnabled || endsWithCR;
  }
}

void WbConsole::handlePossibleAnsiEscapeSequences(const QString &msg, WbLog::Level level) {
  int i = msg.indexOf("\033[");
  if (i != -1) {  // contains ANSI escape sequences
    QString html;
    if (i != 0)
      html = htmlSpan(msg.mid(0, i), level);  // add the text before the ANSI escape sequence
    while (1) {
      i += 2;  // skip the "\033[" chars
      QString sequence;
      while (msg[i] != 'm')
        sequence += msg[i++];
      i++;

      const QStringList codes(sequence.split(";"));  // handle multiple (e.g. sequence "ESC[0;39m" )
      foreach (const QString code, codes) {
        // the stored sequence may be "0" or "1", "30", "31", "32", etc.
        if (code == "0") {  // reset to default
          mColor = ansiBlack();
          mBold = false;
        } else if (code == "1")  // bold
          mBold = true;
        else if (code.startsWith("3")) {  // foreground color change
          const char c = code.toLocal8Bit().data()[1];
          switch (c) {
            case '1':
              mColor = ansiRed();
              break;
            case '2':
              mColor = ansiGreen();
              break;
            case '3':
              mColor = ansiYellow();
              break;
            case '4':
              mColor = ansiBlue();
              break;
            case '5':
              mColor = ansiMagenta();
              break;
            case '6':
              mColor = ansiCyan();
              break;
            case '7':
              mColor = ansiWhite();
              break;
            case '9':  // 39 - Default text color
            default:
              mColor = ansiBlack();
              break;
          }
        }
      }

      int j = i;
      i = msg.indexOf("\033[", i);
      if (i == -1) {
        const QString remains(msg.mid(j));
        html += htmlSpan(remains, level);
        handleCRAndLF(html);
        return;
      }
      if (j != i)
        html += htmlSpan(msg.mid(j, i - j), level);
    }
    handleCRAndLF(html);
  } else
    handleCRAndLF(htmlSpan(msg, level));
}

void WbConsole::appendLog(WbLog::Level level, const QString &message, const QString &prefix, bool popup) {
  mPrefix = prefix;
  appendLog(level, message, popup);
}

void WbConsole::appendLog(WbLog::Level level, const QString &message, bool popup) {
  if (message.isEmpty())
    return;

  switch (level) {
    case WbLog::INFO:
    case WbLog::DEBUG:
      if (gStdoutTee) {
        std::cout << message.toUtf8().constData() << "\n";
        std::cout.flush();
      } else {
        handlePossibleAnsiEscapeSequences(message, level);
        if (popup)
          WbMessageBox::info(message, this);
      }
      break;
    case WbLog::WARNING:
    case WbLog::ERROR:
      if (gStderrTee)
        std::cerr << message.toUtf8().constData() << "\n";
      else {
        handlePossibleAnsiEscapeSequences(message, level);
        if (popup)
          WbMessageBox::warning(message, this);
      }
      break;
    case WbLog::STDOUT:
      if (gStdoutTee) {
        std::cout << message.toUtf8().constData();
        std::cout.flush();
      } else
        handlePossibleAnsiEscapeSequences(message, level);
      break;
    case WbLog::STDERR:
      if (gStderrTee)
        std::cerr << message.toUtf8().constData();
      else
        handlePossibleAnsiEscapeSequences(message, level);
      break;
    case WbLog::FATAL:
      if (gStderrTee)
        std::cerr << message.toUtf8().constData();
      handlePossibleAnsiEscapeSequences(message, level);
      if (popup)
        WbMessageBox::critical(message, this);
      break;
    default:
      break;
  }

  mPrefix = "";
}

QRegExp **WbConsole::createErrorMatchingPatterns() const {
  static QRegExp *exps[] = {
    // gcc: "e-puck.c:7:20: error: stdio.h : No such file or directory"
    // gcc: "main.cc:7: error: 'WbMainWin' was not declared in this scope"
    new QRegExp("(.+\\.\\w+):(\\d+):(\\d+):.*(?:\\w+):.*"), new QRegExp("(.+\\.\\w+):(\\d+):.*(?:\\w+):.*"),

    // javac: "Slave.java:35: illegal start of expression"
    new QRegExp("(.*\\.java):(\\d+): .*"),

    // jvm: "[Driver]   at Driver.run(Driver.java:48)"
    new QRegExp(".*at \\w+\\.\\w+\\((\\w+\\.java):(\\d+)\\)"),

    // Python: "  File "/nao_python/nao_python.py", line 304, in printFootSensors"
    new QRegExp(".*File \"(.+\\.py)\", line (\\d+).*"),

    // Matlab: "Error in ==> /my_nice_file.m at 80"
    // Matlab: "[Rat] Error: File: /rat_controller_matlab.m Line: 134 Column: 20"
    new QRegExp(".*Error in ==> (.+\\.m) at (\\d+)"), new QRegExp(".*Error: File: (.+\\.m) Line: (\\d+) Column: (\\d+)"),

    // Webots parser: "ERROR: '/home/yvan/develop/webots/resources/projects/default/worlds/empty.wbt':19:2: error: skipped
    // unknown 'blabla' field in PointLight node"
    new QRegExp("ERROR: \'(.+\\.(?:wbt|wbo|proto|wrl))\':(\\d+):(\\d+): .*"),
    new QRegExp("ERROR: \'(.+\\.(?:wbt|wbo|proto|wrl))\':(\\d+): .*"),
    new QRegExp("ERROR: \'(.+\\.(?:wbt|wbo|proto|wrl))\': .*"),

    // terminate list
    NULL};

  return exps;
}

void WbConsole::jumpToError(const QString &errorLine) {
  WbBuildEditor *const editor = WbBuildEditor::instance();
  if (!editor)
    return;
  for (int i = 0; mErrorPatterns[i]; ++i) {
    const QRegExp *const exp = mErrorPatterns[i];
    if (exp->exactMatch(errorLine)) {
      const QString fileName(exp->cap(1));  // first parentheses in regexp

      int line = -1;
      if (exp->captureCount() > 1)
        line = exp->cap(2).toInt();  // second parentheses in regexp

      int column = -1;
      if (exp->captureCount() > 2)
        column = exp->cap(3).toInt();  // third parentheses in regexp

      // qDebug() << "WbConsole::jumpToError(): " << fileName << " " << line << " " << column;
      editor->jumpToError(fileName, line - 1, column - 1);
      return;
    }
  }

  editor->unmarkError();
}

void WbConsole::updateFont() {
  // use the font of the preferences
  const WbPreferences *const prefs = WbPreferences::instance();
  QFont font;
  font.fromString(prefs->value("Editor/font").toString());
  mEditor->setFont(font);
}

void WbConsole::handleUserCommand(WbActionManager::WbActionKind actionKind) {
  switch (actionKind) {
    case WbActionManager::COPY:
      mEditor->copy();
      break;
    case WbActionManager::SELECT_ALL:
      mEditor->selectAll();
      break;
    case WbActionManager::FIND:
      openFindDialog();
      break;
    case WbActionManager::FIND_NEXT:
      if (mFindDialog != NULL)
        mFindDialog->next();
      else
        WbFindReplaceDialog::findNext(mTextFind, this);
      break;
    case WbActionManager::FIND_PREVIOUS:
      if (mFindDialog != NULL)
        mFindDialog->previous();
      else
        WbFindReplaceDialog::findPrevious(mTextFind, this);
      break;
    default:
      break;
  }
}

void WbConsole::enableCopyAction(bool enabled) {
  WbActionManager::instance()->setEnabled(WbActionManager::COPY, enabled);
}

void WbConsole::openFindDialog() {
  bool isNew = false;
  if (mFindDialog == NULL) {
    mFindDialog = new WbFindReplaceDialog(mTextFind, false, tr("Console"), this);
    connect(mFindDialog, &WbFindReplaceDialog::finished, this, &WbConsole::deleteFindDialog);
    isNew = true;
  }

  QTextCursor cur = mEditor->textCursor();
  if (cur.hasSelection() && cur.block() == mEditor->document()->findBlock(cur.anchor())) {
    QString selectedText = cur.selectedText();
    if (!selectedText.isEmpty())
      mFindDialog->setFindString(cur.selectedText());
  }

  mFindDialog->show();
  mFindDialog->raise();
  mFindDialog->activateWindow();
  if (isNew)
    mFindDialog->move(mFindDialog->pos() - QPoint(100, -100));
}

void WbConsole::deleteFindDialog() {
  // WbFindReplaceDialog deletes automatically on close
  mFindDialog = NULL;
}
