// Copyright 1996-2020 Cyberbotics Ltd.
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
#include "WbMultiSelectionDialog.hpp"
#include "WbPreferences.hpp"
#include "WbRobot.hpp"
#include "WbSyntaxHighlighter.hpp"
#include "WbTextFind.hpp"
#include "WbWorld.hpp"

#include <QtGui/QTextBlock>
#include <QtGui/QTextDocumentFragment>

#include <QtWidgets/QAction>
#include <QtWidgets/QDialog>
#include <QtWidgets/QLayout>
#include <QtWidgets/QMenu>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStyle>

#include <cassert>
#include <iostream>

#include <ode/ode.h>  // for message handlers

ConsoleEdit::ConsoleEdit(QWidget *parent) : QPlainTextEdit(parent) {
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

ConsoleEdit::~ConsoleEdit() {
  delete mSyntaxHighlighter;
}

void ConsoleEdit::copy() {
  if (textCursor().hasSelection())
    WbClipboard::instance()->setString(textCursor().selection().toPlainText());
}

void ConsoleEdit::mouseDoubleClickEvent(QMouseEvent *event) {
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

void ConsoleEdit::updateSearchTextHighlighting(QRegExp regExp) {
  if (regExp.isEmpty())
    disconnect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting);

  mSyntaxHighlighter->setSearchTextRule(regExp);

  if (!regExp.isEmpty())
    connect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting, Qt::UniqueConnection);
}

void ConsoleEdit::keyPressEvent(QKeyEvent *event) {
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

void ConsoleEdit::focusInEvent(QFocusEvent *event) {
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

void ConsoleEdit::focusOutEvent(QFocusEvent *event) {
  if (WbActionManager::instance()->focusObject() == this)
    WbActionManager::instance()->setFocusObject(NULL);
}

void ConsoleEdit::handleFilterChange() {
  QAction *action = dynamic_cast<QAction *>(sender());
  assert(action);

  if (action->isChecked()) {
    if (action->text() == WbLog::filterName(WbLog::ALL)) {
      // get filters menu
      QMenu *filterMenu = dynamic_cast<QMenu *>(action->parent());
      assert(filterMenu);
      // for each submenu
      for (int i = 0; i < filterMenu->actions().size(); ++i) {
        if (filterMenu->actions()[i]->menu()) {
          const QList<QAction *> actions = filterMenu->actions()[i]->menu()->actions();
          // for each action of the submenu
          for (int j = 0; j < actions.size(); ++j) {
            if (actions[j]->isChecked() && actions[j] != action)
              emit filterDisabled(actions[j]->text());
          }
        }
      }
    } else if (action->text() == WbLog::filterName(WbLog::ALL_WEBOTS) ||
               action->text() == WbLog::filterName(WbLog::ALL_CONTROLLERS)) {
      // get parent menu
      QMenu *menu = dynamic_cast<QMenu *>(action->parent());
      assert(menu);
      const QList<QAction *> actions = menu->actions();
      // for each action of the menu
      for (int j = 0; j < actions.size(); ++j) {
        if (actions[j]->isChecked() && actions[j] != action)
          emit filterDisabled(actions[j]->text());
      }
      // disable global 'All' action
      emit filterDisabled(WbLog::filterName(WbLog::ALL));
    } else {
      // disable global 'All' action
      emit filterDisabled(WbLog::filterName(WbLog::ALL));
      // disable local 'All' action
      QMenu *menu = dynamic_cast<QMenu *>(action->parent());
      assert(menu);
      const QList<QAction *> actions = menu->actions();
      // for each action of the menu
      for (int j = 0; j < actions.size(); ++j) {
        if (actions[j]->isChecked() && (actions[j]->text() == WbLog::filterName(WbLog::ALL_WEBOTS) ||
                                        actions[j]->text() == WbLog::filterName(WbLog::ALL_CONTROLLERS)))
          emit filterDisabled(actions[j]->text());
      }
    }
  }

  if (action->isChecked())
    emit filterEnabled(action->text());
  else
    emit filterDisabled(action->text());
}

void ConsoleEdit::addContextMenuFilterItem(const QString &name, QMenu *menu, const QString &toolTip) {
  WbConsole *console = dynamic_cast<WbConsole *>(parentWidget());
  assert(console);
  QAction *action = new QAction(menu);
  action->setText(name);
  if (!toolTip.isEmpty())
    action->setToolTip(toolTip);
  action->setCheckable(true);
  action->setChecked(console->getEnabledLogs().contains(name));
  menu->addAction(action);
  connect(action, &QAction::toggled, this, &ConsoleEdit::handleFilterChange);
}

void ConsoleEdit::showCustomContextMenu(const QPoint &pt) {
  QMenu *menu = createStandardContextMenu();
  menu->addAction(WbActionManager::instance()->action(WbActionManager::FIND));
  menu->addSeparator();
  QMenu *subMenu = menu->addMenu(tr("&Filters"));
  addContextMenuFilterItem(WbLog::filterName(WbLog::ALL), subMenu, tr("Display all the logs."));
  QMenu *webotsSubMenu = subMenu->addMenu(WbLog::filterName(WbLog::WEBOTS));
  addContextMenuFilterItem(WbLog::filterName(WbLog::ALL_WEBOTS), webotsSubMenu, tr("Display all the messages from Webots."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::ODE), webotsSubMenu, tr("Display error messages from ODE."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::JAVASCRIPT), webotsSubMenu,
                           tr("Display Javascript log from the robot-windows."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::WEBOTS_OTHERS), webotsSubMenu, tr("Display all the other logs."));
  QMenu *controllerSubMenu = subMenu->addMenu(WbLog::filterName(WbLog::CONTROLLERS));
  addContextMenuFilterItem(WbLog::filterName(WbLog::ALL_CONTROLLERS), controllerSubMenu,
                           tr("Display all the messages from the controller(s)."));
  const WbWorld *world = WbWorld::instance();
  if (world) {
    foreach (const WbRobot *robot, world->robots())
      addContextMenuFilterItem(robot->name(), controllerSubMenu,
                               tr("Display output from the controller of the '%1' controller.").arg(robot->name()));
  }
  menu->addSeparator();
  QAction *clearAction = new QAction(this);
  clearAction->setText(tr("Clear Console"));
  connect(clearAction, &QAction::triggered, this, &ConsoleEdit::clear);
  menu->addAction(clearAction);
  menu->addAction(WbActionManager::instance()->action(WbActionManager::CLEAR_CONSOLE));
  menu->addAction(WbActionManager::instance()->action(WbActionManager::NEW_CONSOLE));
  menu->exec(mapToGlobal(pt));

  QList<QAction *> actions = webotsSubMenu->actions();
  actions += controllerSubMenu->actions();
  actions += subMenu->actions();
  for (int i = 0; i < actions.size(); ++i)
    delete actions[i];
  menu->removeAction(clearAction);
  delete clearAction;
  delete menu;
}

static bool gStdoutTee = false;
static bool gStderrTee = false;

void WbConsole::enableStdOutRedirectToTerminal() {
  gStdoutTee = true;
};

void WbConsole::enableStdErrRedirectToTerminal() {
  gStderrTee = true;
};

namespace {
  void odeErrorFunc(int errnum, const char *msg, va_list ap) {
    const QString error = QString::vasprintf(msg, ap);
    emit WbLog::instance()->logEmitted(WbLog::ERROR, QString("ODE Error %1: ").arg(errnum) + error, false,
                                       WbLog::filterName(WbLog::ODE));
  }

  void odeDebugFunc(int errnum, const char *msg, va_list ap) {
    const QString debug = QString::vasprintf(msg, ap);
    emit WbLog::instance()->logEmitted(WbLog::DEBUG, QString("ODE INTERNAL ERROR %1: ").arg(errnum) + debug, false,
                                       WbLog::filterName(WbLog::ODE));
  }

  void odeMessageFunc(int errnum, const char *msg, va_list ap) {
    QString message = QString::vasprintf(msg, ap);
    if (message.startsWith("LCP")) {
      message = QString("The current physics step could not be computed correctly. "
                        "Your world may be too complex. If this problem persists, try simplifying "
                        "your bounding object(s), reducing the number of joints, or reducing "
                        "WorldInfo.basicTimeStep.");

      emit WbLog::instance()->logEmitted(WbLog::WARNING, QString("WARNING: ") + message, false, WbLog::filterName(WbLog::ODE));
    } else
      emit WbLog::instance()->logEmitted(WbLog::WARNING, QString("ODE Message %1: ").arg(errnum) + message, false,
                                         WbLog::filterName(WbLog::ODE));
  }
}  // namespace

WbConsole::WbConsole(QWidget *parent, const QString &name) :
  WbDockWidget(parent),
  mEnabledLogs(WbLog::filterName(WbLog::ALL)),
  mEditor(new ConsoleEdit(this)),
  mErrorPatterns(createErrorMatchingPatterns()),  // patterns for error matching
  mBold(false),
  mUnderline(false),
  mIsOverwriteEnabled(false),  // option to overwrite last line
  mFindDialog(NULL),
  mTextFind(new WbTextFind(mEditor)) {
  setTabbedTitle(name);
  setObjectName(name);
  updateTitle();

  titleBarWidget()->setObjectName("consoleTitleBar");
  titleBarWidget()->style()->polish(titleBarWidget());

  mFiltersButton = new QPushButton(this);
  QIcon icon = QIcon();
  icon.addFile("enabledIcons:filters.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:filters.png", QSize(), QIcon::Disabled);
  mFiltersButton->setIcon(icon);
  titleBarWidget()->layout()->addWidget(mFiltersButton);
  connect(mFiltersButton, &QPushButton::pressed, this, &WbConsole::selectFilters);

  // create text editor
  mEditor->setReadOnly(true);
  mEditor->setMaximumBlockCount(5000);  // limit the memory usage
  mEditor->setFocusPolicy(Qt::ClickFocus);
  setWidget(mEditor);

  connect(mEditor, &ConsoleEdit::filterEnabled, this, &WbConsole::enableFilter);
  connect(mEditor, &ConsoleEdit::filterDisabled, this, &WbConsole::disableFilter);

  connect(mEditor, &ConsoleEdit::copyAvailable, this, &WbConsole::enableCopyAction);
  connect(WbActionManager::instance(), &WbActionManager::userConsoleEditCommandReceived, this, &WbConsole::handleUserCommand);

  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbConsole::updateFont);
  updateFont();

  connect(WbActionManager::instance()->action(WbActionManager::CLEAR_CONSOLE), &QAction::triggered, this, &WbConsole::clear);

  connect(mTextFind, &WbTextFind::findStringChanged, mEditor, &ConsoleEdit::updateSearchTextHighlighting);

  // listen to WbLog
  connect(WbLog::instance(), SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &)), this,
          SLOT(appendLog(WbLog::Level, const QString &, bool, const QString &)));
  connect(WbLog::instance(), SIGNAL(cleared()), this, SLOT(clear()));

  // Install ODE message handlers
  dSetErrorHandler(odeErrorFunc);
  dSetDebugHandler(odeDebugFunc);
  dSetMessageHandler(odeMessageFunc);
}

void WbConsole::setEnabledLogs(const QStringList &logs) {
  mEnabledLogs = logs;
  updateTitle();
}

void WbConsole::clear(bool reset) {
  mEditor->clear();
  if (reset)
    resetFormat();
}

void WbConsole::resetFormat() {
  mForegroundColor.clear();
  mBackgroundColor.clear();
  mBold = false;
  mUnderline = false;
}

QString WbConsole::htmlSpan(const QString &s, WbLog::Level level) const {
  if (s.isEmpty() || s == "\n")
    return "";

  bool bold;
  QString foregroundColor;
  if (level == WbLog::ERROR || level == WbLog::FATAL || level == WbLog::STDERR) {
    foregroundColor = errorColor();
    bold = true;
  } else if (level == WbLog::WARNING || level == WbLog::DEBUG) {
    foregroundColor = errorColor();
    bold = false;
  } else if (level == WbLog::INFO || level == WbLog::STATUS) {
    foregroundColor = infoColor();
    bold = false;
  } else {
    assert(level == WbLog::STDOUT);
    foregroundColor = mForegroundColor;
    bold = mBold;
  }

  QString span("<span");
  if (!foregroundColor.isEmpty() || !mBackgroundColor.isEmpty() || bold || mUnderline) {
    span += " style=\"";
    if (!foregroundColor.isEmpty())
      span += "color:" + foregroundColor + ";";
    if (!mBackgroundColor.isEmpty())
      span += "background-color:" + mBackgroundColor + ";";
    if (bold)
      span += "font-weight:bold;";
    if (mUnderline)
      span += "text-decoration:underline;";
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
    if (i != 0)  // escape code is not at the beginning of the string
      html = htmlSpan(msg.mid(0, i), level);
    while (1) {
      QString sequence;
      int msgLength = msg.length();
      if (msg.at(i) == '\x1b') {
        i += 2;  // skip the "\033[" chars
        int start = i;
        while (i < msgLength && msg.at(i++) < '\x40')
          ;
        sequence += msg.mid(start, i - start);
      }

      const QStringList codes(sequence.split(";"));  // handle multiple (e.g. sequence "ESC[0;39m" )
      foreach (const QString code, codes) {
        // the stored sequence may be "0m" or "1m", "4m", "2J", "30m", "31m", "32m", etc.
        if (code == "0m")  // reset to default
          resetFormat();
        else if (code == "1m")  // bold
          mBold = true;
        else if (code == "4m")  // underlined
          mUnderline = true;
        else if (code.startsWith("2")) {  // clear console screen
          const char c = code.toLocal8Bit().data()[1];
          if (c == 'J') {             // code == "2J"
            WbConsole::clear(false);  // perform a clear by preserving format
            html.clear();             // nothing to output since clear has been done
          }
        } else if (code.startsWith("3")) {  // foreground color change
          const char c = code.toLocal8Bit().data()[1];
          switch (c) {
            case '0':  // code == 30m
              mForegroundColor = ansiBlack();
              break;
            case '1':  // code == 31m
              mForegroundColor = ansiRed();
              break;
            case '2':  // code == 32m
              mForegroundColor = ansiGreen();
              break;
            case '3':  // etc...
              mForegroundColor = ansiYellow();
              break;
            case '4':
              mForegroundColor = ansiBlue();
              break;
            case '5':
              mForegroundColor = ansiMagenta();
              break;
            case '6':
              mForegroundColor = ansiCyan();
              break;
            case '7':
              mForegroundColor = ansiWhite();
              break;
            case '9':  // 39m - Default text color
              mForegroundColor.clear();
              break;
            default:
              break;
          }
        } else if (code.startsWith("4")) {  // background color change
          const char c = code.toLocal8Bit().data()[1];
          switch (c) {
            case '0':  // code == 40m
              mBackgroundColor = ansiBlack();
              break;
            case '1':  // code == 41m
              mBackgroundColor = ansiRed();
              break;
            case '2':  // code == 42m
              mBackgroundColor = ansiGreen();
              break;
            case '3':  // etc...
              mBackgroundColor = ansiYellow();
              break;
            case '4':
              mBackgroundColor = ansiBlue();
              break;
            case '5':
              mBackgroundColor = ansiMagenta();
              break;
            case '6':
              mBackgroundColor = ansiCyan();
              break;
            case '7':
              mBackgroundColor = ansiWhite();
              break;
            case '9':  // 49m - Default background color
              mBackgroundColor.clear();
              break;
            default:
              break;
          }
        }
      }
      int j = i;
      i = msg.indexOf("\033[", i);
      if (i == -1) {  // Previous escape code was the last one found
        const QString remains(msg.mid(j));
        html += htmlSpan(remains, level);
        handleCRAndLF(html);
        return;
      }
      if (j != i)  // Extract text contained between two escape codes if so
        html += htmlSpan(msg.mid(j, i - j), level);
    }
  }
  handleCRAndLF(htmlSpan(msg, level));
}

void WbConsole::appendLog(WbLog::Level level, const QString &message, bool popup, const QString &logName) {
  if (message.isEmpty())
    return;

  if (!mEnabledLogs.contains(WbLog::filterName(WbLog::ALL))) {
    if (logName.isEmpty()) {
      // WEBOTS_OTHERS
      if (!mEnabledLogs.contains(WbLog::filterName(WbLog::WEBOTS_OTHERS)) &&
          !mEnabledLogs.contains(WbLog::filterName(WbLog::ALL_WEBOTS)))
        return;
    } else if (!mEnabledLogs.contains(logName)) {
      if (logName == WbLog::filterName(WbLog::ODE) || logName == WbLog::filterName(WbLog::JAVASCRIPT)) {
        if (!mEnabledLogs.contains(WbLog::filterName(WbLog::ALL_WEBOTS)))
          return;
      } else if (!mEnabledLogs.contains(WbLog::filterName(WbLog::ALL_CONTROLLERS)))
        return;
    }
  }

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

void WbConsole::updateTitle() {
  QString title("Console - ");
  title += mEnabledLogs.join(" | ");
  setWindowTitle(title);
}

void WbConsole::closeEvent(QCloseEvent *event) {
  WbDockWidget::closeEvent(event);
  emit closed();
}

void WbConsole::selectFilters() {
  QStringList options;
  options << WbLog::filterName(WbLog::ALL) << WbLog::filterName(WbLog::WEBOTS_OTHERS) << WbLog::filterName(WbLog::ODE)
          << WbLog::filterName(WbLog::JAVASCRIPT);
  const WbWorld *world = WbWorld::instance();
  if (world) {
    foreach (const WbRobot *robot, world->robots())
      options << robot->name();
  }
  WbMultiSelectionDialog dialog(tr("Select what to display in this console:"), options, mEnabledLogs, this);
  dialog.setWindowTitle("Console Filters");
  const int result = dialog.exec();
  if (result == QDialog::Accepted) {
    mEnabledLogs = dialog.enabledOptions();
    updateTitle();
  }
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

void WbConsole::enableFilter(const QString &filter) {
  assert(!mEnabledLogs.contains(filter));
  mEnabledLogs.append(filter);
  updateTitle();
}

void WbConsole::disableFilter(const QString &filter) {
  mEnabledLogs.removeAll(filter);
  updateTitle();
}
