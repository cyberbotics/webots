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

#include "WbConsole.hpp"

#include "WbActionManager.hpp"
#include "WbBuildEditor.hpp"
#include "WbClipboard.hpp"
#include "WbDockTitleBar.hpp"
#include "WbFindReplaceDialog.hpp"
#include "WbLog.hpp"
#include "WbMessageBox.hpp"
#include "WbPreferences.hpp"
#include "WbRobot.hpp"
#include "WbSyntaxHighlighter.hpp"
#include "WbTextFind.hpp"
#include "WbWorld.hpp"

#include <QtGui/QAction>
#include <QtGui/QTextBlock>
#include <QtGui/QTextDocumentFragment>

#include <QtWidgets/QInputDialog>
#include <QtWidgets/QLayout>
#include <QtWidgets/QMenu>
#include <QtWidgets/QStyle>

#include <cassert>

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
  addAction(WbActionManager::instance()->action(WbAction::CLEAR_CONSOLE));
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

void ConsoleEdit::updateSearchTextHighlighting(QRegularExpression regularExpression) {
  if (regularExpression.pattern().isEmpty())
    disconnect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting);

  mSyntaxHighlighter->setSearchTextRule(regularExpression);

  if (!regularExpression.pattern().isEmpty())
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
  actionManager->enableTextEditActions(false, true);
  actionManager->setEnabled(WbAction::COPY, textCursor().hasSelection());
  actionManager->setEnabled(WbAction::SELECT_ALL, true);
  actionManager->setEnabled(WbAction::FIND, true);
  actionManager->setEnabled(WbAction::FIND_NEXT, true);
  actionManager->setEnabled(WbAction::FIND_PREVIOUS, true);
  actionManager->setEnabled(WbAction::CUT, false);
  actionManager->setEnabled(WbAction::PASTE, false);
  actionManager->setEnabled(WbAction::UNDO, false);
  actionManager->setEnabled(WbAction::REDO, false);
}

void ConsoleEdit::focusOutEvent(QFocusEvent *event) {
  if (WbActionManager::instance()->focusObject() == this)
    WbActionManager::instance()->setFocusObject(NULL);
}

void ConsoleEdit::handleFilterChange() {
  QAction *action = dynamic_cast<QAction *>(sender());
  assert(action);

  // disable conflicting filters
  if (action->isChecked()) {
    if (action->text() == WbLog::filterName(WbLog::ALL)) {
      // disable all the specific filters
      QMenu *menu = dynamic_cast<QMenu *>(action->parent());
      assert(menu);
      const QList<QAction *> actions = menu->actions();
      // for each action of the menu
      for (int i = 0; i < actions.size(); ++i) {
        if (actions[i]->isChecked() && actions[i] != action)
          emit filterDisabled(actions[i]->text());
      }
    } else if (action->text() == WbLog::filterName(WbLog::ALL_WEBOTS)) {
      // disable all the Webots filters
      foreach (const QString filter, WbLog::webotsFilterNames())
        emit filterDisabled(filter);
      emit filterDisabled(WbLog::filterName(WbLog::ALL));
    } else if (action->text() == WbLog::filterName(WbLog::ALL_CONTROLLERS)) {
      // disable all the controller filters
      QMenu *menu = dynamic_cast<QMenu *>(action->parent());
      assert(menu);
      const QList<QAction *> actions = menu->actions();
      // for each action of the menu
      for (int i = 0; i < actions.size(); ++i) {
        if (actions[i]->isChecked() && actions[i]->property("isControllerAction").isValid())
          emit filterDisabled(actions[i]->text());
      }
      emit filterDisabled(WbLog::filterName(WbLog::ALL));
    } else {
      emit filterDisabled(WbLog::filterName(WbLog::ALL));
      if (action->property("isControllerAction").isValid())
        emit filterDisabled(WbLog::filterName(WbLog::ALL_CONTROLLERS));
      else
        emit filterDisabled(WbLog::filterName(WbLog::ALL_WEBOTS));
    }
  }

  // perform the update
  if (action->isChecked())
    emit filterEnabled(action->text());
  else
    emit filterDisabled(action->text());
}

void ConsoleEdit::handleLevelChange() {
  QAction *action = dynamic_cast<QAction *>(sender());
  assert(action);

  // disable conflicting levels
  if (action->isChecked()) {
    if (action->text() == WbLog::filterName(WbLog::ALL)) {
      // disable all the specific levels
      QMenu *menu = dynamic_cast<QMenu *>(action->parent());
      assert(menu);
      const QList<QAction *> actions = menu->actions();
      // for each action of the menu
      for (int i = 0; i < actions.size(); ++i) {
        if (actions[i]->isChecked() && actions[i] != action)
          emit levelDisabled(actions[i]->text());
      }
    } else if (action->text() == WbLog::filterName(WbLog::ALL_WEBOTS)) {
      emit levelDisabled(WbLog::levelName(WbLog::INFO));
      emit levelDisabled(WbLog::levelName(WbLog::WARNING));
      emit levelDisabled(WbLog::levelName(WbLog::ERROR));
      emit levelDisabled(WbLog::filterName(WbLog::ALL));
    } else if (action->text() == WbLog::filterName(WbLog::ALL_CONTROLLERS)) {
      emit levelDisabled(WbLog::levelName(WbLog::STDOUT));
      emit levelDisabled(WbLog::levelName(WbLog::STDERR));
      emit levelDisabled(WbLog::filterName(WbLog::ALL));
    } else {
      emit levelDisabled(WbLog::filterName(WbLog::ALL));
      if (action->text() == WbLog::levelName(WbLog::STDOUT) || action->text() == WbLog::levelName(WbLog::STDERR))
        emit levelDisabled(WbLog::filterName(WbLog::ALL_CONTROLLERS));
      else
        emit levelDisabled(WbLog::filterName(WbLog::ALL_WEBOTS));
    }
  }

  // perform the update
  if (action->isChecked())
    emit levelEnabled(action->text());
  else
    emit levelDisabled(action->text());
}

void ConsoleEdit::addContextMenuFilterItem(const QString &name, QMenu *menu, const QString &toolTip, bool isControllerAction) {
  WbConsole *console = dynamic_cast<WbConsole *>(parentWidget());
  assert(console);
  QAction *action = new QAction(menu);
  action->setText(name);
  if (!toolTip.isEmpty())
    action->setToolTip(toolTip);
  if (isControllerAction)
    action->setProperty("isControllerAction", QVariant(true));
  action->setCheckable(true);
  action->setChecked(console->getEnabledFilters().contains(name));
  menu->addAction(action);
  connect(action, &QAction::toggled, this, &ConsoleEdit::handleFilterChange);
}

void ConsoleEdit::addContextMenuLevelItem(const QString &name, QMenu *menu, const QString &toolTip) {
  WbConsole *console = dynamic_cast<WbConsole *>(parentWidget());
  assert(console);
  QAction *action = new QAction(menu);
  action->setText(name);
  if (!toolTip.isEmpty())
    action->setToolTip(toolTip);
  action->setCheckable(true);
  action->setChecked(console->getEnabledLevels().contains(name));
  menu->addAction(action);
  connect(action, &QAction::toggled, this, &ConsoleEdit::handleLevelChange);
}

void ConsoleEdit::showCustomContextMenu(const QPoint &pt) {
  WbConsole *console = dynamic_cast<WbConsole *>(parentWidget());
  assert(console);

  QMenu *menu = createStandardContextMenu();
  menu->addAction(WbActionManager::instance()->action(WbAction::FIND));
  menu->addSeparator();

  // filters
  QMenu *filterMenu = menu->addMenu(tr("&Filter"));
  addContextMenuFilterItem(WbLog::filterName(WbLog::ALL), filterMenu, tr("Display all the logs."));
  filterMenu->addSeparator();
  addContextMenuFilterItem(WbLog::filterName(WbLog::ALL_WEBOTS), filterMenu, tr("Display all the messages from Webots."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::PARSING), filterMenu,
                           tr("Display parsing error when editing or loading a world."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::ODE), filterMenu, tr("Display error messages from ODE."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::PHYSICS_PLUGINS), filterMenu,
                           tr("Display messages from the physics plugins."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::JAVASCRIPT), filterMenu,
                           tr("Display Javascript log from the robot-windows."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::COMPILATION), filterMenu, tr("Output from the compilation."));
  addContextMenuFilterItem(WbLog::filterName(WbLog::WEBOTS_OTHERS), filterMenu, tr("Display all the other logs."));
  filterMenu->addSeparator();
  addContextMenuFilterItem(WbLog::filterName(WbLog::ALL_CONTROLLERS), filterMenu,
                           tr("Display all the messages from the controller(s)."));
  const WbWorld *world = WbWorld::instance();
  if (world) {
    foreach (const WbRobot *robot, world->robots())
      addContextMenuFilterItem(robot->name(), filterMenu,
                               tr("Display output from the controller of the '%1' controller.").arg(robot->name()), true);
  }

  // levels
  QMenu *levelMenu = menu->addMenu(tr("&Level"));
  addContextMenuLevelItem(WbLog::filterName(WbLog::ALL), levelMenu, tr("Display all the logs."));
  levelMenu->addSeparator();
  addContextMenuLevelItem(WbLog::filterName(WbLog::ALL_WEBOTS), levelMenu, tr("Display all the Webots logs."));
  addContextMenuLevelItem(WbLog::levelName(WbLog::ERROR), levelMenu, tr("Displays Webots errors and controller(s) stderr."));
  addContextMenuLevelItem(WbLog::levelName(WbLog::WARNING), levelMenu, tr("Displays Webots warnings."));
  addContextMenuLevelItem(WbLog::levelName(WbLog::INFO), levelMenu, tr("Displays Webots info."));
  levelMenu->addSeparator();
  addContextMenuLevelItem(WbLog::filterName(WbLog::ALL_CONTROLLERS), levelMenu, tr("Display controller(s) stdout and stderr."));
  addContextMenuLevelItem(WbLog::levelName(WbLog::STDOUT), levelMenu, tr("Display controller(s) stdout."));
  addContextMenuLevelItem(WbLog::levelName(WbLog::STDERR), levelMenu, tr("Display controller(s) stderr."));
  menu->addSeparator();

  // actions
  QAction *renameAction = new QAction(this);
  renameAction->setText(tr("Rename Console"));
  connect(renameAction, &QAction::triggered, console, &WbConsole::rename);
  QAction *clearAction = new QAction(this);
  clearAction->setText(tr("Clear Console"));
  connect(clearAction, &QAction::triggered, this, &ConsoleEdit::clear);
  menu->addAction(renameAction);
  menu->addAction(clearAction);
  menu->addAction(WbActionManager::instance()->action(WbAction::CLEAR_CONSOLE));
  menu->addAction(WbActionManager::instance()->action(WbAction::NEW_CONSOLE));

  // execution
  menu->exec(mapToGlobal(pt));

  // cleanup
  const QList<QAction *> actions = filterMenu->actions() + levelMenu->actions();
  for (int i = 0; i < actions.size(); ++i)
    delete actions[i];
  menu->removeAction(renameAction);
  menu->removeAction(clearAction);
  delete renameAction;
  delete clearAction;
  delete menu;
}

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
  mEnabledFilters(WbLog::filterName(WbLog::ALL)),
  mEnabledLevels(WbLog::filterName(WbLog::ALL)),
  mEditor(new ConsoleEdit(this)),
  mErrorPatterns(createErrorMatchingPatterns()),  // patterns for error matching
  mConsoleName(name),
  mBold(false),
  mUnderline(false),
  mIsOverwriteEnabled(false),  // option to overwrite last line
  mFindDialog(NULL),
  mTextFind(new WbTextFind(mEditor)) {
  updateTitle();

  titleBarWidget()->setObjectName("consoleTitleBar");
  titleBarWidget()->style()->polish(titleBarWidget());

  // create text editor
  mEditor->setReadOnly(true);
  mEditor->setMaximumBlockCount(5000);  // limit the memory usage
  mEditor->setFocusPolicy(Qt::ClickFocus);
  setWidget(mEditor);

  connect(mEditor, &ConsoleEdit::filterEnabled, this, &WbConsole::enableFilter);
  connect(mEditor, &ConsoleEdit::filterDisabled, this, &WbConsole::disableFilter);
  connect(mEditor, &ConsoleEdit::levelEnabled, this, &WbConsole::enableLevel);
  connect(mEditor, &ConsoleEdit::levelDisabled, this, &WbConsole::disableLevel);

  connect(mEditor, &ConsoleEdit::copyAvailable, this, &WbConsole::enableCopyAction);
  connect(WbActionManager::instance(), &WbActionManager::userConsoleEditCommandReceived, this, &WbConsole::handleUserCommand);

  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbConsole::updateFont);
  updateFont();

  connect(WbActionManager::instance()->action(WbAction::CLEAR_CONSOLE), &QAction::triggered, this, &WbConsole::clear);

  connect(mTextFind, &WbTextFind::findStringChanged, mEditor, &ConsoleEdit::updateSearchTextHighlighting);

  // listen to WbLog
  connect(WbLog::instance(), SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &)), this,
          SLOT(appendLog(WbLog::Level, const QString &, bool, const QString &)));

  // Install ODE message handlers
  dSetErrorHandler(odeErrorFunc);
  dSetDebugHandler(odeDebugFunc);
  dSetMessageHandler(odeMessageFunc);
}

void WbConsole::setEnabledFilters(const QStringList &filters) {
  mEnabledFilters = filters;
  updateTitle();
}

void WbConsole::setEnabledLevels(const QStringList &levels) {
  mEnabledLevels = levels;
  updateTitle();
}

void WbConsole::clear(bool reset) {
  mEditor->clear();
  if (reset)
    resetFormat();
}

void WbConsole::rename() {
  bool ok = false;
  const QString nameString =
    QInputDialog::getText(this, tr("Console Name"), tr("New name:"), QLineEdit::Normal, mConsoleName, &ok);
  if (ok && !nameString.isEmpty()) {
    mConsoleName = nameString;
    updateTitle();
  }
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

  assert(!logName.isEmpty() || level == WbLog::STATUS);

  // check enabled filters
  if (!mEnabledFilters.contains(WbLog::filterName(WbLog::ALL)) && !mEnabledFilters.contains(logName)) {
    if (WbLog::webotsFilterNames().contains(logName)) {
      if (!mEnabledFilters.contains(WbLog::filterName(WbLog::ALL_WEBOTS)))
        return;
    } else if (!mEnabledFilters.contains(WbLog::filterName(WbLog::ALL_CONTROLLERS)))
      return;
  }

  // check enabled levels
  if (!mEnabledLevels.contains(WbLog::filterName(WbLog::ALL))) {
    switch (level) {
      case WbLog::DEBUG:
      case WbLog::WARNING:
        if (!mEnabledLevels.contains(WbLog::levelName(WbLog::WARNING)))
          return;
        break;
      case WbLog::STDOUT:
      case WbLog::STDERR:
      case WbLog::INFO:
      case WbLog::ERROR:
        if (!mEnabledLevels.contains(WbLog::levelName(level)))
          return;
        break;
      case WbLog::FATAL:
      default:
        break;
    }
  }

  switch (level) {
    case WbLog::INFO:
    case WbLog::DEBUG:
      handlePossibleAnsiEscapeSequences(message, level);
      if (popup)
        WbMessageBox::info(message, this);
      break;
    case WbLog::WARNING:
    case WbLog::ERROR:
      handlePossibleAnsiEscapeSequences(message, level);
      if (popup)
        WbMessageBox::warning(message, this);
      break;
    case WbLog::STDOUT:
      handlePossibleAnsiEscapeSequences(message, level);
      break;
    case WbLog::STDERR:
      handlePossibleAnsiEscapeSequences(message, level);
      break;
    case WbLog::FATAL:
      handlePossibleAnsiEscapeSequences(message, level);
      if (popup)
        WbMessageBox::critical(message, this);
      break;
    default:
      break;
  }
}

QRegularExpression **WbConsole::createErrorMatchingPatterns() const {
  static QRegularExpression *exps[] = {
    // gcc: "e-puck.c:7:20: error: stdio.h : No such file or directory"
    // gcc: "main.cc:7: error: 'WbMainWin' was not declared in this scope"
    new QRegularExpression("(.+\\.\\w+):(\\d+):(\\d+):.*(?:\\w+):.*"),
    new QRegularExpression("(.+\\.\\w+):(\\d+):.*(?:\\w+):.*"),

    // javac: "Slave.java:35: illegal start of expression"
    new QRegularExpression("(.*\\.java):(\\d+): .*"),

    // jvm: "[Driver]   at Driver.run(Driver.java:48)"
    new QRegularExpression(".*at \\w+\\.\\w+\\((\\w+\\.java):(\\d+)\\)"),

    // Python: "  File "/nao_python/nao_python.py", line 304, in printFootSensors"
    new QRegularExpression(".*File \"(.+\\.py)\", line (\\d+).*"),

    // Matlab: "Error in ==> /my_nice_file.m at 80"
    // Matlab: "[Rat] Error: File: /rat_controller_matlab.m Line: 134 Column: 20"
    new QRegularExpression(".*Error in ==> (.+\\.m) at (\\d+)"),
    new QRegularExpression(".*Error: File: (.+\\.m) Line: (\\d+) Column: (\\d+)"),

    // Webots parser: "ERROR: '/home/yvan/develop/webots/resources/projects/default/worlds/empty.wbt':19:2: error: skipped
    // unknown 'blabla' field in PointLight node"
    new QRegularExpression("ERROR: \'(.+\\.(?:wbt|proto))\':(\\d+):(\\d+): .*"),
    new QRegularExpression("ERROR: \'(.+\\.(?:wbt|proto))\':(\\d+): .*"),
    new QRegularExpression("ERROR: \'(.+\\.(?:wbt|proto))\': .*"),

    // terminate list
    NULL};

  return exps;
}

void WbConsole::jumpToError(const QString &errorLine) {
  WbBuildEditor *const editor = WbBuildEditor::instance();
  if (!editor)
    return;
  for (int i = 0; mErrorPatterns[i]; ++i) {
    const QRegularExpression *const exp = mErrorPatterns[i];
    QRegularExpressionMatch match = exp->match(errorLine);
    if (match.hasMatch()) {
      const QString fileName(match.captured(1));  // first parentheses in regular expression

      int line = -1;
      if (match.lastCapturedIndex() > 1)
        line = match.captured(2).toInt();  // second parentheses in regular expression

      int column = -1;
      if (match.lastCapturedIndex() > 2)
        column = match.captured(3).toInt();  // third parentheses in regular expression

      // qDebug() << "WbConsole::jumpToError(): " << fileName << " " << line << " " << column;
      editor->jumpToError(fileName, line - 1, column - 1);
      return;
    }
  }

  editor->unmarkError();
}

void WbConsole::updateTitle() {
  setObjectName(mConsoleName + mEnabledFilters.join(QString()) + mEnabledLevels.join(QString()));
  QString title(mConsoleName + " - ");
  title += mEnabledFilters.join(" | ");
  if (!mEnabledLevels.contains(WbLog::filterName(WbLog::ALL)))
    title += QString(" - ") + mEnabledLevels.join(" | ");
  setWindowTitle(title);
  if (mEnabledFilters.size() == 1 && mConsoleName == "Console")
    setTabbedTitle(mEnabledFilters.at(0));
  else
    setTabbedTitle(mConsoleName);
}

void WbConsole::closeEvent(QCloseEvent *event) {
  WbDockWidget::closeEvent(event);
  emit closed();
}

void WbConsole::updateFont() {
  // use the font of the preferences
  const WbPreferences *const prefs = WbPreferences::instance();
  QFont font;
  font.fromString(prefs->value("Editor/font").toString());
  mEditor->setFont(font);
}

void WbConsole::handleUserCommand(WbAction::WbActionKind actionKind) {
  switch (actionKind) {
    case WbAction::COPY:
      mEditor->copy();
      break;
    case WbAction::SELECT_ALL:
      mEditor->selectAll();
      break;
    case WbAction::FIND:
      openFindDialog();
      break;
    case WbAction::FIND_NEXT:
      if (mFindDialog != NULL)
        mFindDialog->next();
      else
        WbFindReplaceDialog::findNext(mTextFind, this);
      break;
    case WbAction::FIND_PREVIOUS:
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
  WbActionManager::instance()->setEnabled(WbAction::COPY, enabled);
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
  assert(!mEnabledFilters.contains(filter));
  mEnabledFilters.append(filter);
  updateTitle();
}

void WbConsole::disableFilter(const QString &filter) {
  mEnabledFilters.removeAll(filter);
  updateTitle();
}

void WbConsole::enableLevel(const QString &level) {
  assert(!mEnabledLevels.contains(level));
  mEnabledLevels.append(level);
  updateTitle();
}

void WbConsole::disableLevel(const QString &level) {
  mEnabledLevels.removeAll(level);
  updateTitle();
}
