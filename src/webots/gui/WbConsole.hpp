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

#ifndef WB_CONSOLE_HPP
#define WB_CONSOLE_HPP

//
// Description: Webots console widget
//

#include "WbActionManager.hpp"
#include "WbDockWidget.hpp"
#include "WbLog.hpp"

#include <QtCore/QRegularExpression>
#include <QtWidgets/QPlainTextEdit>

class QAction;
class WbFindReplaceDialog;
class WbTextFind;
class WbSyntaxHighlighter;

class ConsoleEdit : public QPlainTextEdit {
  Q_OBJECT

public:
  explicit ConsoleEdit(QWidget *parent);
  virtual ~ConsoleEdit();
  void copy();
  void mouseDoubleClickEvent(QMouseEvent *event) override;

signals:
  void filterEnabled(const QString &filter);
  void filterDisabled(const QString &filter);
  void levelEnabled(const QString &level);
  void levelDisabled(const QString &level);

public slots:
  void updateSearchTextHighlighting(QRegularExpression regularExpression);

protected:
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override { event->ignore(); }
  void focusInEvent(QFocusEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;

private:
  WbSyntaxHighlighter *mSyntaxHighlighter;

  void addContextMenuFilterItem(const QString &name, QMenu *menu, const QString &toolTip = QString(),
                                bool isControllerAction = false);
  void addContextMenuLevelItem(const QString &name, QMenu *menu, const QString &toolTip = QString());

private slots:
  void showCustomContextMenu(const QPoint &pt);
  void resetSearchTextHighlighting() { updateSearchTextHighlighting(QRegularExpression()); }
  void handleFilterChange();
  void handleLevelChange();
};

// cppcheck-suppress noConstructor
class WbConsole : public WbDockWidget {
  Q_OBJECT
  // WbLog colors
  Q_PROPERTY(QString errorColor MEMBER mErrorColor READ errorColor WRITE setErrorColor)
  Q_PROPERTY(QString infoColor MEMBER mInfoColor READ infoColor WRITE setInfoColor)
  // ANSI escape code colors
  Q_PROPERTY(QString ansiBlack MEMBER mAnsiBlack READ ansiBlack WRITE setAnsiBlack)
  Q_PROPERTY(QString ansiRed MEMBER mAnsiRed READ ansiRed WRITE setAnsiRed)
  Q_PROPERTY(QString ansiGreen MEMBER mAnsiGreen READ ansiGreen WRITE setAnsiGreen)
  Q_PROPERTY(QString ansiYellow MEMBER mAnsiYellow READ ansiYellow WRITE setAnsiYellow)
  Q_PROPERTY(QString ansiBlue MEMBER mAnsiBlue READ ansiBlue WRITE setAnsiBlue)
  Q_PROPERTY(QString ansiMagenta MEMBER mAnsiMagenta READ ansiMagenta WRITE setAnsiMagenta)
  Q_PROPERTY(QString ansiCyan MEMBER mAnsiCyan READ ansiCyan WRITE setAnsiCyan)
  Q_PROPERTY(QString ansiWhite MEMBER mAnsiWhite READ ansiWhite WRITE setAnsiWhite)

public:
  explicit WbConsole(QWidget *parent = NULL, const QString &name = QString("Console"));
  virtual ~WbConsole() {}

  // parse compilation error line
  void jumpToError(const QString &errorLine);

  const QString &errorColor() const { return mErrorColor; }
  const QString &infoColor() const { return mInfoColor; }

  const QString &ansiBlack() const { return mAnsiBlack; }
  const QString &ansiRed() const { return mAnsiRed; }
  const QString &ansiGreen() const { return mAnsiGreen; }
  const QString &ansiYellow() const { return mAnsiYellow; }
  const QString &ansiBlue() const { return mAnsiBlue; }
  const QString &ansiMagenta() const { return mAnsiMagenta; }
  const QString &ansiCyan() const { return mAnsiCyan; }
  const QString &ansiWhite() const { return mAnsiWhite; }

  void setErrorColor(const QString &color) { mErrorColor = color; }
  void setInfoColor(const QString &color) { mInfoColor = color; }

  void setAnsiBlack(const QString &color) { mAnsiBlack = color; }
  void setAnsiRed(const QString &color) { mAnsiRed = color; }
  void setAnsiGreen(const QString &color) { mAnsiGreen = color; }
  void setAnsiYellow(const QString &color) { mAnsiYellow = color; }
  void setAnsiBlue(const QString &color) { mAnsiBlue = color; }
  void setAnsiMagenta(const QString &color) { mAnsiMagenta = color; }
  void setAnsiCyan(const QString &color) { mAnsiCyan = color; }
  void setAnsiWhite(const QString &color) { mAnsiWhite = color; }

  const QStringList getEnabledFilters() const { return mEnabledFilters; }
  void setEnabledFilters(const QStringList &filters);

  const QStringList getEnabledLevels() const { return mEnabledLevels; }
  void setEnabledLevels(const QStringList &levels);

  const QString name() const { return mConsoleName; }

signals:
  void closed();

public slots:
  // clear console and resets its attributes when performed by webots
  // only from within a controller, this option is set to false
  void clear(bool reset = true);

  // rename the console
  void rename();

  // append internal error message of Webots
  // the message color depends on the level
  void appendLog(WbLog::Level level, const QString &message, bool popup, const QString &logName);

private:
  QString mErrorColor, mInfoColor;
  QString mAnsiBlack, mAnsiRed, mAnsiGreen, mAnsiYellow, mAnsiBlue, mAnsiMagenta, mAnsiCyan, mAnsiWhite;
  QStringList mEnabledFilters, mEnabledLevels;
  ConsoleEdit *mEditor;
  QRegularExpression **mErrorPatterns;
  QString mForegroundColor;
  QString mBackgroundColor;
  QString mConsoleName;
  bool mBold;
  bool mUnderline;
  bool mIsOverwriteEnabled;

  void resetFormat();
  QString htmlSpan(const QString &s, WbLog::Level level) const;
  void handleCRAndLF(const QString &msg);
  void handlePossibleAnsiEscapeSequences(const QString &msg, WbLog::Level);
  QRegularExpression **createErrorMatchingPatterns() const;
  void updateTitle();

  void openFindDialog();
  WbFindReplaceDialog *mFindDialog;
  WbTextFind *mTextFind;

protected slots:
  void closeEvent(QCloseEvent *event) override;

private slots:
  void updateFont();
  void enableCopyAction(bool enabled);
  void handleUserCommand(WbAction::WbActionKind actionKind);
  void deleteFindDialog();
  void enableFilter(const QString &filter);
  void disableFilter(const QString &filter);
  void enableLevel(const QString &level);
  void disableLevel(const QString &level);
};

#endif
