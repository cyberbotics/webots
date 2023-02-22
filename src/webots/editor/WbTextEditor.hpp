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

#ifndef WB_TEXT_EDITOR_HPP
#define WB_TEXT_EDITOR_HPP

//
// Description: Webots text editor
//   -with editing actions: cut, copy paste, etc
//   -with multiple tabbed buffers
//   -without compilation capabilities
//
// Inherited by: WbBuildEditor
//

#include "WbActionManager.hpp"
#include "WbDockWidget.hpp"

class WbFindReplaceDialog;
class WbTextBuffer;
class WbTextFind;

class QPrinter;
class QRegularExpression;
class QTabWidget;
class QToolBar;

class WbTextEditor : public WbDockWidget {
  Q_OBJECT

public:
  explicit WbTextEditor(QWidget *parent, const QString &toolBarAlign);
  virtual ~WbTextEditor();

  // index of currently selected tab, or -1 if there is no tab
  int selectedTab() const;
  void selectTab(int tab);

  // open a file
  // if the file is already open in a tab then select that tab
  // if title is not specified it will be computed from the file path
  bool openFile(const QString &path, const QString &title = QString());

  // close all tabs, open the specified list of files and select the specified tab
  void openFiles(const QStringList &list, int selectedTab);

  // returns the list of currently open files
  QStringList openFiles() const;

  // close all tabs
  void closeAllBuffers();
  void ignoreFileChangedEvent();

  void updateProjectPath(const QString &oldPath, const QString &newPath);

public slots:
  void handleUserCommand(WbAction::WbActionKind action);

protected:
  WbTextBuffer *currentBuffer() const { return mCurrentBuffer; }
  int bufferCount() const;
  WbTextBuffer *buffer(int tab) const;
  QToolBar *toolBar() const { return mToolBar; }

protected slots:
  // cppcheck-suppress virtualCallInConstructor
  virtual void updateGui();
  void tabChanged(int);
  void preview();
  bool saveAllFiles();

private slots:
  void newFile();
  void openFileDialog();
  void saveFile();
  void saveFileAs();
  void revertFile();
  void openReplaceDialog();
  void deleteReplaceDialog();
  void deleteFindDialog();
  void goToLine();
  void toggleLineComment();
  void print();
  void printPreview();
  void modificationChanged(bool changed);
  void closeBufferIfAccepted(int tab);
  void selectTab();
  void enableCopy(bool enabled);
  void enableUndo(bool enabled);
  void enableRedo(bool enabled);
  void showModifiedTextBuffer();

private:
  QToolBar *mToolBar;
  WbTextBuffer *mCurrentBuffer;
  QTabWidget *mTabWidget;
  QPrinter *mPrinter;
  WbTextFind *mTextFind;
  WbFindReplaceDialog *mFindDialog, *mReplaceDialog;

  void connectBuffer(WbTextBuffer *buffer);
  void openFindDialog();
  bool load(const QString &fn);
  void connectActions();
  QToolBar *createToolBar();
  void updateEditMenu();
  void updateFileNames();
  void updateApplicationActions();
  bool saveBuffer(WbTextBuffer *buffer, bool saveAs = false);
  void selectBuffer(WbTextBuffer *buffer);
  void closeBuffer(int tab, bool closeAnyway = false);
  void highlightSearchText(const QRegularExpression &regularExpression);
};

#endif
