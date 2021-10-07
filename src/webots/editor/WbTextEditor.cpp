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

#include "WbTextEditor.hpp"

#include "WbClipboard.hpp"
#include "WbFindReplaceDialog.hpp"
#include "WbMessageBox.hpp"
#include "WbProject.hpp"
#include "WbProjectRelocationDialog.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"
#include "WbTextBuffer.hpp"
#include "WbVariant.hpp"

#include <QtCore/QDir>
#include <QtPrintSupport/QPrintDialog>
#include <QtPrintSupport/QPrintPreviewDialog>
#include <QtPrintSupport/QPrinter>
#include <QtWidgets/QAction>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QTabBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>

#include <cassert>

WbTextEditor::WbTextEditor(QWidget *parent, const QString &toolBarAlign) : WbDockWidget(parent) {
  setObjectName("TextEditor");
  setTabbedTitle("Text Editor");

  mCurrentBuffer = NULL;
  mFindDialog = NULL;
  mReplaceDialog = NULL;
  mTextFind = new WbTextFind(NULL);
  connect(mTextFind, &WbTextFind::findStringChanged, this, &WbTextEditor::highlightSearchText);

  mPrinter = NULL;  // we create it only when needed to avoid heavy initialization overhead (including networking)

  // setup for main window
  QAction *action = toggleViewAction();
  action->setText("&Text Editor");
  action->setStatusTip("Toggle the view of the text editor.");
  action->setShortcut(Qt::CTRL + Qt::Key_E);

  connectActions();
  mToolBar = createToolBar();

  mTabWidget = new QTabWidget(this);
  mTabWidget->setMovable(true);
  mTabWidget->setTabsClosable(true);
  connect(mTabWidget, &QTabWidget::currentChanged, this, &WbTextEditor::tabChanged);
  connect(mTabWidget, &QTabWidget::tabCloseRequested, this, &WbTextEditor::closeBufferIfAccepted);

  QVBoxLayout *layout = new QVBoxLayout();
  layout->setSpacing(0);
  layout->setContentsMargins(0, 0, 0, 0);
  mToolBar->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  if (toolBarAlign == "center") {
    QHBoxLayout *hlayout = new QHBoxLayout();
    hlayout->addWidget(mToolBar);
    layout->addLayout(hlayout, 0);
  } else  // assuming left alignment
    layout->addWidget(mToolBar);
  layout->addWidget(mTabWidget);

  QWidget *widget = new QWidget(this);
  setWidget(widget);
  widget->setLayout(layout);
}

WbTextEditor::~WbTextEditor() {
  delete mPrinter;
  delete mTextFind;
}

QToolBar *WbTextEditor::createToolBar() {
  QToolBar *mToolBar = new QToolBar(this);
  WbActionManager *actionManager = WbActionManager::instance();

  QAction *action = actionManager->action(WbAction::NEW_FILE);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("editorButton");

  action = actionManager->action(WbAction::OPEN_FILE);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("editorButton");

  action = actionManager->action(WbAction::SAVE_FILE);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("editorButton");

  action = actionManager->action(WbAction::SAVE_FILE_AS);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("editorButton");

  action = actionManager->action(WbAction::REVERT_FILE);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("editorButton");

  mToolBar->addSeparator();

  action = actionManager->action(WbAction::FIND);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("editorButton");

  action = actionManager->action(WbAction::REPLACE);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("editorButton");

  return mToolBar;
}

void WbTextEditor::connectActions() {
  WbActionManager *actionManager = WbActionManager::instance();
  connect(actionManager->action(WbAction::NEW_FILE), &QAction::triggered, this, &WbTextEditor::newFile);
  connect(actionManager->action(WbAction::OPEN_FILE), &QAction::triggered, this, &WbTextEditor::openFileDialog);
  connect(actionManager->action(WbAction::SAVE_FILE), &QAction::triggered, this, &WbTextEditor::saveFile);
  connect(actionManager->action(WbAction::SAVE_FILE_AS), &QAction::triggered, this, &WbTextEditor::saveFileAs);
  connect(actionManager->action(WbAction::REVERT_FILE), &QAction::triggered, this, &WbTextEditor::revertFile);
  connect(actionManager->action(WbAction::REPLACE), &QAction::triggered, this, &WbTextEditor::openReplaceDialog);
  connect(actionManager->action(WbAction::GO_TO_LINE), &QAction::triggered, this, &WbTextEditor::goToLine);
  connect(actionManager->action(WbAction::TOGGLE_LINE_COMMENT), &QAction::triggered, this, &WbTextEditor::toggleLineComment);
  connect(actionManager->action(WbAction::DUPLICATE_SELECTION), &QAction::triggered, this, &WbTextEditor::duplicateSelection);
  connect(actionManager->action(WbAction::TRANSPOSE_LINE), &QAction::triggered, this, &WbTextEditor::transposeCurrentLine);
  connect(actionManager->action(WbAction::PRINT), &QAction::triggered, this, &WbTextEditor::print);
  connect(actionManager->action(WbAction::PRINT_PREVIEW), &QAction::triggered, this, &WbTextEditor::printPreview);

  connect(actionManager, &WbActionManager::userTextEditCommandReceived, this, &WbTextEditor::handleUserCommand);
}

void WbTextEditor::updateGui() {
  WbActionManager *actionManager = WbActionManager::instance();
  actionManager->setEnabled(WbAction::SAVE_FILE, mCurrentBuffer && mCurrentBuffer->isModified());
  actionManager->setEnabled(WbAction::SAVE_FILE_AS, mCurrentBuffer);
  actionManager->setEnabled(WbAction::REVERT_FILE, mCurrentBuffer);

  updateFileNames();
  updateEditMenu();
}

void WbTextEditor::updateFileNames() {
  if (bufferCount() == 0) {
    setWindowTitle("Text Editor");
    return;
  }

  // update tabs titles
  for (int i = 0; i < bufferCount(); i++) {
    WbTextBuffer *b = buffer(i);

    QString tabText(b->shortName());
    if (b->document()->isModified())
      tabText += "*";
    mTabWidget->setTabText(i, tabText);
  }

  // update window title
  WbTextBuffer *selectedBuffer = currentBuffer();
  if (selectedBuffer) {
    QString windowTitle(QDir::toNativeSeparators(selectedBuffer->fileName()));
    if (selectedBuffer->document()->isModified())
      windowTitle += "*";
    setWindowTitle(windowTitle);
  } else
    setWindowTitle(tr("Text Editor"));
}

void WbTextEditor::updateEditMenu() {
  WbActionManager::instance()->setEnabled(WbAction::TOGGLE_LINE_COMMENT, mCurrentBuffer);
  WbActionManager::instance()->setEnabled(WbAction::DUPLICATE_SELECTION, mCurrentBuffer);
  WbActionManager::instance()->setEnabled(WbAction::TRANSPOSE_LINE, mCurrentBuffer);
  WbActionManager::instance()->enableTextEditActions(mCurrentBuffer);
  WbActionManager::instance()->setFocusObject(this);

  updateApplicationActions();
}

void WbTextEditor::updateApplicationActions() {
  QTextDocument *doc = NULL;
  if (mCurrentBuffer)
    doc = mCurrentBuffer->document();

  enableUndo(doc && doc->isUndoAvailable());
  enableRedo(doc && doc->isRedoAvailable());
  enableCopy(mCurrentBuffer && mCurrentBuffer->hasSelection());
  WbActionManager::instance()->setEnabled(WbAction::PASTE, mCurrentBuffer && !WbClipboard::instance()->isEmpty());
  WbActionManager::instance()->setEnabled(WbAction::SELECT_ALL, mCurrentBuffer);
}

void WbTextEditor::enableCopy(bool enabled) {
  WbActionManager::instance()->setEnabled(WbAction::CUT, enabled);
  WbActionManager::instance()->setEnabled(WbAction::COPY, enabled);
}

void WbTextEditor::enableUndo(bool enabled) {
  WbActionManager::instance()->setEnabled(WbAction::UNDO, enabled);
}

void WbTextEditor::enableRedo(bool enabled) {
  WbActionManager::instance()->setEnabled(WbAction::REDO, enabled);
}

void WbTextEditor::modificationChanged(bool changed) {
  updateFileNames();
  WbActionManager::instance()->setEnabled(WbAction::SAVE_FILE, mCurrentBuffer && changed);
}

void WbTextEditor::tabChanged(int tab) {
  if (mCurrentBuffer) {
    disconnect(mCurrentBuffer, 0, this, 0);
  }

  mCurrentBuffer = buffer(tab);

  if (mCurrentBuffer) {
    mCurrentBuffer->setFocusPolicy(Qt::ClickFocus);
    mTextFind->setEditor(mCurrentBuffer);
    WbActionManager::instance()->setEnabled(WbAction::SAVE_FILE, mCurrentBuffer->isModified());
    connect(mCurrentBuffer, &WbTextBuffer::undoAvailable, this, &WbTextEditor::enableUndo);
    connect(mCurrentBuffer, &WbTextBuffer::redoAvailable, this, &WbTextEditor::enableRedo);
    connect(mCurrentBuffer, &WbTextBuffer::copyAvailable, this, &WbTextEditor::enableCopy);
    connect(mCurrentBuffer, &WbTextBuffer::modificationChanged, this, &WbTextEditor::modificationChanged);
    connect(mCurrentBuffer, &WbTextBuffer::focusIn, this, &WbTextEditor::updateEditMenu);
  }
  updateGui();
}

void WbTextEditor::handleUserCommand(WbAction::WbActionKind action) {
  switch (action) {
    case WbAction::CUT:
      mCurrentBuffer->cut();
      break;
    case WbAction::COPY:
      mCurrentBuffer->copy();
      break;
    case WbAction::PASTE:
      mCurrentBuffer->paste();
      break;
    case WbAction::SELECT_ALL:
      mCurrentBuffer->selectAll();
      break;
    case WbAction::UNDO:
      mCurrentBuffer->undo();
      break;
    case WbAction::REDO:
      mCurrentBuffer->redo();
      break;
    case WbAction::FIND:
      openFindDialog();
      break;
    case WbAction::FIND_NEXT:
      if (mFindDialog != NULL)
        mFindDialog->next();
      else if (mReplaceDialog != NULL)
        mReplaceDialog->next();
      else
        WbFindReplaceDialog::findNext(mTextFind, this);
      break;
    case WbAction::FIND_PREVIOUS:
      if (mFindDialog != NULL)
        mFindDialog->previous();
      else if (mReplaceDialog != NULL)
        mReplaceDialog->previous();
      else
        WbFindReplaceDialog::findPrevious(mTextFind, this);
      break;
    default:
      break;
  }

  updateApplicationActions();
}

void WbTextEditor::newFile() {
  WbTextBuffer *buffer = new WbTextBuffer(this);
  connectBuffer(buffer);
  mTabWidget->addTab(buffer, "New");
  selectTab(mTabWidget->count() - 1);
  if (!isVisible())
    toggleViewAction()->trigger();
}

void WbTextEditor::selectTab(int tab) {
  if (tab < 0 || tab >= mTabWidget->count())
    return;
  mTabWidget->setCurrentIndex(tab);
  updateGui();
}

bool WbTextEditor::openFile(const QString &path) {
  // see if this file is already open in a tab
  int n = bufferCount();
  if (n > 0) {
    QFileInfo info(path);
    QString canonical = info.canonicalFilePath();
    for (int i = 0; i < n; i++) {
      WbTextBuffer *buf = buffer(i);
      if (canonical == buf->fileName()) {
        selectTab(i);
        return true;
      }
    }
  }

  // add a new tab
  WbTextBuffer *buf = new WbTextBuffer(this);
  connectBuffer(buf);
  if (!buf->load(path))
    return false;

  mTabWidget->addTab(buf, buf->shortName());
  selectTab(mTabWidget->count() - 1);

  return true;
}

void WbTextEditor::openFileDialog() {
  WbSimulationState *simulationState = WbSimulationState::instance();
  simulationState->pauseSimulation();

  // find a smart dir
  QString dir;
  WbTextBuffer *buffer = currentBuffer();
  if (buffer)
    dir = buffer->path();
  else if (WbProject::current())
    dir = WbProject::current()->path();
  else
    dir = QDir::homePath();

  // get list of files
  QStringList list = QFileDialog::getOpenFileNames(this, tr("Open File..."), dir, tr("All Files (*)"));
  if (!list.isEmpty()) {
    // open all existing files
    for (int i = 0; i < list.size(); ++i) {
      const QString &name = list.at(i);
      if (QFile::exists(name)) {
        openFile(name);
        if (!isVisible())
          toggleViewAction()->trigger();
      }
    }

    updateGui();
  }

  simulationState->resumeSimulation();
}

void WbTextEditor::revertFile() {
  mCurrentBuffer->revert(true);
  updateGui();
}

void WbTextEditor::selectTab() {
  WbTextBuffer *b = dynamic_cast<WbTextBuffer *>(sender());
  if (!b)
    return;
  selectBuffer(b);
}

void WbTextEditor::selectBuffer(WbTextBuffer *buffer) {
  int count = mTabWidget->count();
  for (int i = 0; i < count; i++)
    if (buffer == this->buffer(i)) {
      selectTab(i);
      break;
    }
}

void WbTextEditor::connectBuffer(WbTextBuffer *buffer) {
  connect(buffer, &WbTextBuffer::fileNameChanged, this, &WbTextEditor::updateGui);
  connect(buffer, &WbTextBuffer::showRequested, this, &WbTextEditor::showModifiedTextBuffer);
}

void WbTextEditor::showModifiedTextBuffer() {
  if (!isHidden())
    return;

  // show text editor and reset selected tab
  WbTextBuffer *b = dynamic_cast<WbTextBuffer *>(sender());
  selectBuffer(b);
  show();
}

void WbTextEditor::openFindDialog() {
  if (mReplaceDialog != NULL) {
    // find and replace dialog already open
    if (mCurrentBuffer->hasSingleBlockSelection())
      mReplaceDialog->setFindString(mCurrentBuffer->textCursor().selectedText());
    mReplaceDialog->show();
    mReplaceDialog->raise();
    mReplaceDialog->activateWindow();
    return;
  }

  if (mFindDialog == NULL) {
    mFindDialog = new WbFindReplaceDialog(mTextFind, false, tr("Text Editor"), this);
    connect(mFindDialog, &WbFindReplaceDialog::finished, this, &WbTextEditor::deleteFindDialog);
  }

  if (mCurrentBuffer->hasSingleBlockSelection()) {
    QString selectedText = mCurrentBuffer->textCursor().selectedText();
    if (!selectedText.isEmpty())
      mFindDialog->setFindString(mCurrentBuffer->textCursor().selectedText());
  }

  mFindDialog->show();
  mFindDialog->raise();
  mFindDialog->activateWindow();
}

void WbTextEditor::openReplaceDialog() {
  if (mFindDialog != NULL) {
    // close find open
    mFindDialog->close();
    mFindDialog = NULL;
  }

  if (mReplaceDialog == NULL) {
    mReplaceDialog = new WbFindReplaceDialog(mTextFind, true, tr("Text Editor"), this);
    connect(mReplaceDialog, &WbFindReplaceDialog::finished, this, &WbTextEditor::deleteReplaceDialog);
  }

  if (mCurrentBuffer->hasSingleBlockSelection()) {
    QString selectedText = mCurrentBuffer->textCursor().selectedText();
    if (!selectedText.isEmpty())
      mReplaceDialog->setFindString(mCurrentBuffer->textCursor().selectedText());
  }

  mReplaceDialog->show();
  mReplaceDialog->raise();
  mReplaceDialog->activateWindow();
}

void WbTextEditor::deleteFindDialog() {
  // WbFindReplaceDialog deletes automatically on close
  mFindDialog = NULL;
}

void WbTextEditor::deleteReplaceDialog() {
  // WbFindReplaceDialog deletes automatically on close
  mReplaceDialog = NULL;
}

void WbTextEditor::highlightSearchText(QRegExp regExp) {
  WbTextBuffer *buffer = dynamic_cast<WbTextBuffer *>(mTabWidget->currentWidget());
  if (buffer)
    buffer->updateSearchTextHighlighting(regExp);
}

void WbTextEditor::goToLine() {
  mCurrentBuffer->goToLine();
}

void WbTextEditor::toggleLineComment() {
  mCurrentBuffer->toggleLineComment();
}

void WbTextEditor::duplicateSelection() {
  mCurrentBuffer->duplicateSelection();
}

void WbTextEditor::transposeCurrentLine() {
  mCurrentBuffer->transposeCurrentLine();
}

void WbTextEditor::print() {
  WbSimulationState *simulationState = WbSimulationState::instance();
  simulationState->pauseSimulation();

  if (!mPrinter)
    mPrinter = new QPrinter(QPrinter::HighResolution);
  QPrintDialog *dialog = new QPrintDialog(mPrinter, this);
  if (mCurrentBuffer->textCursor().hasSelection())
    dialog->addEnabledOption(QAbstractPrintDialog::PrintSelection);
  dialog->setWindowTitle(tr("Print Document"));
  if (dialog->exec() == QDialog::Accepted)
    mCurrentBuffer->print(mPrinter);
  delete dialog;

  simulationState->resumeSimulation();
}

void WbTextEditor::printPreview() {
  WbSimulationState *simulationState = WbSimulationState::instance();
  simulationState->pauseSimulation();

  if (!mPrinter)
    mPrinter = new QPrinter(QPrinter::HighResolution);
  QPrintPreviewDialog preview(mPrinter, this);
  connect(&preview, &QPrintPreviewDialog::paintRequested, this, &WbTextEditor::preview);
  preview.exec();

  simulationState->resumeSimulation();
}

void WbTextEditor::preview() {
  assert(mPrinter);
  mCurrentBuffer->print(mPrinter);
}

void WbTextEditor::saveFile() {
  saveBuffer(mCurrentBuffer);
  updateGui();
}

void WbTextEditor::saveFileAs() {
  saveBuffer(mCurrentBuffer, true);
  updateGui();
}

bool WbTextEditor::saveAllFiles() {
  int tab = mTabWidget->currentIndex();
  int count = mTabWidget->count();
  for (int i = 0; i < count; i++) {
    // only status of selected tab is automatically updated
    mTabWidget->setCurrentIndex(i);
    WbTextBuffer *b = buffer(i);
    if ((b->isModified() || !QFile::exists(b->fileName())) && !saveBuffer(b))
      return false;
  }

  // restore original selected tab
  selectTab(tab);
  updateGui();
  return true;
}

bool WbTextEditor::saveBuffer(WbTextBuffer *buffer, bool saveAs) {
  WbSimulationState *simulationState = WbSimulationState::instance();
  simulationState->pauseSimulation();

  QString fileName = buffer->fileName();

  if (buffer->isUnnamed() || QFileInfo(fileName).isRelative() || saveAs) {
    fileName =
      QFileDialog::getSaveFileName(this, tr("Save as..."), WbProject::computeBestPathForSaveAs(fileName), tr("All Files (*)"));
    if (fileName.isEmpty()) {
      simulationState->resumeSimulation();
      return false;
    }
  }

  simulationState->resumeSimulation();

  // is the file located in Webots installation directory
  // if relocation is needed, then fileName will be updated to match the new text file
  if (!WbProjectRelocationDialog::validateLocation(this, fileName)) {
    simulationState->resumeSimulation();
    return false;
  }
  fileName = QFileInfo(fileName).path() + "/" + QFileInfo(fileName).fileName();
  if (saveAs || buffer->isModified() || !QFile::exists(fileName))
    buffer->saveAs(fileName);

  return true;
}

void WbTextEditor::closeBufferIfAccepted(int tab) {
  closeBuffer(tab, false);
}

void WbTextEditor::closeBuffer(int tab, bool closeAnyway) {
  WbTextBuffer *buf = buffer(tab);
  if (buf->isModified()) {
    QMessageBox::StandardButtons buttons = QMessageBox::Save | QMessageBox::Discard;
    if (!closeAnyway)
      buttons |= QMessageBox::Cancel;
    QString message(tr("The file '%1' is not saved.\nDo you want to save it before closing?").arg(buf->shortName()));
    int ret = WbMessageBox::question(message, this, tr("Question"), QMessageBox::Save, buttons);
    if (ret == QMessageBox::Cancel)
      return;
    else if (ret == QMessageBox::Save)
      saveBuffer(buf);
  }
  mTabWidget->removeTab(tab);
  delete buf;
}

void WbTextEditor::closeAllBuffers() {
  while (mTabWidget->count() > 0)
    closeBuffer(0, true);
}

int WbTextEditor::bufferCount() const {
  return mTabWidget->count();
}

WbTextBuffer *WbTextEditor::buffer(int tab) const {
  return static_cast<WbTextBuffer *>(mTabWidget->widget(tab));
}

void WbTextEditor::ignoreFileChangedEvent() {
  int count = mTabWidget->count();
  for (int i = 0; i < count; i++)
    buffer(i)->ignoreFileChangedEvent();
}

QStringList WbTextEditor::openFiles() const {
  QStringList list;
  int count = mTabWidget->count();
  for (int i = 0; i < count; i++) {
    WbTextBuffer *buf = buffer(i);
    list << buf->fileName();
  }

  return list;
}

void WbTextEditor::openFiles(const QStringList &list, int selectedTab) {
  closeAllBuffers();

  foreach (QString file, list) {
    WbTextBuffer *newBuffer = new WbTextBuffer(this);
    connectBuffer(newBuffer);
    bool success = newBuffer->load(file);
    if (success)
      mTabWidget->addTab(newBuffer, newBuffer->shortName());
    else
      delete newBuffer;
  }

  selectTab(selectedTab);
}

int WbTextEditor::selectedTab() const {
  return mTabWidget->currentIndex();
}

void WbTextEditor::updateProjectPath(const QString &oldPath, const QString &newPath) {
  int count = mTabWidget->count();
  for (int i = 0; i < count; i++) {
    WbTextBuffer *buf = buffer(i);
    QString filename = buf->fileName();
    if (filename.startsWith(oldPath)) {
      filename.replace(oldPath, newPath);
      if (QFile::exists(filename))
        buf->setFileName(filename);
    }
  }
  updateGui();
}
