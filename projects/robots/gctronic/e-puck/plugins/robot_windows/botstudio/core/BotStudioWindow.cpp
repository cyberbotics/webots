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

/*
 * Description:  Implementation of the Gui.hpp functions
 */

#include "BotStudioWindow.hpp"
#include "Automaton.hpp"
#include "AutomatonWidget.hpp"
#include "BotStudioPaths.hpp"
#include "Model.hpp"
#include "RemoteControlWidget.hpp"
#include "RobotFacade.hpp"
#include "RobotObjectFactory.hpp"
#include "SwitchWidget.hpp"

#include <webots/robot.h>

#include <QtCore/QFileInfo>
#include <QtCore/QTextStream>
#include <QtGui/QAction>
#include <QtGui/QActionGroup>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>

using namespace webotsQtUtils;

static const int STATUS_BAR_TIME_OUT = 3000;

BotStudioWindow::BotStudioWindow() : MainWindow() {
  mModel = Model::instance();

  QString defaultFileName = "output.bsg";
  QString fileName = QString(wb_robot_get_controller_name()) + ".bsg";
  if (QFile::exists(fileName))
    defaultFileName = fileName;
  mFile = new QFile(defaultFileName);

  mBaseWindowTitle = "BotStudio (" + QString(wb_robot_get_name()) + ")";
  setWindowTitle(mBaseWindowTitle);

  mAutomatonWidget = new AutomatonWidget(this);
  setCentralWidget(mAutomatonWidget);

  createActions();
  createToolBars();
  createStatusBar();
  createDockWindows();
  updateToolBars();

  setUnifiedTitleAndToolBarOnMac(true);

  connect(mAutomatonWidget->scene(), SIGNAL(selectionChanged()), this, SLOT(updateToolBars()));
  connect(mModel->instance()->robotFacade(), SIGNAL(warning(const QString &)), this, SLOT(showWarningMessage(const QString &)));
  loadStateMachine();
}

BotStudioWindow::~BotStudioWindow() {
  delete mAutomatonWidget;
  delete mRemoteControlWidget;
  delete mRemoteControlDock;
  delete mSwitchWidget;
  delete mSwitchWidgetDock;

  delete mActionGroup;

  delete mNewStateMachineAct;
  delete mOpenStateMachineAct;
  delete mSaveAct;
  delete mSaveAsAct;
  delete mSelectionAct;
  delete mNewStateAct;
  delete mNewTransitionAct;
  delete mDeleteAct;
  delete mInitialStateAct;
  delete mUploadAct;

  delete mFile;
}

void BotStudioWindow::updateValues() {
  mRemoteControlWidget->updateValues();
  mSwitchWidget->updateValues();
}

void BotStudioWindow::createActions() {
  mActionGroup = new QActionGroup(this);

  mNewStateMachineAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "new.xpm"), tr("&New"), this);
  mNewStateMachineAct->setShortcuts(QKeySequence::New);
  mNewStateMachineAct->setStatusTip(tr("Create an empty state machine"));
  connect(mNewStateMachineAct, SIGNAL(triggered()), this, SLOT(deleteAllObjects()));

  mOpenStateMachineAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "open.xpm"), tr("&Open"), this);
  mOpenStateMachineAct->setShortcuts(QKeySequence::Open);
  mOpenStateMachineAct->setStatusTip(tr("Open an existing state machine"));
  connect(mOpenStateMachineAct, SIGNAL(triggered()), this, SLOT(openStateMachine()));

  mSaveAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "save.xpm"), tr("&Save"), this);
  mSaveAct->setShortcuts(QKeySequence::Save);
  mSaveAct->setStatusTip(tr("Save the current state machine"));
  connect(mSaveAct, SIGNAL(triggered()), this, SLOT(saveStateMachine()));

  mSaveAsAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "save_as.xpm"), tr("Save As..."), this);
  mSaveAsAct->setStatusTip(tr("Save the current state machine into a specfic file"));
  connect(mSaveAsAct, SIGNAL(triggered()), this, SLOT(saveAsStateMachine()));

  mSelectionAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "arrow.xpm"), tr("&Selection Tool"), this);
  mSelectionAct->setShortcut(Qt::Key_P);
  mSelectionAct->setStatusTip(tr("Allow to select states or transitions (P)"));
  mSelectionAct->setCheckable(true);
  mSelectionAct->setActionGroup(mActionGroup);
  connect(mSelectionAct, SIGNAL(triggered(bool)), this, SLOT(selectionToolSelected(bool)));

  mNewStateAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "new_state.xpm"), tr("&New State Tool"), this);
  mNewStateAct->setShortcut(Qt::Key_S);
  mNewStateAct->setStatusTip(tr("Allow to create new states (S)"));
  mNewStateAct->setCheckable(true);
  mNewStateAct->setActionGroup(mActionGroup);
  connect(mNewStateAct, SIGNAL(triggered(bool)), this, SLOT(newStateToolSelected(bool)));

  mNewTransitionAct =
    new QAction(QIcon(BotStudioPaths::getIconsPath() + "new_transition.xpm"), tr("&New Transition Tool"), this);
  mNewTransitionAct->setShortcut(Qt::Key_T);
  mNewTransitionAct->setStatusTip(tr("Allow to create new transitions (T)"));
  mNewTransitionAct->setCheckable(true);
  mNewTransitionAct->setActionGroup(mActionGroup);
  connect(mNewTransitionAct, SIGNAL(triggered(bool)), this, SLOT(newTransitionToolSelected(bool)));

  mInitialStateAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "flag.xpm"), tr("&Initial State"), this);
  mInitialStateAct->setShortcut(Qt::Key_I);
  mInitialStateAct->setStatusTip(tr("Set the selected state as the initial state (I)"));
  connect(mInitialStateAct, SIGNAL(triggered()), this, SLOT(setSelectedStateAsInitial()));

  mDeleteAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "delete.xpm"), tr("&Delete"), this);
  mDeleteAct->setShortcuts(QKeySequence::Delete);
  mDeleteAct->setStatusTip(tr("Delete the selected transition or state (Del)"));
  connect(mDeleteAct, SIGNAL(triggered()), this, SLOT(deleteSelectedObjects()));

  mUploadAct = new QAction(QIcon(BotStudioPaths::getIconsPath() + "upload.xpm"), tr("&Upload"), this);
  mUploadAct->setShortcut(Qt::Key_U);
  mUploadAct->setStatusTip(tr("Upload the current state machine to the robot (U)"));
  mUploadAct->setCheckable(true);
  connect(mUploadAct, SIGNAL(triggered(bool)), this, SLOT(upload(bool)));

  // allow to revert to the selection mode after creating a State or a Transition
  connect(mModel->automaton(), SIGNAL(stateCreated(State *)), mSelectionAct, SLOT(trigger()));
  connect(mModel->automaton(), SIGNAL(transitionCreated(Transition *)), mSelectionAct, SLOT(trigger()));

  mSelectionAct->setChecked(true);
}

void BotStudioWindow::createToolBars() {
  mSelectionButton = new QToolButton;
  mSelectionButton->setDefaultAction(mSelectionAct);
  mNewStateButton = new QToolButton;
  mNewStateButton->setDefaultAction(mNewStateAct);
  mNewTransitionButton = new QToolButton;
  mNewTransitionButton->setDefaultAction(mNewTransitionAct);

  mModeButtonGroup = new QButtonGroup(this);
  mModeButtonGroup->addButton(mSelectionButton);
  mModeButtonGroup->addButton(mNewStateButton);
  mModeButtonGroup->addButton(mNewTransitionButton);

  mFileToolBar = addToolBar(tr("File"));
  mFileToolBar->setFloatable(false);
  mFileToolBar->addAction(mNewStateMachineAct);
  mFileToolBar->addAction(mOpenStateMachineAct);
  mFileToolBar->addAction(mSaveAct);
  mFileToolBar->addAction(mSaveAsAct);

  mEditToolBar = addToolBar(tr("Edit"));
  mEditToolBar->setFloatable(false);
  mEditToolBar->addWidget(mSelectionButton);
  mEditToolBar->addWidget(mNewStateButton);
  mEditToolBar->addWidget(mNewTransitionButton);
  mEditToolBar->addSeparator();
  mEditToolBar->addAction(mInitialStateAct);
  mEditToolBar->addAction(mDeleteAct);
  mEditToolBar->addSeparator();
  mEditToolBar->addAction(mUploadAct);
}

void BotStudioWindow::createStatusBar() {
  statusBar()->showMessage(tr("Ready"));
}

void BotStudioWindow::createDockWindows() {
  mRemoteControlWidget = new RemoteControlWidget;
  mRemoteControlDock = new QDockWidget(tr("Remote Control"));
  mRemoteControlDock->setWidget(mRemoteControlWidget);
  mRemoteControlDock->setFeatures(mRemoteControlDock->features() & ~QDockWidget::DockWidgetFloatable &
                                  ~QDockWidget::DockWidgetClosable);
  addDockWidget(Qt::RightDockWidgetArea, mRemoteControlDock);

  mSwitchWidget = new SwitchWidget;
  mSwitchWidgetDock = new QDockWidget(QString(RobotObjectFactory::instance()->name()) + " viewer");
  mSwitchWidgetDock->setWidget(mSwitchWidget);
  mSwitchWidgetDock->setFeatures(mSwitchWidgetDock->features() & ~QDockWidget::DockWidgetFloatable &
                                 ~QDockWidget::DockWidgetClosable);
  addDockWidget(Qt::RightDockWidgetArea, mSwitchWidgetDock);
}

void BotStudioWindow::openStateMachine() {
  // the static function QFileDialog::getOpenFileName doesn't work as expected
  QFileDialog dialog(this, tr("Open a BotStudio file"));
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setNameFilter(tr("BotStudio files (*.bsg)"));
  dialog.setDirectory(QDir::current());
  dialog.setAcceptMode(QFileDialog::AcceptOpen);
  QStringList fileNames;
  if (dialog.exec())
    fileNames = dialog.selectedFiles();

  if (fileNames.size() > 0 && !fileNames.isEmpty()) {
    mFile->setFileName(fileNames[0]);
    loadStateMachine();
    updateToolBars();
  }
}

void BotStudioWindow::loadStateMachine() {
  mModel->setRunning(false);

  if (!mFile->open(QIODevice::ReadOnly | QIODevice::Text)) {
    QString s = tr("Cannot open the %1 file").arg(mFile->fileName());
    statusBar()->showMessage(s, STATUS_BAR_TIME_OUT);
    return;
  }

  mModel->automaton()->deleteAllObjects();

  QTextStream in(mFile);
  QString all = in.readAll();
  mFile->close();

  try {
    mModel->fromString(all);
  } catch (const QString &error) {
    QString s = tr("Problem when loading the %1 file: ").arg(mFile->fileName());
    s += error;
    statusBar()->showMessage(s, STATUS_BAR_TIME_OUT);
    mModel->automaton()->deleteAllObjects();
    return;
  }

  setWindowTitle(mBaseWindowTitle + " - " + QFileInfo(*mFile).fileName());

  QString s;
  s += tr("State machine %1 loaded").arg(mFile->fileName());
  statusBar()->showMessage(s, STATUS_BAR_TIME_OUT);
}

void BotStudioWindow::saveStateMachine() {
  if (mFile->open(QFile::WriteOnly | QFile::Truncate)) {
    QTextStream out(mFile);
    out << mModel->toString();
    mFile->close();

    QString s;
    s += tr("State machine saved into %1").arg(mFile->fileName());
    statusBar()->showMessage(s, STATUS_BAR_TIME_OUT);
  } else {
    QString s = tr("Cannot open the %1 file").arg(mFile->fileName());
    statusBar()->showMessage(s, STATUS_BAR_TIME_OUT);
  }
}

void BotStudioWindow::saveAsStateMachine() {
  // the static function QFileDialog::getSaveFileName doesn't work as expected
  QFileDialog dialog(this, tr("Save a BotStudio file"));
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setNameFilter(tr("BotStudio files (*.bsg)"));
  dialog.setDirectory(QDir::current());
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  QStringList fileNames;
  if (dialog.exec())
    fileNames = dialog.selectedFiles();

  if (fileNames.size() > 0 && !fileNames.isEmpty())
    mFile->setFileName(fileNames[0]);

  saveStateMachine();
}

void BotStudioWindow::selectionToolSelected(bool checked) {
  if (!checked)
    return;
  mAutomatonWidget->setMode(AutomatonWidget::SelectionMode);
}

void BotStudioWindow::newStateToolSelected(bool checked) {
  if (!checked)
    return;
  mAutomatonWidget->setMode(AutomatonWidget::StateMode);
}

void BotStudioWindow::newTransitionToolSelected(bool checked) {
  if (!checked)
    return;
  mAutomatonWidget->setMode(AutomatonWidget::TransitionMode);
}

void BotStudioWindow::setSelectedStateAsInitial() {
  mModel->automaton()->setSelectedStateAsInitial();
}

void BotStudioWindow::deleteSelectedObjects() {
  mModel->automaton()->deleteSelectedObjects();
  updateToolBars();
}

void BotStudioWindow::deleteAllObjects() {
  mModel->automaton()->deleteAllObjects();
  updateToolBars();
}

void BotStudioWindow::upload(bool checked) {
  mModel->setRunning(checked);
  mSwitchWidget->setEnabled(!checked);
  mRemoteControlWidget->setEnabled(!checked);
  mAutomatonWidget->setEnabled(!checked);

  if (checked && checked != mModel->isRunning()) {
    QString title = tr("Warning");
    QString text = tr("This automaton cannot run");
    QString info = tr("Please make sure that at least one state is initial");

    QMessageBox msgBox(this);
    msgBox.setWindowTitle(title);
    msgBox.setText(text);
    msgBox.setInformativeText(info);
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();

    statusBar()->showMessage(text, STATUS_BAR_TIME_OUT);
  }
}

void BotStudioWindow::updateToolBars() {
  int numberOfSelectedItems = mModel->automaton()->computeNumberOfSelectedItems();
  int numberOfSelectedStates = mModel->automaton()->computeNumberOfSelectedStates();
  mInitialStateAct->setEnabled(numberOfSelectedStates == 1);
  mDeleteAct->setEnabled(numberOfSelectedItems > 0);
}

void BotStudioWindow::hideEvent(QHideEvent *event) {
  if (mModel->isRunning())
    QMessageBox::warning(this, tr("Warning"), tr("The robot controller was suspended as you closed the Botstudio window."));
  MainWindow::hideEvent(event);
}

void BotStudioWindow::showWarningMessage(const QString &text) {
  QMessageBox::warning(this, windowTitle() + " Warning", text);
}
