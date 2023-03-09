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
 * Description:  Class defining the robot window of the e-puck
 */

#ifndef BOTSTUDIO_WINDOW_HPP
#define BOTSTUDIO_WINDOW_HPP

#include <gui/MainWindow.hpp>

class Model;
class AutomatonWidget;
class RemoteControlWidget;
class SwitchWidget;

QT_BEGIN_NAMESPACE
class QGraphicsView;
class QButtonGroup;
class QToolButton;
class QFile;
class QActionGroup;
QT_END_NAMESPACE

class BotStudioWindow : public webotsQtUtils::MainWindow {
  Q_OBJECT

public:
  BotStudioWindow();
  virtual ~BotStudioWindow();
  void updateValues();

private slots:
  void openStateMachine();
  void saveStateMachine();
  void saveAsStateMachine();
  void selectionToolSelected(bool checked);
  void newStateToolSelected(bool checked);
  void newTransitionToolSelected(bool checked);
  void setSelectedStateAsInitial();
  void deleteSelectedObjects();
  void deleteAllObjects();
  void upload(bool);

  void updateToolBars();

  void showWarningMessage(const QString &text);

private:
  virtual void hideEvent(QHideEvent *event);

  void loadStateMachine();

  void createActions();
  void createToolBars();
  void createStatusBar();
  void createDockWindows();

  Model *mModel;
  AutomatonWidget *mAutomatonWidget;

  RemoteControlWidget *mRemoteControlWidget;
  QDockWidget *mRemoteControlDock;

  SwitchWidget *mSwitchWidget;
  QDockWidget *mSwitchWidgetDock;

  QToolBar *mFileToolBar;
  QToolBar *mEditToolBar;

  QButtonGroup *mModeButtonGroup;
  QToolButton *mSelectionButton;
  QToolButton *mNewStateButton;
  QToolButton *mNewTransitionButton;

  QActionGroup *mActionGroup;
  QAction *mNewStateMachineAct;
  QAction *mOpenStateMachineAct;
  QAction *mSaveAct;
  QAction *mSaveAsAct;
  QAction *mSelectionAct;
  QAction *mNewStateAct;
  QAction *mNewTransitionAct;
  QAction *mDeleteAct;
  QAction *mInitialStateAct;
  QAction *mUploadAct;

  QFile *mFile;

  QString mBaseWindowTitle;
};

#endif
