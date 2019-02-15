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

#include "WbSimulationView.hpp"

#include "WbActionManager.hpp"
#include "WbApplication.hpp"
#include "WbContextMenuGenerator.hpp"
#include "WbControlledWorld.hpp"
#include "WbDockTitleBar.hpp"
#include "WbLog.hpp"
#include "WbMainWindow.hpp"
#include "WbMessageBox.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbRobot.hpp"
#include "WbSceneTree.hpp"
#include "WbSelection.hpp"
#include "WbSimulationState.hpp"
#include "WbSimulationStateIndicator.hpp"
#include "WbSimulationWorld.hpp"
#include "WbSolid.hpp"
#include "WbSoundEngine.hpp"
#include "WbStandardPaths.hpp"
#include "WbSysInfo.hpp"
#include "WbVideoRecorder.hpp"
#include "WbView3D.hpp"
#include "WbViewpoint.hpp"
#include "WbVisualBoundingSphere.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"

#include <QtCore/QFileInfo>
#include <QtCore/QTimer>
#include <QtGui/QImage>
#include <QtGui/QResizeEvent>
#include <QtWidgets/QAction>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMenu>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>

enum { HIDE, SHOW };
enum { VIEW3D, MESSAGE };

WbSimulationView::WbSimulationView(QWidget *parent, const QString &toolBarAlign) :
  QWidget(parent),
  mIsScreenshotRequestedFromGui(false),
  mIsDecorationVisible(true),
  mToolBarExtensionMenu(NULL),
  mSelection(new WbSelection()),
  mSplitter(new QSplitter()),
  mSceneTree(new WbSceneTree(mSplitter)),
  mSplitterStatus(0),
  mSupervisorMovieRecordingEnabled(false) {
  // add container widget to 'hide' the black window by forcing it to
  // have the same position of the rendering window
  QWidget *const view3DVideoResizeWidget = new QWidget(mSplitter);
  mView3D = new WbView3D;
  mView3DContainer = QWidget::createWindowContainer(mView3D, view3DVideoResizeWidget);
  mView3DContainer->setMinimumSize(mView3D->minimumSize());
  mView3D->setParentWidget(mView3DContainer);
  mView3DContainer->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(mView3D, &WbView3D::contextMenuRequested, this, &WbSimulationView::showMenu);

  // it is preferable to resize the mView3DContainer instead of the mView3D
  // when creating a video so that the black window always has exactly the
  // same position than the rendering window
  // but we have to add some special stretching factor during the video
  // recording to fill the remaining available space otherwise the
  // toolbar and scene tree will be automatically resized as well
  mView3DResizeLayout = new QGridLayout(view3DVideoResizeWidget);
  mView3DResizeLayout->setSpacing(0);
  mView3DResizeLayout->setContentsMargins(0, 0, 0, 0);
  mView3DResizeLayout->addWidget(mView3DContainer, 0, 0);
  setView3DResizeStretch(false);

  // central widget
  mSplitter->setObjectName("horizontalSplitter");
  mSplitter->addWidget(mSceneTree);
  mSplitter->addWidget(view3DVideoResizeWidget);
  mSplitter->setStretchFactor(0, 0);
  mSplitter->setStretchFactor(1, 1);
  QList<int> initialSplitterSizes;
  initialSplitterSizes << 0 << (mSceneTree->sizeHint().width() + view3DVideoResizeWidget->sizeHint().width());
  mSplitter->setSizes(initialSplitterSizes);

  mShowSceneTreeButton = new QToolButton(this);
  mShowSceneTreeButton->setObjectName("menuButton");
  mShowSceneTreeButton->setFocusPolicy(Qt::ClickFocus);
  connect(mShowSceneTreeButton, &QToolButton::pressed, this, &WbSimulationView::toggleSceneTreeVisibility);

  // main objects
  createActions();
  mTitleBar = new WbDockTitleBar(false, this);
  mToolBar = createToolBar();
  mNeedRestoreFastMode = false;

  // top level layout
  QVBoxLayout *vlayout = new QVBoxLayout(this);
  vlayout->setSpacing(0);
  vlayout->setContentsMargins(0, 0, 0, 0);
  vlayout->addWidget(mTitleBar, 0);
  mToolBar->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  if (toolBarAlign == "center") {
    QHBoxLayout *hlayout = new QHBoxLayout();
    hlayout->addWidget(mToolBar);
    vlayout->addLayout(hlayout, 0);
  } else  // assuming left alignment
    vlayout->addWidget(mToolBar);
  vlayout->addWidget(mSplitter, 1);

  WbSimulationState *state = WbSimulationState::instance();

  //  show a black screen is fast mode is selected
  if (state->mode() == WbSimulationState::FAST)
    renderABlackScreen();

  connect(mTitleBar, &WbDockTitleBar::closeClicked, this, &WbSimulationView::hide);
  connect(mTitleBar, &WbDockTitleBar::maximizeClicked, this, &WbSimulationView::needsMaximize);
  connect(mTitleBar, &WbDockTitleBar::minimizeClicked, this, &WbSimulationView::needsMinimize);
  connect(mSplitter, &QSplitter::splitterMoved, this, &WbSimulationView::needsActionsUpdate);
  connect(WbActionManager::instance()->action(WbActionManager::STEP), &QAction::triggered, mView3D, &WbView3D::unleashAndClean);
  connect(mView3D, &WbView3D::applicationActionsUpdateRequested, mSceneTree, &WbSceneTree::updateApplicationActions);

  // video recording
  mRecordingTimer = new QTimer(this);
  connect(mRecordingTimer, &QTimer::timeout, this, &WbSimulationView::toggleRecordingIcon);
  connect(WbApplication::instance(), &WbApplication::requestScreenshot, this, &WbSimulationView::takeScreenshotAndSaveAs);
  connect(WbApplication::instance(), SIGNAL(videoCaptureStarted(const QString &, int, int, int, int, int, bool)), this,
          SLOT(startVideoCapture(const QString &, int, int, int, int, int, bool)));
  connect(WbApplication::instance(), &WbApplication::videoCaptureStopped, this, &WbSimulationView::stopVideoCapture);
  connect(WbVideoRecorder::instance(), &WbVideoRecorder::videoCreationStatusChanged, WbApplication::instance(),
          &WbApplication::videoCreationStatusChanged);

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parent);
  assert(mainWindow);
  WbVideoRecorder::setMainWindow(mainWindow);
  mWasMinimized = false;
}

WbSimulationView::~WbSimulationView() {
  // explicitly delete scene tree and 3D view before the selection
  delete mSceneTree;
  mSceneTree = NULL;

  delete mView3D;
  mView3D = NULL;

  delete mSelection;
}

QToolBar *WbSimulationView::createToolBar() {
  mToolBar = new QToolBar(this);

  WbActionManager *manager = WbActionManager::instance();

  mToolBar->addWidget(mShowSceneTreeButton);

  QAction *action = manager->action(WbActionManager::ADD_NEW);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  mToolBar->addSeparator();

  action = manager->action(WbActionManager::RESTORE_VIEWPOINT);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = manager->action(WbActionManager::VIEW_MENU);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");
  QToolButton *viewMenuButton = dynamic_cast<QToolButton *>(mToolBar->widgetForAction(action));
  viewMenuButton->setPopupMode(QToolButton::InstantPopup);
  QMenu *viewMenu = new QMenu(viewMenuButton);
  viewMenu->addAction(manager->action(WbActionManager::FRONT_VIEW));
  viewMenu->addAction(manager->action(WbActionManager::BACK_VIEW));
  viewMenu->addAction(manager->action(WbActionManager::LEFT_VIEW));
  viewMenu->addAction(manager->action(WbActionManager::RIGHT_VIEW));
  viewMenu->addAction(manager->action(WbActionManager::TOP_VIEW));
  viewMenu->addAction(manager->action(WbActionManager::BOTTOM_VIEW));
  viewMenuButton->setMenu(viewMenu);

  mToolBar->addSeparator();

  mToolBarExtensionMenu = mToolBar->findChild<QMenu *>();
  connect(mToolBarExtensionMenu, &QMenu::aboutToShow, this, &WbSimulationView::hideInappropriateToolBarItems);

  action = manager->action(WbActionManager::OPEN_WORLD);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = manager->action(WbActionManager::SAVE_WORLD);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = manager->action(WbActionManager::RELOAD_WORLD);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  mToolBar->addSeparator();

  mToolBar->addWidget(new WbSimulationStateIndicator(mToolBar));

  action = manager->action(WbActionManager::RESET_SIMULATION);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = manager->action(WbActionManager::STEP);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = mPlayAnchor = new QAction(this);  // anchor to help replacing actions
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("invisibleButton");
  mToolBar->widgetForAction(action)->setVisible("false");

  updatePlayButtons();
  connect(WbApplication::instance(), &WbApplication::postWorldLoaded, this, &WbSimulationView::updatePlayButtons);
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbSimulationView::updatePlayButtons);

  mToolBar->addSeparator();

  mToolBar->addAction(mMovieAction);
  mToolBar->widgetForAction(mMovieAction)->setObjectName("menuButton");

  action = manager->action(WbActionManager::ANIMATION);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  mToolBar->addAction(mTakeScreenshotAction);
  mToolBar->widgetForAction(mTakeScreenshotAction)->setObjectName("menuButton");

  mToolBar->addSeparator();

  action = mSoundAnchor = new QAction(this);  // anchor to help replacing actions
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("invisibleButton");
  mToolBar->widgetForAction(action)->setVisible("false");

  mSoundVolumeSlider = new QSlider(Qt::Horizontal, this);

  if (WbPreferences::instance()->value("Sound/mute", false).toBool())
    mSoundVolumeSlider->setSliderPosition(0);
  else
    mSoundVolumeSlider->setSliderPosition(WbPreferences::instance()->value("Sound/volume", 80).toInt());

  mSoundVolumeSlider->setFocusPolicy(Qt::ClickFocus);
  mToolBar->addWidget(mSoundVolumeSlider);
  connect(mSoundVolumeSlider, &QSlider::valueChanged, this, &WbSimulationView::updateSoundVolume);

  updateSoundButtons();

  return mToolBar;
}

void WbSimulationView::createActions() {
  QAction *action;

  action = mToggleView3DAction = new QAction(this);
  action->setCheckable(true);
  action->setChecked(true);
  action->setText(tr("3D View"));
  action->setStatusTip("Toggle the 3D View.");
  action->setShortcut(Qt::CTRL + Qt::Key_B);
  connect(action, &QAction::toggled, this, &WbSimulationView::updateVisibility);

  action = mToggleSceneTreeAction = new QAction(this);
  action->setCheckable(true);
  action->setChecked(true);
  action->setText(tr("Scene Tree"));
  action->setStatusTip("Toggle the scene tree.");
  action->setShortcut(Qt::CTRL + Qt::Key_T);
  connect(action, &QAction::toggled, this, &WbSimulationView::updateVisibility);

  updateSceneTreeActions(true);

  // TODO: for sure there is a clever location to do the following connections
  //       (this has nothing to do with a window)
  WbActionManager *manager = WbActionManager::instance();
  connect(manager->action(WbActionManager::PAUSE), &QAction::triggered, this, &WbSimulationView::pause);
  connect(manager->action(WbActionManager::STEP), &QAction::triggered, this, &WbSimulationView::step);
  connect(manager->action(WbActionManager::REAL_TIME), &QAction::triggered, this, &WbSimulationView::realTime);
  connect(manager->action(WbActionManager::RUN), &QAction::triggered, this, &WbSimulationView::run);
  connect(manager->action(WbActionManager::FAST), &QAction::triggered, this, &WbSimulationView::fast);

  // add actions available in full-screen mode to the current widget
  // otherwise they will be automatically disabled when the toolbar is hidden
  addAction(manager->action(WbActionManager::PAUSE));
  addAction(manager->action(WbActionManager::STEP));
  addAction(manager->action(WbActionManager::REAL_TIME));
  addAction(manager->action(WbActionManager::RUN));
  addAction(manager->action(WbActionManager::FAST));
  addAction(manager->action(WbActionManager::DEL));
  addAction(manager->action(WbActionManager::MOVE_VIEWPOINT_TO_OBJECT));

  mMovieAction = new QAction(this);
  toggleMovieAction(false);

  mTakeScreenshotAction = manager->action(WbActionManager::TAKE_SCREENSHOT);
  connect(mTakeScreenshotAction, &QAction::triggered, this, &WbSimulationView::takeScreenshot);
  // so taking screenshots can be done in full-screen mode, when the toolbar is hidden
  addAction(mTakeScreenshotAction);

  connect(manager->action(WbActionManager::SOUND_UNMUTE), &QAction::triggered, this, &WbSimulationView::unmuteSound);
  connect(manager->action(WbActionManager::SOUND_MUTE), &QAction::triggered, this, &WbSimulationView::muteSound);
}

void WbSimulationView::setMaximized(bool maximized) {
  mTitleBar->setMaximized(maximized);
}

void WbSimulationView::setDecorationVisible(bool visible) {
  static QList<QByteArray> previousState;

  if (visible) {
    restoreState(previousState);
    previousState.clear();
  } else
    previousState = saveState();

  mSceneTree->setVisible(visible);
  mToolBar->setVisible(visible);
  mTitleBar->setVisible(visible);
  mIsDecorationVisible = visible;
}

bool WbSimulationView::isSceneTreeButtonStatusVisible() const {
  return mShowSceneTreeButton->property("state") == HIDE;
}

void WbSimulationView::updateSceneTreeActions(bool enabled) {
  if (enabled) {
    // side bar visible
    mShowSceneTreeButton->setStatusTip(tr("Hide the Scene Tree side bar."));
    mShowSceneTreeButton->setIcon(QIcon("enabledIcons:hide_side_bar.png"));
    mShowSceneTreeButton->setToolTip(mShowSceneTreeButton->statusTip());
    mShowSceneTreeButton->setProperty("state", HIDE);
  } else {
    // side bar hidden
    mShowSceneTreeButton->setStatusTip(tr("Show the Scene Tree side bar."));
    mShowSceneTreeButton->setIcon(QIcon("enabledIcons:show_side_bar.png"));
    mShowSceneTreeButton->setToolTip(mShowSceneTreeButton->statusTip());
    mShowSceneTreeButton->setProperty("state", SHOW);
  }

  mToggleSceneTreeAction->blockSignals(true);
  mToggleSceneTreeAction->setChecked(enabled);
  mToggleSceneTreeAction->blockSignals(false);
}

void WbSimulationView::updateToggleView3DAction(bool enabled) {
  mToggleView3DAction->blockSignals(true);
  mToggleView3DAction->setChecked(enabled);
  mToggleView3DAction->blockSignals(false);
}

void WbSimulationView::needsActionsUpdate(int position, int index) {
  static bool hidden = false;

  if (position == 0 && !hidden) {
    updateSceneTreeActions(false);
    hidden = true;
  } else if (position > 0 && hidden) {
    updateSceneTreeActions(true);
    hidden = false;
  }

  updateToggleView3DAction(mView3D->width() > 0);
}

void WbSimulationView::toggleSceneTreeVisibility() {
  static int lastSplitterPosition = -1;

  setUpdatesEnabled(false);

  bool show = (mShowSceneTreeButton->property("state") == SHOW);
  if (show) {
    // show scene tree
    if (lastSplitterPosition <= 0)
      lastSplitterPosition = mSceneTree->sizeHint().width();

    const int view3DWidth = mView3D->width();
    if (lastSplitterPosition >= view3DWidth)
      lastSplitterPosition = view3DWidth / 2;

    QList<int> sizes = QList<int>() << lastSplitterPosition << (view3DWidth - lastSplitterPosition);
    mSplitter->setSizes(sizes);

  } else {
    // hide scene tree
    lastSplitterPosition = mSceneTree->width();
    QList<int> sizes = QList<int>() << 0 << (lastSplitterPosition + mView3D->width());
    mSplitter->setSizes(sizes);
    updateToggleView3DAction(true);
  }

  updateSceneTreeActions(show);

  setUpdatesEnabled(true);
}

void WbSimulationView::setView3DVisibility(bool visible) {
  static int lastSplitterPosition = -1;
  const int view3DWidth = mView3D->width();

  if (!visible && (view3DWidth > 0)) {
    // hide view 3D
    lastSplitterPosition = view3DWidth;

    QList<int> sizes = QList<int>() << (lastSplitterPosition + mSceneTree->width()) << 0;
    mSplitter->setSizes(sizes);
    updateToggleView3DAction(false);

  } else if (visible && (view3DWidth == 0)) {
    // show view 3D
    if (lastSplitterPosition <= 0)
      lastSplitterPosition = mView3D->sizeHint().width();

    const int sceneTreeWidth = mSceneTree->width();
    if (lastSplitterPosition >= sceneTreeWidth)
      lastSplitterPosition = sceneTreeWidth / 2;

    QList<int> sizes = QList<int>() << (sceneTreeWidth - lastSplitterPosition) << lastSplitterPosition;
    mSplitter->setSizes(sizes);
    updateToggleView3DAction(true);
  }
}

void WbSimulationView::updateVisibility() {
  const bool isView3DVisible = mToggleView3DAction->isChecked();
  const bool isSceneTreeVisible = mToggleSceneTreeAction->isChecked();
  const bool isSimulationViewVisible = isView3DVisible || isSceneTreeVisible;

  if (isSimulationViewVisible) {
    setView3DVisibility(isView3DVisible);
    if (isSceneTreeVisible != isSceneTreeButtonStatusVisible())
      toggleSceneTreeVisibility();
  }

  const bool show = isSimulationViewVisible && !isVisible();
  QSize tempMinimumSize;
  if (show) {
    // hack to restore simulation view size
    tempMinimumSize = minimumSize();
    setMinimumSize(mLastSize);
  }

  setVisible(isSimulationViewVisible);

  if (show)
    // restore minimum size
    setMinimumSize(tempMinimumSize);
}

void WbSimulationView::unmuteSound() {
  WbPreferences::instance()->setValue("Sound/mute", false);
  const WbSimulationState::Mode mode = WbSimulationState::instance()->mode();
  if (mode != WbSimulationState::RUN && mode != WbSimulationState::FAST)
    WbSoundEngine::setMute(false);
  mSoundVolumeSlider->setSliderPosition(WbPreferences::instance()->value("Sound/volume", 80).toInt());
  connect(mSoundVolumeSlider, &QSlider::valueChanged, this, &WbSimulationView::updateSoundVolume);
  updateSoundButtons();
}

void WbSimulationView::muteSound() {
  WbPreferences::instance()->setValue("Sound/mute", true);
  disconnect(mSoundVolumeSlider, &QSlider::valueChanged, this, &WbSimulationView::updateSoundVolume);
  mSoundVolumeSlider->setSliderPosition(0);
  WbSoundEngine::setMute(true);
  updateSoundButtons();
}

void WbSimulationView::updateSoundVolume(int volume) {
  WbSoundEngine::setVolume(volume);
  WbPreferences::instance()->setValue("Sound/volume", volume);
}

void WbSimulationView::hideInappropriateToolBarItems() {
  foreach (QAction *const action, mToolBarExtensionMenu->actions()) {
    // widgets that aren't de facto menu actions (speedometer and volume slider)
    // have blank action text and aren't parented by the toolbar. We need to check
    // the parent as menu separators have blank text but are always parented by the
    // QToolBar instance
    if (action->text().isEmpty() && action->parentWidget() != mToolBar)
      action->setVisible(false);
  }
}

void WbSimulationView::toggleMovieAction(bool isRecording) {
  if (isRecording) {
    mMovieAction->setText(tr("Stop &Movie..."));
    mMovieAction->setStatusTip(tr("Stop video recording."));
    mMovieAction->setIcon(QIcon("enabledIcons:movie_red_button.png"));
    disconnect(mMovieAction, &QAction::triggered, this, &WbSimulationView::makeMovie);
    connect(mMovieAction, &QAction::triggered, this, &WbSimulationView::stopMovie);
  } else {
    mMovieAction->setText(tr("Make &Movie..."));
    mMovieAction->setStatusTip(tr("Start video recording of the current simulation."));
    mMovieAction->setIcon(QIcon("enabledIcons:movie_black_button.png"));
    disconnect(mMovieAction, &QAction::triggered, this, &WbSimulationView::stopMovie);
    connect(mMovieAction, &QAction::triggered, this, &WbSimulationView::makeMovie);
  }

  mMovieAction->setToolTip(mMovieAction->statusTip());
}

void WbSimulationView::toggleRecordingIcon() {
  static bool isRecOn = false;

  if (!isRecOn) {
    mMovieAction->setIcon(QIcon("enabledIcons:movie_red_button.png"));
    isRecOn = true;
  } else {
    mMovieAction->setIcon(QIcon("enabledIcons:movie_black_button.png"));
    isRecOn = false;
  }
}

void WbSimulationView::startVideoCapture(const QString &fileName, int codec, int width, int height, int quality,
                                         int acceleration, bool showCaption) {
  WbVideoRecorder *videoRecorder = WbVideoRecorder::instance();
  const bool success = videoRecorder->initRecording(this, WbWorld::instance()->worldInfo()->basicTimeStep(),
                                                    QSize(width, height), quality, codec, acceleration, showCaption, fileName);
  if (success) {
    mSupervisorMovieRecordingEnabled = true;
    mRecordingTimer->start(800);
    toggleMovieAction(true);
    mTakeScreenshotAction->setEnabled(false);
    switchToRunModeIfNecessary();
    WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
    if (mainWindow->isMinimized()) {
      mWasMinimized = true;
      mainWindow->showMaximized();
    }
  }
}

void WbSimulationView::stopVideoCapture(bool canceled) {
  WbVideoRecorder::instance()->stopRecording(canceled);
  restoreFastModeIfNecessary();
  if (mWasMinimized) {
    WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
    mainWindow->showMinimized();
    mWasMinimized = false;
  }
  // re-enable take screenshot action
  mTakeScreenshotAction->setEnabled(true);
  mRecordingTimer->stop();
  toggleMovieAction(false);
  mSupervisorMovieRecordingEnabled = false;
}

void WbSimulationView::cancelSupervisorMovieRecording() {
  if (mSupervisorMovieRecordingEnabled) {
    mView3D->resetScreenshotRequest();
    stopVideoCapture(true);
  }
}

void WbSimulationView::stopMovie() {
  stopVideoCapture();
  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
  mainWindow->restorePerspective(false, false, true);
}

void WbSimulationView::makeMovie() {
  if (WbSimulationState::instance()->isFast()) {
    WbLog::warning(tr("Impossible to record a movie while running the simulation in 'Fast' mode."), true);
    return;
  }

  // pause simulation before recording video
  WbSimulationState::Mode currentMode = WbSimulationState::instance()->mode();
  if (!WbSimulationState::instance()->isPaused())
    pause();

  // store our perspective for when we stop
  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
  mainWindow->savePerspective(false, false);

  WbVideoRecorder *videoRecorder = WbVideoRecorder::instance();
  bool success = videoRecorder->initRecording(this, WbWorld::instance()->worldInfo()->basicTimeStep());
  if (success) {
    mRecordingTimer->start(800);
    toggleMovieAction(true);
    // disable take screenshot action
    mTakeScreenshotAction->setEnabled(false);
  }

  // reset current simulation mode
  WbSimulationState::instance()->setMode(currentMode);
  updateFastModeOverlay();
}

void WbSimulationView::switchToRunModeIfNecessary() {
  // remove "Fast Mode" overlay if necessary
  if (WbSimulationState::instance()->isFast()) {
    WbSimulationState::instance()->setMode(WbSimulationState::RUN);
    mView3D->hideFastModeOverlay();
    mNeedRestoreFastMode = true;
  }
}

void WbSimulationView::restoreFastModeIfNecessary() {
  if (mNeedRestoreFastMode) {
    mView3D->showFastModeOverlay();
    WbSimulationState::instance()->setMode(WbSimulationState::FAST);
    mNeedRestoreFastMode = false;
  }
}

void WbSimulationView::writeScreenshot(QImage image) {
  disconnect(mView3D, &WbView3D::screenshotReady, this, &WbSimulationView::writeScreenshot);

  QString filename;
  while (!mScreenshotFileNameList.isEmpty() && !mScreenshotQualityList.isEmpty()) {
    filename = mScreenshotFileNameList.takeFirst();
    if (!image.save(filename, 0, mScreenshotQualityList.takeFirst()))
      WbLog::error(QString("Error while writing file: %1").arg(filename));
    else if (mIsScreenshotRequestedFromGui && mIsDecorationVisible)
      emit requestOpenUrl(filename, tr("The screenshot has been created:\n%1\n\nDo you want to open it now?").arg(filename),
                          tr("Take Screenshot"));
  }

  if (mIsScreenshotRequestedFromGui) {
    WbSimulationState::instance()->resumeSimulation();
    mIsScreenshotRequestedFromGui = false;
  }
  restoreFastModeIfNecessary();

  if (mWasMinimized) {
    WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
    mainWindow->showMinimized();
    mWasMinimized = false;
  }
}

void WbSimulationView::takeScreenshotAndSaveAs(const QString &fileName, int quality) {
  mScreenshotQualityList.append(quality);
  mScreenshotFileNameList.append(fileName);
  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
  if (mainWindow->isMinimized()) {
    mWasMinimized = true;
    mainWindow->showMaximized();
  }
  // In fullscreen mode we don't have a handy dialog to delay things for us so
  // we must ensure the OpenGL context is correct, delaying the screenshot like
  // we do for movies. We can only ask for a screenshot if the view3D is definitely
  // ready.
  if (WbSimulationState::instance()->isPaused() && mIsDecorationVisible) {
    writeScreenshot(mView3D->grabWindowBufferNow());
    return;
  }
  connect(mView3D, &WbView3D::screenshotReady, this, &WbSimulationView::writeScreenshot);
  switchToRunModeIfNecessary();
  mView3D->requestScreenshot();

  if (mIsScreenshotRequestedFromGui) {
    WbSimulationState::instance()->resumeSimulation();
    repaintView3D();
  }
}

void WbSimulationView::takeScreenshot() {
  WbSimulationState *simulationState = WbSimulationState::instance();
  simulationState->pauseSimulation();

  static QStringList winFilters = QStringList() << tr("PNG (*.png)") << tr("JPEG (*.jpg *.jpeg)");
  static QString otherFilters = tr("Images (*.png *.jpg *.jpeg)");
  static QString dialogCaption = tr("Save as...");

  QFileInfo fi(WbWorld::instance()->fileName());
  QString worldBaseName = fi.baseName();

  QString fileName;
  for (int i = 0; i < 1000; ++i) {
    QString suffix = i == 0 ? "" : QString("_%1").arg(i);
    fileName = WbPreferences::instance()->value("Directories/screenshots").toString() + worldBaseName + suffix + ".png";
    if (!QFileInfo::exists(fileName))
      break;
  }

  // show "save as" dialog when not in full-screen mode
  if (mIsDecorationVisible)
    fileName = QFileDialog::getSaveFileName(this, dialogCaption, WbProject::computeBestPathForSaveAs(fileName), otherFilters);

  if (fileName.isEmpty()) {
    simulationState->resumeSimulation();
    return;
  }

  WbPreferences::instance()->setValue("Directories/screenshots", QFileInfo(fileName).absolutePath() + "/");

  QFileInfo file(fileName);
  QString suffix = file.suffix();
  if (suffix.toLower() == "png" || suffix.toLower() == "jpg" || suffix.toLower() == "jpeg") {
    mIsScreenshotRequestedFromGui = true;
    takeScreenshotAndSaveAs(fileName);
    return;
  }

  if (suffix.isEmpty())
    WbMessageBox::warning(tr("Unable to save screenshot because the file format is missing. "
                             "Please use the '.png', '.jpg' or '.jpeg' file extension to set the file format.")
                            .arg(suffix),
                          this, tr("Missing file format"));
  else
    WbMessageBox::warning(tr("Unable to save screenshot because the '.%1' format is unsupported. "
                             "Please use only the '.png', '.jpg' or '.jpeg' file extension.")
                            .arg(suffix),
                          this, tr("Unsupported file format"));

  simulationState->resumeSimulation();
}

void WbSimulationView::pause() {
  repaintView3D();  // update 3D view if not refreshed after last step
  WbSimulationState::instance()->setMode(WbSimulationState::PAUSE);
}

void WbSimulationView::step() {
  WbSimulationState::instance()->setMode(WbSimulationState::STEP);
  WbSimulationWorld::instance()->step();
  WbSimulationState::instance()->setMode(WbSimulationState::PAUSE);
}

void WbSimulationView::realTime() {
  WbSimulationState::instance()->setMode(WbSimulationState::REALTIME);
}

void WbSimulationView::run() {
  WbSimulationState::instance()->setMode(WbSimulationState::RUN);
}

void WbSimulationView::fast() {
  WbSimulationState::instance()->setMode(WbSimulationState::FAST);
}

void WbSimulationView::updateFastModeOverlay() {
  if (WbSimulationState::instance()->mode() == WbSimulationState::FAST)
    renderABlackScreen();
  else
    retrieveSimulationView();
}

void WbSimulationView::prepareWorldLoading() {
  mSceneTree->prepareWorldLoading();
  mView3D->prepareWorldLoading();

  disconnect(mSceneTree, &WbSceneTree::valueChangedFromGui, mView3D, &WbView3D::renderLater);

  // solid selection
  disconnect(mSelection, &WbSelection::visibleHandlesChanged, mView3D, &WbView3D::renderLater);
  disconnect(mSelection, &WbSelection::selectionChangedFromSceneTree, mView3D, &WbView3D::renderLater);
  disconnect(mSelection, &WbSelection::selectionChangedFromView3D, mSceneTree, &WbSceneTree::selectTransform);
  disconnect(mSelection, &WbSelection::selectionConfirmedFromView3D, mSceneTree, &WbSceneTree::selectTransform);
  disconnect(mSceneTree, &WbSceneTree::nodeSelected, mSelection, &WbSelection::selectNodeFromSceneTree);
  if (!WbSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty()) {
    disconnect(mSceneTree, &WbSceneTree::nodeSelected, WbVisualBoundingSphere::instance(), &WbVisualBoundingSphere::show);
    WbVisualBoundingSphere::instance()->show(NULL);
  }
}

void WbSimulationView::setWorld(WbSimulationWorld *w) {
  // first set world in mView3D and then in mSceneTree
  // otherwise mView3D receives a selection changed signal from mSceneTree when no
  // world is load and the selection in the two views will mismatch
  mView3D->setWorld(w);
  mSceneTree->setWorld(w);
  updateTitleBarTitle();
  connect(w->worldInfo(), &WbWorldInfo::titleChanged, this, &WbSimulationView::updateTitleBarTitle);
  connect(w, &WbSimulationWorld::physicsStepEnded, WbSelection::instance(),
          &WbSelection::propagateBoundingObjectMaterialUpdate);
  WbControlledWorld *cw = dynamic_cast<WbControlledWorld *>(w);
  assert(cw);
  connect(cw, &WbControlledWorld::stepBlocked, this, &WbSimulationView::disableStepButton);

  // update save action based on simulation world state
  WbActionManager::instance()->setEnabled(WbActionManager::SAVE_WORLD, false);
  connect(w, &WbWorld::modificationChanged, WbActionManager::instance()->action(WbActionManager::SAVE_WORLD),
          &QAction::setEnabled);
  connect(w, &WbSimulationWorld::simulationStartedAfterSave, WbActionManager::instance()->action(WbActionManager::SAVE_WORLD),
          &QAction::setEnabled);

  connect(mSceneTree, &WbSceneTree::valueChangedFromGui, mView3D, &WbView3D::renderLater);

  // solid selection
  connect(mSelection, &WbSelection::visibleHandlesChanged, mView3D, &WbView3D::renderLater);
  connect(mSelection, &WbSelection::selectionChangedFromSceneTree, mView3D, &WbView3D::renderLater);
  connect(mSelection, &WbSelection::selectionChangedFromView3D, mSceneTree, &WbSceneTree::selectTransform);
  connect(mSelection, &WbSelection::selectionConfirmedFromView3D, mSceneTree, &WbSceneTree::selectTransform);
  connect(mSceneTree, &WbSceneTree::nodeSelected, mSelection, &WbSelection::selectNodeFromSceneTree);
  if (!WbSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty())
    connect(mSceneTree, &WbSceneTree::nodeSelected, WbVisualBoundingSphere::instance(), &WbVisualBoundingSphere::show);

  connect(WbActionManager::instance()->action(WbActionManager::FRONT_VIEW), &QAction::triggered,
          WbSimulationWorld::instance()->viewpoint(), &WbViewpoint::frontView);
  connect(WbActionManager::instance()->action(WbActionManager::BACK_VIEW), &QAction::triggered,
          WbSimulationWorld::instance()->viewpoint(), &WbViewpoint::backView);
  connect(WbActionManager::instance()->action(WbActionManager::LEFT_VIEW), &QAction::triggered,
          WbSimulationWorld::instance()->viewpoint(), &WbViewpoint::leftView);
  connect(WbActionManager::instance()->action(WbActionManager::RIGHT_VIEW), &QAction::triggered,
          WbSimulationWorld::instance()->viewpoint(), &WbViewpoint::rightView);
  connect(WbActionManager::instance()->action(WbActionManager::TOP_VIEW), &QAction::triggered,
          WbSimulationWorld::instance()->viewpoint(), &WbViewpoint::topView);
  connect(WbActionManager::instance()->action(WbActionManager::BOTTOM_VIEW), &QAction::triggered,
          WbSimulationWorld::instance()->viewpoint(), &WbViewpoint::bottomView);

  mSceneTree->updateSelection();
  disableStepButton(false);
}

// update title bar's title
void WbSimulationView::updateTitleBarTitle() {
  WbWorld *world = WbSimulationWorld::instance();
  const QString &title = world->worldInfo()->title();
  if (title.isEmpty())
    mTitleBar->setTitle(tr("Simulation View"));
  else
    mTitleBar->setTitle(title);
}

void WbSimulationView::repaintView3D() {
  mView3D->renderLater();
}

void WbSimulationView::renderABlackScreen() {
  if (mView3D)
    mView3D->showFastModeOverlay();
}

void WbSimulationView::retrieveSimulationView() {
  if (mView3D)
    mView3D->hideFastModeOverlay();
}

void WbSimulationView::modeKeyPressed(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_0:
      // Ctrl + 0
      pause();
      return;
    case Qt::Key_1:
      // Ctrl + 1
      step();
      return;
    case Qt::Key_2:
      // Ctrl + 2
      realTime();
      return;
    case Qt::Key_3:
      // Ctrl + 3
      run();
      return;
    case Qt::Key_4:
      // Ctrl + 4
      fast();
      return;
    default:
      break;
  }
}

void WbSimulationView::disableStepButton(bool disabled) {
  WbActionManager::instance()->action(WbActionManager::STEP)->setEnabled(!disabled);
}

void WbSimulationView::updatePlayButtons() {
  mToolBar->setUpdatesEnabled(false);

  WbActionManager *manager = WbActionManager::instance();

  QAction *pause = manager->action(WbActionManager::PAUSE);
  QAction *realtime = manager->action(WbActionManager::REAL_TIME);
  QAction *run = manager->action(WbActionManager::RUN);
  QAction *fast = manager->action(WbActionManager::FAST);

  mToolBar->removeAction(pause);
  mToolBar->removeAction(realtime);
  mToolBar->removeAction(run);
  mToolBar->removeAction(fast);

  QList<QAction *> actions;

  switch (WbSimulationState::instance()->mode()) {
    case WbSimulationState::REALTIME:
      actions << pause << run << fast;
      break;

    case WbSimulationState::RUN:
      actions << realtime << pause << fast;
      break;

    case WbSimulationState::FAST:
      actions << realtime << run << pause;
      break;

    default:  // PAUSE
      actions << realtime << run << fast;
      break;
  }

  mToolBar->insertActions(mPlayAnchor, actions);

  // setObjectName (used by the stylesheet)
  QWidget *pauseWidget = mToolBar->widgetForAction(pause);
  QWidget *realTimeWidget = mToolBar->widgetForAction(realtime);
  QWidget *runWidget = mToolBar->widgetForAction(run);
  QWidget *fastWidget = mToolBar->widgetForAction(fast);
  if (runWidget)
    runWidget->setObjectName("menuButton");
  if (fastWidget)
    fastWidget->setObjectName("menuButton");
  if (realTimeWidget)
    realTimeWidget->setObjectName("menuButton");
  if (pauseWidget)
    pauseWidget->setObjectName("menuButton");

  mToolBar->update();

  mToolBar->setUpdatesEnabled(true);

  updateFastModeOverlay();
}

void WbSimulationView::updateSoundButtons() {
  mToolBar->setUpdatesEnabled(false);

  WbActionManager *manager = WbActionManager::instance();

  QAction *soundUnmuteAction = manager->action(WbActionManager::SOUND_UNMUTE);
  QAction *soundMuteAction = manager->action(WbActionManager::SOUND_MUTE);

  mToolBar->removeAction(soundUnmuteAction);
  mToolBar->removeAction(soundMuteAction);

  bool mute = WbPreferences::instance()->value("Sound/mute", true).toBool();
  mSoundVolumeSlider->setEnabled(!mute);
  mToolBar->insertAction(mSoundAnchor, mute ? soundUnmuteAction : soundMuteAction);

  // setObjectName (used by the stylesheet)
  QWidget *soundEnableWidget = mToolBar->widgetForAction(soundUnmuteAction);
  QWidget *soundDisableWidget = mToolBar->widgetForAction(soundMuteAction);
  if (soundEnableWidget)
    soundEnableWidget->setObjectName("menuButton");
  if (soundDisableWidget)
    soundDisableWidget->setObjectName("menuButton");

  mToolBar->setUpdatesEnabled(true);

  mToolBar->update();
}

QList<QByteArray> WbSimulationView::saveState() const {
  QList<QByteArray> state;
  state << mSplitter->saveState() << mSceneTree->saveState();
  return state;
}

void WbSimulationView::restoreState(QList<QByteArray> state, bool firstLoad) {
  assert(state.size() == 2);

  if (!state[0].isEmpty()) {
    mSplitter->restoreState(state[0]);
    mSplitter->setHandleWidth(mHandleWidth);
  } else if (firstLoad)
    restoreFactoryLayout();

  if (!state[1].isEmpty())
    mSceneTree->restoreState(state[1]);

  updateSceneTreeActions(isVisible() && mSplitter->sizes()[0] > 0);
  updateToggleView3DAction(isVisible() && mView3D->width() > 0);
}

void WbSimulationView::restoreFactoryLayout() {
  const int halfSplitterWidth = mSplitter->width() * 0.5;
  int preferredSceneTreeWidth = mSceneTree->sizeHint().width();
  if (preferredSceneTreeWidth > halfSplitterWidth)
    // default scene tree width should never be bigger than 3D view width
    preferredSceneTreeWidth = halfSplitterWidth;

  QList<int> sizes = QList<int>() << preferredSceneTreeWidth << (mSplitter->width() - preferredSceneTreeWidth);
  mSplitter->setSizes(sizes);
  mSplitter->setHandleWidth(mHandleWidth);
  mSceneTree->restoreFactoryLayout();
  updateSceneTreeActions(isVisible() && mSceneTree->width() > 0);
  updateToggleView3DAction(isVisible() && mView3D->width() > 0);
}

void WbSimulationView::setView3DResizeStretch(bool isSizeFixed) {
  const int view3DStretch = isSizeFixed ? 0 : 1;
  const int emptyCellStretch = isSizeFixed ? 1 : 0;
  mView3DResizeLayout->setColumnStretch(0, view3DStretch);
  mView3DResizeLayout->setColumnStretch(1, emptyCellStretch);
  mView3DResizeLayout->setRowStretch(0, view3DStretch);
  mView3DResizeLayout->setRowStretch(1, emptyCellStretch);
}

void WbSimulationView::enableView3DFixedSize(const QSize &size) {
  setView3DResizeStretch(true);
  mView3DContainer->setMinimumSize(size);
  mView3DContainer->setMaximumSize(size);
  // manually update the WREN render window size
  // on Jenkins machine the resize event is processed too late after the first
  // scene screenshot has already been taken
  mView3D->resizeWren(size.width(), size.height());
}

void WbSimulationView::disableView3DFixedSize() {
  setView3DResizeStretch(false);
  mView3DContainer->setMinimumSize(0, 0);
  mView3DContainer->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
}

void WbSimulationView::applyChanges() {
  mSceneTree->applyChanges();
}

void WbSimulationView::cleanup() {
  mView3D->cleanupEvents();
  mSceneTree->cleanup();
}

void WbSimulationView::internalScreenChangedCallback() {
  mView3D->updateWrenViewportDimensions();
}

WbRobot *WbSimulationView::selectedRobot() const {
  WbBaseNode *selectedNode = mSelection->selectedNode();
  if (selectedNode) {
    WbRobot *robot = dynamic_cast<WbRobot *>(selectedNode);
    if (!robot)
      robot = WbNodeUtilities::findRobotAncestor(selectedNode);
    if (robot)
      return robot;
  }

  if (WbWorld::instance()) {
    const QList<WbRobot *> &robotList = WbWorld::instance()->robots();
    if (robotList.size() == 1)
      return robotList.first();
  }

  return NULL;
}

void WbSimulationView::keyPressEvent(QKeyEvent *event) {
  mView3D->handleModifierKey(event, true);
}

void WbSimulationView::keyReleaseEvent(QKeyEvent *event) {
  mView3D->handleModifierKey(event, false);
}

void WbSimulationView::hideEvent(QHideEvent *event) {
  mLastSize = size();
  mSplitterStatus = mToggleSceneTreeAction->isChecked() ? SCENE_TREE_VISIBLE : 0;
  mSplitterStatus = mSplitterStatus | (mToggleView3DAction->isChecked() ? VIEW_3D_VISIBLE : 0);
  mToggleSceneTreeAction->blockSignals(true);
  mToggleSceneTreeAction->setChecked(false);
  mToggleSceneTreeAction->blockSignals(false);
  updateToggleView3DAction(false);
}

void WbSimulationView::showEvent(QShowEvent *event) {
  if (mToggleSceneTreeAction->isChecked() || mToggleView3DAction->isChecked())
    return;

  // show after minimize dock event
  mToggleSceneTreeAction->blockSignals(true);
  mToggleSceneTreeAction->setChecked(mSplitterStatus & SCENE_TREE_VISIBLE);
  mToggleSceneTreeAction->blockSignals(false);
  updateToggleView3DAction(mSplitterStatus & VIEW_3D_VISIBLE);
}

void WbSimulationView::showMenu(const QPoint &position) {
  const WbBaseNode *selectedNode = WbSelection::instance() ? WbSelection::instance()->selectedNode() : NULL;
  WbContextMenuGenerator::generateContextMenu(position, selectedNode);
}
