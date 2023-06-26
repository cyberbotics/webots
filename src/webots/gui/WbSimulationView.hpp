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

#ifndef WB_SIMULATION_VIEW_HPP
#define WB_SIMULATION_VIEW_HPP

//
// Description: central widget of WbMainWindow, contains all the actions for managing the simulation state
//

#include <QtWidgets/QWidget>

class WbDockTitleBar;
class WbRobot;
class WbSceneTree;
class WbSelection;
class WbSimulationWorld;
class WbView3D;

class QAction;
class QGridLayout;
class QHBoxLayout;
class QLabel;
class QMenu;
class QSlider;
class QSplitter;
class QTimer;
class QToolBar;
class QToolButton;

class WbSimulationView : public QWidget {
  Q_OBJECT
  Q_PROPERTY(int handleWidth MEMBER mHandleWidth READ handleWidth WRITE setHandleWidth)

public:
  WbSimulationView(QWidget *parent, const QString &toolBarAlign);
  virtual ~WbSimulationView();

  void prepareWorldLoading();
  void setWorld(WbSimulationWorld *w);

  WbView3D *view3D() const { return mView3D; }
  const WbSceneTree *sceneTree() const { return mSceneTree; }

  const WbSelection *selection() const { return mSelection; }
  WbRobot *selectedRobot() const;

  QAction *toggleView3DAction() const { return mToggleView3DAction; }
  QAction *toggleSceneTreeAction() const { return mToggleSceneTreeAction; }
  QAction *movieAction() const { return mMovieAction; }
  void cancelSupervisorMovieRecording();

  void modeKeyPressed(QKeyEvent *event);

  // just change the icons appearance
  void setMaximized(bool maximized);

  void setDecorationVisible(bool visible);

  int &handleWidth() { return mHandleWidth; }
  void setHandleWidth(const int &handleWidth) { mHandleWidth = handleWidth; }

  // save/restore splitter perspective
  QList<QByteArray> saveState() const;
  void restoreState(QList<QByteArray> state, bool firstLoad = false);
  void restoreFactoryLayout();

  void enableView3DFixedSize(const QSize &size);
  void disableView3DFixedSize();
  void repaintView3D();

  void cleanup();
  void applyChanges();
  QSize mLastSize;

  void internalScreenChangedCallback();

signals:
  // signals called when the corresponding toolbar buttons are pushed
  void needsMaximize();
  void needsMinimize();
  void requestOpenUrl(const QString &fileName, const QString &message, const QString &title);

  // signals for screenshots and thumbnails
  void screenshotWritten();
  void thumbnailTaken();

public slots:
  void disableRendering(bool disabled);
  void disableStepButton(bool disabled);
  void takeThumbnail(const QString &fileName);

protected slots:
  void keyReleaseEvent(QKeyEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void showMenu(const QPoint &position, QWidget *parentWidget);

private slots:
  void makeMovie();
  void stopMovie();
  void toggleRecordingIcon();
  void startVideoCapture(const QString &fileName, int codec, int width, int height, int quality, int acceleration,
                         bool showCaption);
  void stopVideoCapture(bool canceled = false);
  void takeScreenshotAndSaveAs(const QString &fileName, int quality = -1);
  void takeScreenshot();
  void takeScreesnhotForThumbnail();
  void restoreViewAfterThumbnail();
  void pause();
  void step();
  void realTime();
  void fast();
  void toggleRendering();
  void updateVisibility();
  void writeScreenshot();
  void writeScreenshotForThumbnail();
  void updateTitleBarTitle();
  void updatePlayButtons();
  void updateRendering();
  void updateSoundButtons();
  void needsActionsUpdate(int position, int index);
  void toggleSceneTreeVisibility();
  void unmuteSound();
  void muteSound();
  void updateSoundVolume(int volume);
  void hideInappropriateToolBarItems();

private:
  bool mWasMinimized;
  bool mIsScreenshotRequestedFromGui;
  bool mIsDecorationVisible;
  QAction *mPlayAnchor, *mSoundAnchor;
  QMenu *mToolBarExtensionMenu;

  WbSelection *mSelection;

  WbDockTitleBar *mTitleBar;
  QSplitter *mSplitter;
  WbSceneTree *mSceneTree;
  WbView3D *mView3D;
  QWidget *mView3DContainer;
  QGridLayout *mView3DResizeLayout;
  enum SplitterStatus { SCENE_TREE_VISIBLE = 0x01, VIEW_3D_VISIBLE = 0x02 };
  int mSplitterStatus;
  int mHandleWidth;

  QToolBar *mToolBar;
  QHBoxLayout *mToolsLayout;
  QLabel *mMessageLabel;
  QSlider *mSoundVolumeSlider;
  bool mNeedToRestoreBlackRenderingOverlay;
  bool mNeedToRestoreRendering;

  QAction *mToggleView3DAction, *mToggleSceneTreeAction;
  QToolButton *mShowSceneTreeButton;
  QAction *mMovieAction, *mTakeScreenshotAction;

  QTimer *mRecordingTimer;
  bool mSupervisorMovieRecordingEnabled;

  QList<int> mScreenshotQualityList;
  QStringList mScreenshotFileNameList;

  QSize mSizeBeforeThumbnail;
  QString mThumbnailFileName;

  void createActions();
  QToolBar *createToolBar();
  void updateBlackRenderingOverlay();
  void renderABlackScreen();
  void retrieveSimulationView();
  void showRenderingIfNecessary();
  void restoreNoRenderingIfNecessary();
  void toggleMovieAction(bool isRecording);
  void updateSceneTreeActions(bool enabled);
  void updateToggleView3DAction(bool enabled);
  void setView3DVisibility(bool visible);
  void setView3DResizeStretch(bool isSizeFixed);
  bool isSceneTreeButtonStatusVisible() const;
};

#endif
