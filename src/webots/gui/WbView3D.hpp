// Copyright 1996-2019 Cyberbotics Ltd.
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

#ifndef WB_VIEW_3D_HPP
#define WB_VIEW_3D_HPP

//
// Description: 3D window for displaying the scene and for switching the display mode
//

#include "WbWrenWindow.hpp"

#include <wren/camera.h>
#include <wren/viewport.h>

#include <QtCore/QElapsedTimer>
#include <QtCore/QPoint>

class WbAbstractTransform;
class WbCamera;
class WbDragKinematicsEvent;
class WbDragForceEvent;
class WbDragScaleHandleEvent;
class WbDragTorqueEvent;
class WbDragOverlayEvent;
class WbDragResizeHandleEvent;
class WbDragRotateAroundWorldVerticalAxisEvent;
class WbDragRotateAroundAxisEvent;
class WbDragTranslateAlongAxisEvent;
class WbMatter;
class WbWrenRenderingContext;
class WbContactPointsRepresentation;
class WbRobot;
class WbSolid;
class WbSimulationWorld;
class WbTouchSensor;
class WbWheelEvent;
class WbWrenPicker;
class WbWrenFullScreenOverlay;

class WbView3D : public WbWrenWindow {
  Q_OBJECT;

public:
  explicit WbView3D();
  virtual ~WbView3D();

  void setParentWidget(QWidget *widget) { mParentWidget = widget; }

  // accessor
  WbWrenRenderingContext *wrenRenderingContext() const { return mWrenRenderingContext; }
  // rendering
  void showFastModeOverlay();
  void hideFastModeOverlay();
  void remoteMouseEvent(QMouseEvent *event);
  void remoteWheelEvent(QWheelEvent *event);

  void prepareWorldLoading();
  void setWorld(WbSimulationWorld *w);
  void requestScreenshot() { mScreenshotRequested = true; }
  void resetScreenshotRequest() { mScreenshotRequested = false; }
  void cleanupEvents();
  void cleanupOptionalRendering();
  void cleanupFullScreenOverlay();
  void updateVirtualRealityHeadsetOverlay();
  void restoreOptionalRendering(const QStringList &enabledCenterOfMassNodeNames,
                                const QStringList &enabledCenterOfBuoyancyNodeNames,
                                const QStringList &enabledSupportPolygonNodeNames) const;

  void enableResizeManipulator(bool enabled);
  void resizeWren(int width, int height) override;

  void logWrenStatistics() const;
  void handleModifierKey(QKeyEvent *event, bool pressed);

public slots:
  void refresh();
  void setShowRenderingDevice(bool checked);
  void unleashAndClean();

protected slots:
  void renderNow(bool culling = true) override;

protected:
  void initialize() override;

  virtual void leaveEvent(QEvent *event);
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  // replaces QWidget::mouseDoubleClickEvent() which is never called under Windows (Qt bug ?)
  virtual void mouseDoubleClick(QMouseEvent *event);
  void keyReleaseEvent(QKeyEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void focusInEvent(QFocusEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;

signals:
  void mainRenderingStarted(bool fromPhysics);
  void mainRenderingEnded(bool fromPhysics);
  void mouseDoubleClicked(QMouseEvent *event);
  void screenshotReady(QImage image);
  void showRobotWindowRequest();
  void applicationActionsUpdateRequested();
  void contextMenuRequested(const QPoint &pos);

private:
  QWidget *mParentWidget;
  QElapsedTimer mLastRefreshTimer;
  static int cView3DNumber;
  WrCameraProjectionMode mProjectionMode;
  WrViewportPolygonMode mRenderingMode;
  int mRefreshCounter;
  QTime *mMousePressTime;
  QPoint mMousePressPosition;
  bool mSelectionDisabled;
  bool mViewpointLocked;
  double mAspectRatio;
  WbWrenFullScreenOverlay *mFastModeOverlay;
  WbWrenFullScreenOverlay *mLoadingWorldOverlay;
  WbWrenFullScreenOverlay *mVirtualRealityHeadsetOverlay;

  WbContactPointsRepresentation *mContactPointsRepresentation;
  WbWrenRenderingContext *mWrenRenderingContext;

  // Cleanup
  void cleanupDrags();
  void cleanupPhysicsDrags();
  void cleanupWheel();
  void cleanupCameraRecognizedObjectsOverlayIfNeeded();
  void cleanupPickers();

  // setters
  void setProjectionMode(WrCameraProjectionMode mode, bool updatePerspective);
  void setRenderingMode(WrViewportPolygonMode mode, bool updatePerspective);

  // Others
  WrViewportPolygonMode stringToRenderingMode(const QString &s);
  WrCameraProjectionMode stringToProjectionMode(const QString &s);
  void rescaleFastModePanel();
  void enableOptionalRenderingFromPerspective();
  WbRobot *getCurrentRobot() const;
  void checkRendererCapabilities();
  static bool isContextMenuShortcut(const QMouseEvent *event);
  void selectNode(const QMouseEvent *event);

  bool mPhysicsRefresh;
  bool mScreenshotRequested;

  WbSimulationWorld *mWorld;
  WbTouchSensor *mTouchSensor;                     // touch sensor pressed by the mouse pointer if any
  WbCamera *mCameraUsingRecognizedObjectsOverlay;  // camera using the recognized object overlay if any

  // Drags
  WbDragForceEvent *mDragForce;
  WbDragTorqueEvent *mDragTorque;
  WbDragKinematicsEvent *mDragKinematics;
  WbDragOverlayEvent *mDragOverlay;
  WbDragResizeHandleEvent *mDragResize;
  WbDragScaleHandleEvent *mDragScale;
  WbDragTranslateAlongAxisEvent *mDragTranslate;
  WbDragRotateAroundWorldVerticalAxisEvent *mDragVerticalAxisRotate;
  WbDragRotateAroundAxisEvent *mDragRotate;
  bool mResizeHandlesDisabled;

  // Pickers
  WbWrenPicker *mPicker;
  WbWrenPicker *mControllerPicker;
  WbMatter *mPickedMatter;
  WbWheelEvent *mWheel;

  bool mMouseEventInitialized;
  QCursor mLastMouseCursor;
  Qt::MouseButtons mLastButtonState;

  // On selection changed
  void setCheckedShowCenterOfMassAction(WbSolid *selectedSolid);
  void setCheckedShowCenterOfBuoyancyAction(WbSolid *selectedSolid);
  void setCheckedShowSupportPolygonAction(WbSolid *selectedSolid);
  void setCheckedFollowObjectAction(WbSolid *selectedSolid);

private slots:
  void abortPhysicsDrag();
  void abortResizeDrag();
  void abortScaleDrag();
  void abortOverlayDrag();
  void followNone(bool checked);
  void followTracking(bool checked);
  void followMounted(bool checked);
  void followPanAndTilt(bool checked);
  void showCenterOfMass(bool checked);
  void showCenterOfBuoyancy(bool checked);
  void showSupportPolygon(bool checked);
  void notifyFollowObjectAction(int type);
  void restoreViewpoint();
  void setPerspectiveProjection();
  void setOrthographicProjection();
  void setPlain();
  void setWireframe();
  void setVirtualRealityHeadset(bool enable);
  void setVirtualRealityHeadsetPositionTracking(bool enable);
  void setVirtualRealityHeadsetOrientationTracking(bool enable);
  void setVirtualRealityHeadsetLeftEyeView(bool enable);
  void setVirtualRealityHeadsetRightEyeView(bool enable);
  void setVirtualRealityHeadsetNoEyeView(bool enable);
  void setVirtualRealityHeadsetAntiAliasing(bool enable);
  void setShowCoordinateSystem(bool show);
  void setShowBoundingObjects(bool show);
  void setShowContactPoints(bool show);
  void setShowConnectorAxes(bool show);
  void setShowJointAxes(bool show);
  void setShowCameraFrustums(bool show);
  void setShowRangeFinderFrustums(bool show);
  void setShowRadarFrustums(bool show);
  void setShowLidarRaysPaths(bool show);
  void setShowLidarPointClouds(bool show);
  void setHideAllCameraOverlays(bool hidden);
  void setHideAllRangeFinderOverlays(bool hidden);
  void setHideAllDisplayOverlays(bool hidden);
  void setShowDistanceSensorRays(bool show);
  void setShowLightSensorRays(bool show);
  void setShowLightsPositions(bool show);
  void setShowPenPaintingRays(bool show);
  void setShowSkeletonAction(bool show);
  void setShowPhysicsClustersAction(bool show);
  void setShowBoundingSphereAction(bool show);
  void setSelectionDisabled(bool disabled);
  void setViewPointLocked(bool locked);
  void updateMousesPosition(bool fromMouseClick = false, bool fromMouseMove = false);

  void cleanWorld() { mWorld = NULL; }
  void updateViewport();
  void updateShadowState();
  void unleashPhysicsDrags();
  void onSelectionChanged(WbAbstractTransform *selectedAbstractTransform);
  void handleWorldModificationFromSupervior();
};

#endif
