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

#include "WbView3D.hpp"

#include "WbAbstractDragEvent.hpp"
#include "WbActionManager.hpp"
#include "WbBox.hpp"
#include "WbCamera.hpp"
#include "WbCapsule.hpp"
#include "WbCone.hpp"
#include "WbContactPointsRepresentation.hpp"
#include "WbCylinder.hpp"
#include "WbDragOverlayEvent.hpp"
#include "WbDragResizeEvent.hpp"
#include "WbDragScaleEvent.hpp"
#include "WbDragSolidEvent.hpp"
#include "WbDragTransformEvent.hpp"
#include "WbDragViewpointEvent.hpp"
#include "WbElevationGrid.hpp"
#include "WbGroup.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbLog.hpp"
#include "WbMatter.hpp"
#include "WbMessageBox.hpp"
#include "WbMouse.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeDebugger.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPerspective.hpp"
#include "WbPlane.hpp"
#include "WbPreferences.hpp"
#include "WbRenderingDevice.hpp"
#include "WbRenderingDeviceWindowFactory.hpp"
#include "WbRobot.hpp"
#include "WbSelection.hpp"
#include "WbSimulationState.hpp"
#include "WbSimulationWorld.hpp"
#include "WbSkin.hpp"
#include "WbSolid.hpp"
#include "WbSphere.hpp"
#include "WbStandardPaths.hpp"
#include "WbSupervisorUtilities.hpp"
#include "WbSysInfo.hpp"
#include "WbTouchSensor.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbVersion.hpp"
#include "WbVideoRecorder.hpp"
#include "WbViewpoint.hpp"
#include "WbVisualBoundingSphere.hpp"
#include "WbWheelEvent.hpp"
#include "WbWorldInfo.hpp"
#include "WbWrenFullScreenOverlay.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPicker.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenTextureOverlay.hpp"

#ifdef _WIN32
#include "WbVirtualRealityHeadset.hpp"
#endif

#include <QtCore/QTime>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QScreen>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>

#include <wren/camera.h>
#include <wren/config.h>
#include <wren/gl_state.h>
#include <wren/scene.h>
#include <wren/transform.h>
#include <wren/viewport.h>

int WbView3D::cView3DNumber = 0;

WbView3D::WbView3D() :
  WbWrenWindow(),
  mParentWidget(NULL),
  mLastRefreshTimer(),
  mRefreshCounter(0),
  mMousePressTime(NULL),
  mSelectionDisabled(false),
  mViewpointLocked(false),
  mAspectRatio(1.0),
  mFastModeOverlay(NULL),
  mLoadingWorldOverlay(NULL),
  mVirtualRealityHeadsetOverlay(NULL),
  mContactPointsRepresentation(NULL),
  mWrenRenderingContext(NULL),
  mPhysicsRefresh(false),
  mScreenshotRequested(false),
  mWorld(NULL),
  mTouchSensor(NULL),
  mCameraUsingRecognizedObjectsOverlay(NULL),
  mDragForce(NULL),
  mDragTorque(NULL),
  mDragKinematics(NULL),
  mDragOverlay(NULL),
  mDragResize(NULL),
  mDragScale(NULL),
  mDragTranslate(NULL),
  mDragVerticalAxisRotate(NULL),
  mDragRotate(NULL),
  mResizeHandlesDisabled(false),
  mPicker(NULL),
  mControllerPicker(NULL),
  mPickedMatter(NULL),
  mWheel(NULL),
  mMouseEventInitialized(false),
  mLastButtonState(Qt::NoButton) {
  QDir::addSearchPath("gl", WbStandardPaths::resourcesPath() + "wren");

  mLastRefreshTimer.start();
  setObjectName("View3D");

  WbWrenRenderingContext::setWrenRenderingContext(width(), height());
  mWrenRenderingContext = WbWrenRenderingContext::instance();

  WbActionManager *actionManager = WbActionManager::instance();
  // render after each simulation step and when simulation mode changed
  connect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this, &WbView3D::refresh,
          Qt::UniqueConnection);
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbView3D::refresh, Qt::UniqueConnection);
  // clean up pending drag-force / drag-torque when simulation restarts
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbView3D::unleashPhysicsDrags);
  // update mouses if required
  connect(WbSimulationState::instance(), SIGNAL(physicsStepStarted()), this, SLOT(updateMousesPosition()));
  // viewpoint
  connect(actionManager->action(WbActionManager::FOLLOW_OBJECT), &QAction::triggered, this, &WbView3D::followSolid);
  connect(actionManager->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE), &QAction::triggered, this,
          &WbView3D::followSolidAndRotate);
  connect(actionManager->action(WbActionManager::RESTORE_VIEWPOINT), &QAction::triggered, this, &WbView3D::restoreViewpoint);
  // signal the simulation state about a rendering
  connect(actionManager->action(WbActionManager::ORTHOGRAPHIC_PROJECTION), &QAction::triggered, this,
          &WbView3D::setOrthographicProjection);
  connect(actionManager->action(WbActionManager::PERSPECTIVE_PROJECTION), &QAction::triggered, this,
          &WbView3D::setPerspectiveProjection);
  connect(actionManager->action(WbActionManager::PLAIN_RENDERING), &QAction::triggered, this, &WbView3D::setPlain);
  connect(actionManager->action(WbActionManager::WIREFRAME_RENDERING), &QAction::triggered, this, &WbView3D::setWireframe);
  connect(actionManager->action(WbActionManager::DISABLE_SELECTION), &QAction::triggered, this,
          &WbView3D::setSelectionDisabled);
  connect(actionManager->action(WbActionManager::LOCK_VIEWPOINT), &QAction::triggered, this, &WbView3D::setViewPointLocked);
  // optional renderings
  connect(actionManager->action(WbActionManager::COORDINATE_SYSTEM), &QAction::toggled, this,
          &WbView3D::setShowCoordinateSystem);
  connect(actionManager->action(WbActionManager::BOUNDING_OBJECT), &QAction::toggled, this, &WbView3D::setShowBoundingObjects);
  connect(actionManager->action(WbActionManager::CONTACT_POINTS), &QAction::toggled, this, &WbView3D::setShowContactPoints);
  connect(actionManager->action(WbActionManager::CONNECTOR_AXES), &QAction::toggled, this, &WbView3D::setShowConnectorAxes);
  connect(actionManager->action(WbActionManager::JOINT_AXES), &QAction::toggled, this, &WbView3D::setShowJointAxes);
  connect(actionManager->action(WbActionManager::RANGE_FINDER_FRUSTUMS), &QAction::toggled, this,
          &WbView3D::setShowRangeFinderFrustums);
  connect(actionManager->action(WbActionManager::LIDAR_RAYS_PATH), &QAction::toggled, this, &WbView3D::setShowLidarRaysPaths);
  connect(actionManager->action(WbActionManager::LIDAR_POINT_CLOUD), &QAction::toggled, this,
          &WbView3D::setShowLidarPointClouds);
  connect(actionManager->action(WbActionManager::CAMERA_FRUSTUM), &QAction::toggled, this, &WbView3D::setShowCameraFrustums);
  connect(actionManager->action(WbActionManager::DISTANCE_SENSOR_RAYS), &QAction::toggled, this,
          &WbView3D::setShowDistanceSensorRays);
  connect(actionManager->action(WbActionManager::LIGHT_SENSOR_RAYS), &QAction::toggled, this,
          &WbView3D::setShowLightSensorRays);
  connect(actionManager->action(WbActionManager::LIGHT_POSITIONS), &QAction::toggled, this, &WbView3D::setShowLightsPositions);
  connect(actionManager->action(WbActionManager::CENTER_OF_BUOYANCY), &QAction::triggered, this,
          &WbView3D::showCenterOfBuoyancy);
  connect(actionManager->action(WbActionManager::PEN_PAINTING_RAYS), &QAction::toggled, this,
          &WbView3D::setShowPenPaintingRays);
  connect(actionManager->action(WbActionManager::CENTER_OF_MASS), &QAction::triggered, this, &WbView3D::showCenterOfMass);
  connect(actionManager->action(WbActionManager::SUPPORT_POLYGON), &QAction::triggered, this, &WbView3D::showSupportPolygon);
  connect(actionManager->action(WbActionManager::SKIN_SKELETON), &QAction::triggered, this, &WbView3D::setShowSkeletonAction);
  connect(actionManager->action(WbActionManager::RADAR_FRUSTUMS), &QAction::toggled, this, &WbView3D::setShowRadarFrustums);
  connect(actionManager->action(WbActionManager::PHYSICS_CLUSTERS), &QAction::triggered, this,
          &WbView3D::setShowPhysicsClustersAction);
  connect(actionManager->action(WbActionManager::BOUNDING_SPHERE), &QAction::triggered, this,
          &WbView3D::setShowBoundingSphereAction);
  // virtual reality headset
  const WbPreferences *const prefs = WbPreferences::instance();
  connect(actionManager->action(WbActionManager::VIRTUAL_REALITY_HEADSET_ENABLE), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadset);
  setVirtualRealityHeadset(WbPreferences::instance()->value("VirtualRealityHeadset/enable").toBool());
  connect(actionManager->action(WbActionManager::VIRTUAL_REALITY_HEADSET_POSITION), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetPositionTracking);
  setVirtualRealityHeadsetPositionTracking(WbPreferences::instance()->value("VirtualRealityHeadset/trackPosition").toBool());
  connect(actionManager->action(WbActionManager::VIRTUAL_REALITY_HEADSET_ORIENTATION), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetOrientationTracking);
  setVirtualRealityHeadsetOrientationTracking(
    WbPreferences::instance()->value("VirtualRealityHeadset/trackOrientation").toBool());
  connect(actionManager->action(WbActionManager::VIRTUAL_REALITY_HEADSET_LEFT_EYE), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetLeftEyeView);
  setVirtualRealityHeadsetLeftEyeView(WbPreferences::instance()->value("VirtualRealityHeadset/visibleEye").toString() ==
                                      "left");
  connect(actionManager->action(WbActionManager::VIRTUAL_REALITY_HEADSET_RIGHT_EYE), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetRightEyeView);
  setVirtualRealityHeadsetRightEyeView(WbPreferences::instance()->value("VirtualRealityHeadset/visibleEye").toString() ==
                                       "right");
  connect(actionManager->action(WbActionManager::VIRTUAL_REALITY_HEADSET_NO_EYE), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetNoEyeView);
  setVirtualRealityHeadsetNoEyeView(WbPreferences::instance()->value("VirtualRealityHeadset/visibleEye").toString() == "none");
  connect(actionManager->action(WbActionManager::VIRTUAL_REALITY_HEADSET_ANTI_ALIASING), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetAntiAliasing);
  setVirtualRealityHeadsetAntiAliasing(WbPreferences::instance()->value("VirtualRealityHeadset/antiAliasing").toBool());
  actionManager->action(WbActionManager::HIDE_ALL_CAMERA_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllCameraOverlays", false).toBool());
  connect(actionManager->action(WbActionManager::HIDE_ALL_CAMERA_OVERLAYS), &QAction::toggled, this,
          &WbView3D::setHideAllCameraOverlays);
  actionManager->action(WbActionManager::HIDE_ALL_RANGE_FINDER_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllRangeFinderOverlays", false).toBool());
  connect(actionManager->action(WbActionManager::HIDE_ALL_RANGE_FINDER_OVERLAYS), &QAction::toggled, this,
          &WbView3D::setHideAllRangeFinderOverlays);
  actionManager->action(WbActionManager::HIDE_ALL_DISPLAY_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllDisplayOverlays", false).toBool());
  connect(actionManager->action(WbActionManager::HIDE_ALL_DISPLAY_OVERLAYS), &QAction::toggled, this,
          &WbView3D::setHideAllDisplayOverlays);
  // enable/disable shadows when preferences change
  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbView3D::updateShadowState);
}

void WbView3D::setPerspectiveProjection() {
  setProjectionMode(WR_CAMERA_PROJECTION_MODE_PERSPECTIVE, true);
}

void WbView3D::setOrthographicProjection() {
  setProjectionMode(WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC, true);
}

void WbView3D::setPlain() {
  setRenderingMode(WR_VIEWPORT_POLYGON_MODE_FILL, true);
}

void WbView3D::setWireframe() {
  setRenderingMode(WR_VIEWPORT_POLYGON_MODE_LINE, true);
}

void WbView3D::onSelectionChanged(WbAbstractTransform *selectedAbstractTransform) {
  assert(mWorld);

  WbSolid *const selectedSolid = dynamic_cast<WbSolid *>(selectedAbstractTransform);

  if (selectedSolid) {
    setCheckedShowSupportPolygonAction(selectedSolid);
    setCheckedShowCenterOfMassAction(selectedSolid);
    setCheckedShowCenterOfBuoyancyAction(selectedSolid);
    setCheckedFollowObjectAction(selectedSolid);
    selectedSolid->updateTranslateRotateHandlesSize();
  } else {
    WbViewpoint *const viewpoint = mWorld->viewpoint();
    if (viewpoint->isFollowingOrientation())
      WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setChecked(true);
    else if (viewpoint->followedSolid())
      WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(true);
    WbActionManager::instance()->action(WbActionManager::SUPPORT_POLYGON)->setChecked(false);
    WbActionManager::instance()->action(WbActionManager::CENTER_OF_MASS)->setChecked(false);
    WbActionManager::instance()->action(WbActionManager::CENTER_OF_BUOYANCY)->setChecked(false);
  }

  bool enable = selectedSolid != NULL;
  WbActionManager::instance()
    ->action(WbActionManager::FOLLOW_OBJECT)
    ->setEnabled(enable || WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->isChecked());
  WbActionManager::instance()
    ->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)
    ->setEnabled(enable || WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->isChecked());
  WbActionManager::instance()->action(WbActionManager::CENTER_OF_BUOYANCY)->setEnabled(enable);
  WbActionManager::instance()->action(WbActionManager::CENTER_OF_MASS)->setEnabled(enable);
  WbActionManager::instance()->action(WbActionManager::SUPPORT_POLYGON)->setEnabled(enable);

  cleanupEvents();
}

WbView3D::~WbView3D() {
  cleanupFullScreenOverlay();
  cleanupPickers();
  cleanupOptionalRendering();
  WbWrenRenderingContext::cleanup();
  delete mMousePressTime;

  WbWrenLabelOverlay::cleanup();
#ifdef _WIN32
  WbVirtualRealityHeadset::cleanup();
#endif
}

void WbView3D::focusInEvent(QFocusEvent *event) {
  WbActionManager::instance()->enableTextEditActions(false);
  WbActionManager::instance()->setFocusObject(this);
  emit applicationActionsUpdateRequested();
}

void WbView3D::focusOutEvent(QFocusEvent *event) {
  if (WbActionManager::instance()->focusObject() == this)
    WbActionManager::instance()->setFocusObject(NULL);
}

// main refresh function (update from the simulation engine)
// for refresh coming from the GUI, use renderLater() instead
void WbView3D::refresh() {
  if (!mWorld) {
    // render black screen
    renderLater();
    return;
  }

  const WbSimulationState *const sim = WbSimulationState::instance();
  mPhysicsRefresh = true;
  if (sim->isPaused())
    renderLater();
  else if (sim->isStep() || sim->isRealTime() || sim->isRunning()) {
    if (WbVideoRecorder::instance()->isRecording()) {
      const int displayRefresh = WbVideoRecorder::displayRefresh();
      mRefreshCounter = (mRefreshCounter + 1) % displayRefresh;
      if (mRefreshCounter == 0)
        // render main window immediately even if it is not exposed
        renderNow();
    } else if (sim->isPaused())
      renderLater();
    else {
      const qint64 lastRefreshDelta = mLastRefreshTimer.elapsed();
      const double maxFrameDuration = 1000.0 / mWorld->worldInfo()->fps();  // ms
      if (lastRefreshDelta > maxFrameDuration)
        renderNow();
    }
  }
  // else isFast -> no rendering

  mPhysicsRefresh = false;
}

// Initializes or terminates solid's camera follow up according to the status of the WbActionManager::FOLLOW_OBJECT action
void WbView3D::followSolid(bool checked) {
  mWorld->setModified();

  WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setChecked(false);

  WbViewpoint *const viewpoint = mWorld->viewpoint();
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  if (!checked) {
    viewpoint->terminateFollowUp();
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setEnabled(selectedSolid != NULL);
    return;
  }

  assert(selectedSolid);

  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();

  viewpoint->setFollowOrientation(false);
  viewpoint->startFollowUp(selectedSolid, true);
}

// Initializes or terminates solid's camera follow up according to the status of the WbActionManager::FOLLOW_OBJECT_AND_ROTATE
// action
void WbView3D::followSolidAndRotate(bool checked) {
  mWorld->setModified();

  WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(false);

  WbViewpoint *const viewpoint = mWorld->viewpoint();
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  if (!checked) {
    viewpoint->terminateFollowUp();
    viewpoint->setFollowOrientation(false);
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setEnabled(selectedSolid != NULL);
    return;
  }

  assert(selectedSolid);

  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();

  viewpoint->setFollowOrientation(true);
  viewpoint->startFollowUp(selectedSolid, true);
}

void WbView3D::setCheckedFollowObjectAction(WbSolid *selectedSolid) {
  if (selectedSolid) {
    const WbViewpoint *const viewpoint = mWorld->viewpoint();
    if (viewpoint->isFollowingOrientation()) {
      WbActionManager::instance()
        ->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)
        ->setChecked(viewpoint->isFollowed(selectedSolid));
      WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(false);
    } else {
      WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(viewpoint->isFollowed(selectedSolid));
      WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setChecked(false);
    }
  }
}

// Notifies a change in the follow object action (checked/unchecked) from mViewpoint
void WbView3D::notifyFollowObjectAction(bool validField) {
  const WbViewpoint *const viewpoint = mWorld->viewpoint();
  if (viewpoint->isFollowingOrientation()) {
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setChecked(validField);
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(false);
  } else {
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(validField);
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setChecked(false);
  }
}

// Notifies a change in the follow object action (checked/unchecked) from mViewpoint
void WbView3D::notifyFollowObjectAndRotationAction(bool rotate) {
  if (rotate) {
    WbActionManager::instance()
      ->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)
      ->setChecked(WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->isChecked());
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(false);
  } else {
    WbActionManager::instance()
      ->action(WbActionManager::FOLLOW_OBJECT)
      ->setChecked(WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->isChecked());
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setChecked(false);
  }
}

// Shows the center of mass and the support polygon of a dynamic top WbSolid
void WbView3D::showSupportPolygon(bool checked) {
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (!selectedSolid->showSupportPolygonRepresentation(checked))
    WbActionManager::instance()->action(WbActionManager::SUPPORT_POLYGON)->setChecked(false);

  renderLater();
}

// Shows the center of mass of a dynamic WbSolid
void WbView3D::showCenterOfMass(bool checked) {
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (selectedSolid->showGlobalCenterOfMassRepresentation(checked) == false)
    WbActionManager::instance()->action(WbActionManager::CENTER_OF_MASS)->setChecked(false);

  renderLater();
}

void WbView3D::setCheckedShowCenterOfMassAction(WbSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->globalCenterOfMassRepresentationEnabled();
  WbActionManager::instance()->action(WbActionManager::CENTER_OF_MASS)->setChecked(enabled);
  if (enabled)
    renderLater();
}

// Shows the center of buoyancy of a dynamic WbSolid
void WbView3D::showCenterOfBuoyancy(bool checked) {
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (selectedSolid->showCenterOfBuoyancyRepresentation(checked) == false)
    WbActionManager::instance()->action(WbActionManager::CENTER_OF_BUOYANCY)->setChecked(false);

  renderLater();
}

void WbView3D::setCheckedShowCenterOfBuoyancyAction(WbSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->centerOfBuoyancyRepresentationEnabled();
  WbActionManager::instance()->action(WbActionManager::CENTER_OF_BUOYANCY)->setChecked(enabled);
  if (enabled)
    renderLater();
}

void WbView3D::setCheckedShowSupportPolygonAction(WbSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->supportPolygonRepresentationEnabled();
  WbActionManager::instance()
    ->action(WbActionManager::SUPPORT_POLYGON)
    ->setChecked(selectedSolid->supportPolygonRepresentationEnabled());
  if (enabled)
    renderLater();
}

void WbView3D::restoreViewpoint() {
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  viewpoint->restore();
  renderLater();
}

WrViewportPolygonMode WbView3D::stringToRenderingMode(const QString &s) {
  if (s == "WIREFRAME")
    return WR_VIEWPORT_POLYGON_MODE_LINE;

  return WR_VIEWPORT_POLYGON_MODE_FILL;  // default value
}

WrCameraProjectionMode WbView3D::stringToProjectionMode(const QString &s) {
  if (s == "ORTHOGRAPHIC")
    return WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC;

  return WR_CAMERA_PROJECTION_MODE_PERSPECTIVE;
}

void WbView3D::setRenderingMode(WrViewportPolygonMode mode, bool updatePerspective) {
  switch (mode) {
    case WR_VIEWPORT_POLYGON_MODE_FILL:
      if (updatePerspective && mWorld)
        mWorld->perspective()->setRenderingMode("PLAIN");
      WbActionManager::instance()->action(WbActionManager::PLAIN_RENDERING)->setChecked(true);
      break;
    case WR_VIEWPORT_POLYGON_MODE_LINE:
      if (updatePerspective && mWorld)
        mWorld->perspective()->setRenderingMode("WIREFRAME");
      WbActionManager::instance()->action(WbActionManager::WIREFRAME_RENDERING)->setChecked(true);
      break;
    default:
      assert(false);
  }

  mRenderingMode = mode;

  if (wr_gl_state_is_initialized()) {
    if (mRenderingMode == WR_VIEWPORT_POLYGON_MODE_FILL)
      WbWrenRenderingContext::instance()->setRenderingMode(WbWrenRenderingContext::RM_PLAIN, true);
    else
      WbWrenRenderingContext::instance()->setRenderingMode(WbWrenRenderingContext::RM_WIREFRAME, true);
  }

  renderLater();
}

void WbView3D::setVirtualRealityHeadset(bool enable) {
  if (mWorld) {
    bool sucess = mWorld->viewpoint()->enableVirtualRealityHeadset(enable);
    if (sucess)
      renderLater();
    else
      enable = !enable;
  }

  WbPreferences::instance()->setValue("VirtualRealityHeadset/enable", enable);
  WbActionManager::instance()->action(WbActionManager::VIRTUAL_REALITY_HEADSET_ENABLE)->setChecked(enable);

  if (enable) {
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_CAMERA, false);
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, false);
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, false);
  } else {
    if (!WbPreferences::instance()->value("View3d/hideAllCameraOverlays").toBool())
      WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_CAMERA, true);
    if (!WbPreferences::instance()->value("View3d/hideAllRangeFinderOverlays").toBool())
      WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, true);
    if (!WbPreferences::instance()->value("View3d/hideAllDisplayOverlays").toBool())
      WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, true);
  }

  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setVirtualRealityHeadsetPositionTracking(bool enable) {
  WbPreferences::instance()->setValue("VirtualRealityHeadset/trackPosition", enable);
  WbActionManager::instance()->action(WbActionManager::VIRTUAL_REALITY_HEADSET_POSITION)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse()) {
    WbVirtualRealityHeadset::instance()->enablePositionTracking(enable);
    renderLater();
  }
#endif
}

void WbView3D::setVirtualRealityHeadsetOrientationTracking(bool enable) {
  WbPreferences::instance()->setValue("VirtualRealityHeadset/trackOrientation", enable);
  WbActionManager::instance()->action(WbActionManager::VIRTUAL_REALITY_HEADSET_ORIENTATION)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse()) {
    WbVirtualRealityHeadset::instance()->enableOrientationTracking(enable);
    renderLater();
  }
#endif
}

void WbView3D::setVirtualRealityHeadsetLeftEyeView(bool enable) {
  if (enable)
    WbPreferences::instance()->setValue("VirtualRealityHeadset/visibleEye", "left");
  WbActionManager::instance()->action(WbActionManager::VIRTUAL_REALITY_HEADSET_LEFT_EYE)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && enable) {
    WbVirtualRealityHeadset::instance()->setEyeView(WbVirtualRealityHeadset::LEFT);
    renderLater();
  }
#endif
  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setVirtualRealityHeadsetRightEyeView(bool enable) {
  if (enable)
    WbPreferences::instance()->setValue("VirtualRealityHeadset/visibleEye", "right");
  WbActionManager::instance()->action(WbActionManager::VIRTUAL_REALITY_HEADSET_RIGHT_EYE)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && enable) {
    WbVirtualRealityHeadset::instance()->setEyeView(WbVirtualRealityHeadset::RIGHT);
    renderLater();
  }
#endif
  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setVirtualRealityHeadsetNoEyeView(bool enable) {
  if (enable)
    WbPreferences::instance()->setValue("VirtualRealityHeadset/visibleEye", "none");
  WbActionManager::instance()->action(WbActionManager::VIRTUAL_REALITY_HEADSET_NO_EYE)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && enable) {
    WbVirtualRealityHeadset::instance()->setEyeView(WbVirtualRealityHeadset::NONE);
    renderLater();
  }
#endif
  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setVirtualRealityHeadsetAntiAliasing(bool enable) {
  WbPreferences::instance()->setValue("VirtualRealityHeadset/antiAliasing", enable);
  WbActionManager::instance()->action(WbActionManager::VIRTUAL_REALITY_HEADSET_ANTI_ALIASING)->setChecked(enable);
  if (mWorld) {
    mWorld->viewpoint()->setVirtualRealityHeadsetAntiAliasing(enable);
    renderLater();
  }
  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setProjectionMode(WrCameraProjectionMode mode, bool updatePerspective) {
  mProjectionMode = mode;
  if (mWorld)
    mWorld->viewpoint()->setProjectionMode(mode);

  switch (mode) {
    case WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC:
      WbActionManager::instance()->action(WbActionManager::ORTHOGRAPHIC_PROJECTION)->setChecked(true);
      if (mWorld) {
        mWorld->viewpoint()->updateOrthographicViewHeight();
        if (updatePerspective)
          mWorld->perspective()->setProjectionMode("ORTHOGRAPHIC");
      }
      break;
    default:
      if (updatePerspective && mWorld)
        mWorld->perspective()->setProjectionMode("PERSPECTIVE");
      WbActionManager::instance()->action(WbActionManager::PERSPECTIVE_PROJECTION)->setChecked(true);
      break;
  }

  if (wr_gl_state_is_initialized())
    wr_camera_set_projection_mode(wr_viewport_get_camera(wr_scene_get_viewport(wr_scene_get_instance())), mProjectionMode);

  renderLater();
}

void WbView3D::setShowCoordinateSystem(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("CoordinateSystem", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_COORDINATE_SYSTEM, show);
  // Like other optional rendering features, enabling the coordinate system
  // triggers a redraw on the screen. However until the user interacts with
  // webots the coordinate system will not be rendered onto the scene.
  // We force the coordinate system to be rendered here so that it appears
  // immediately, without needing user interaction.
  renderNow();
}

void WbView3D::setShowBoundingObjects(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("AllBoundingObjects", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS, show);
}

void WbView3D::setShowContactPoints(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("ContactPoints", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CONTACT_POINTS, show);
}

void WbView3D::setShowConnectorAxes(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("ConnectorAxes", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CONNECTOR_AXES, show);
}

void WbView3D::setShowJointAxes(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("JointAxes", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_JOINT_AXES, show);
}

void WbView3D::setShowCameraFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("CameraFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CAMERA_FRUSTUMS, show);
}

void WbView3D::setShowRangeFinderFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("RangeFinderFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS, show);
}

void WbView3D::setShowRadarFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("RadarFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_RADAR_FRUSTUMS, show);
}

void WbView3D::setShowLidarRaysPaths(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LidarRaysPaths", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIDAR_RAYS_PATHS, show);
}

void WbView3D::setShowLidarPointClouds(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LidarPointClouds", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIDAR_POINT_CLOUD, show);
}

void WbView3D::setShowRenderingDevice(bool checked) {
  WbRenderingDevice *device = static_cast<WbRenderingDevice *>(sender()->property("renderingDevice").value<void *>());
  device->toggleOverlayVisibility(checked);
  renderLater();
}

void WbView3D::setHideAllCameraOverlays(bool hidden) {
  WbPreferences::instance()->setValue("View3d/hideAllCameraOverlays", hidden);

#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && hidden)
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_CAMERA, false);
  else if (!WbVirtualRealityHeadset::isInUse())
#endif
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_CAMERA, !hidden);

  renderLater();
}

void WbView3D::setHideAllRangeFinderOverlays(bool hidden) {
  WbPreferences::instance()->setValue("View3d/hideAllRangeFinderOverlays", hidden);

#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && hidden)
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, false);
  else if (!WbVirtualRealityHeadset::isInUse())
#endif
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, !hidden);

  renderLater();
}

void WbView3D::setHideAllDisplayOverlays(bool hidden) {
  WbPreferences::instance()->setValue("View3d/hideAllDisplayOverlays", hidden);

#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && hidden)
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, false);
  else if (!WbVirtualRealityHeadset::isInUse())
#endif
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, !hidden);

  renderLater();
}

void WbView3D::setShowDistanceSensorRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("DistanceSensorRays", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_DISTANCE_SENSORS_RAYS, show);
}

void WbView3D::setShowLightSensorRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LightSensorRays", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIGHT_SENSORS_RAYS, show);
}

void WbView3D::setShowLightsPositions(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LightPositions", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIGHTS_POSITIONS, show);
}

void WbView3D::setShowPenPaintingRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("PenPaintingRays", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_PEN_RAYS, show);
}

void WbView3D::setShowSkeletonAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("Skeleton", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_SKIN_SKELETON, show);
}

void WbView3D::setShowPhysicsClustersAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("PhysicsClusters", show);
  WbOdeDebugger::instance()->toggleDebugging(show);
}

void WbView3D::setShowBoundingSphereAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("BoundingSphere", show);
  WbVisualBoundingSphere::enable(show, WbSelection::instance()->selectedNode());
  renderLater();
}

void WbView3D::setSelectionDisabled(bool disabled) {
  mSelectionDisabled = disabled;
  if (mWorld)
    mWorld->perspective()->setSelectionDisabled(disabled);
}

void WbView3D::setViewPointLocked(bool locked) {
  mViewpointLocked = locked;
  if (mWorld)
    mWorld->perspective()->setViewpointLocked(locked);
}

void WbView3D::updateMousesPosition(bool fromMouseClick, bool fromMouseMove) {
  const QList<WbMouse *> mouses = WbMouse::mouses();
  if (mouses.size() == 0)
    return;

  QList<WbMouse *> mousesRequiringRefresh;
  bool shouldUsePicker = false;
  for (int i = 0; i < WbMouse::mouses().size(); ++i) {
    WbMouse *mouse = WbMouse::mouses().at(i);
    if (mouse->needToRefresh()) {
      mousesRequiringRefresh.append(mouse);
      if (!shouldUsePicker && mouse->is3dPositionEnabled())
        shouldUsePicker = true;
    }
    if (!mouse->isTracked()) {
      // In the non-tracked case, update the buttons in any cases to avoid loosing a press event
      // in case the press and release events happen in the same step
      mouse->setLeft(mouse->left() | (mLastButtonState & Qt::LeftButton));
      mouse->setMiddle(mouse->middle() | (mLastButtonState & Qt::MiddleButton));
      mouse->setRight(mouse->right() | (mLastButtonState & Qt::RightButton));
    }
  }

  if (mousesRequiringRefresh.size() == 0)
    return;

  const QPoint position = mapFromGlobal(QCursor::pos());
  if (position.x() < 0 || position.y() < 0 || position.x() >= width() || position.y() >= height())
    return;

  if (!mControllerPicker)
    mControllerPicker = new WbWrenPicker();

  const bool picked = shouldUsePicker ? mControllerPicker->pick(position.x(), position.y()) : false;

  foreach (WbMouse *mouse, mousesRequiringRefresh) {
    if (picked && mouse->is3dPositionEnabled()) {
      WbVector3 screenPosition = mControllerPicker->screenCoordinates();
      screenPosition[0] = (screenPosition[0] / width()) * 2 - 1;
      screenPosition[1] = (screenPosition[1] / height()) * 2 - 1;
      WbVector3 worldPosition;
      mWorld->viewpoint()->toWorld(screenPosition, worldPosition);
      mouse->setPosition(worldPosition.x(), worldPosition.y(), worldPosition.z());
    }

    mouse->setScreenPosition((double)position.x() / width(), (double)position.y() / height());

    if (mouse->isTracked()) {
      mouse->setLeft(mLastButtonState & Qt::LeftButton);
      mouse->setMiddle(mLastButtonState & Qt::MiddleButton);
      mouse->setRight(mLastButtonState & Qt::RightButton);
    }
    mouse->setHasMoved(fromMouseMove);
    mouse->setHasClicked(fromMouseClick);
    mouse->refreshSensorIfNeeded();
    emit mouse->changed();
  }
}

void WbView3D::logWrenStatistics() const {
  // TODO_WREN
  // WbPerformanceLog *log = WbPerformanceLog::instance();
  // if (log && mRenderWindow) {
  //   Ogre::RenderTarget::FrameStats stats = mRenderWindow->getStatistics();
  //   log->setAvgFPS(stats.avgFPS);
  // }
}

void WbView3D::prepareWorldLoading() {
  WbWrenOpenGlContext::makeWrenCurrent();

  // reset text labels
  WbWrenLabelOverlay::removeAllLabels();

  if (!wr_gl_state_is_initialized())  // may occur at least on Windows when launched with the minimized option
    initialize();

  if (!mLoadingWorldOverlay) {
    mLoadingWorldOverlay = new WbWrenFullScreenOverlay(tr("Loading world"), 80, true);
    mLoadingWorldOverlay->attachToViewport(wr_scene_get_viewport(wr_scene_get_instance()));
#ifdef _WIN32
    WbVirtualRealityHeadset::setLoadingTexture(mLoadingWorldOverlay->overlayTexture());
#endif
  }
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse())
    WbVirtualRealityHeadset::instance()->setTextureOverlayVisible(true);
#endif
  hideFastModeOverlay();
  mLoadingWorldOverlay->setVisible(true);
  WbWrenWindow::renderNow();

  // restart refresh counter
  mRefreshCounter = 0;

  // Resets the background if no Background node exists
  const float clearColor[] = {1.0f, 1.0f, 1.0f};
  wr_viewport_set_clear_color_rgb(wr_scene_get_viewport(wr_scene_get_instance()), clearColor);

  // Cleanup the drags events that were possibly used in the previous world
  cleanupEvents();

  // signals that update the menu's ticks according to the status of the selection
  disconnect(WbSelection::instance(), &WbSelection::selectionChangedFromView3D, this, &WbView3D::onSelectionChanged);
  disconnect(WbSelection::instance(), &WbSelection::selectionChangedFromSceneTree, this, &WbView3D::onSelectionChanged);

  cleanupOptionalRendering();

  WbWrenOpenGlContext::doneWren();
}

void WbView3D::updateViewport() {
  // Sets the solid follow up according to viewpoint's follow field
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  connect(viewpoint, &WbViewpoint::followInvalidated, this, &WbView3D::notifyFollowObjectAction);
  connect(viewpoint, &WbViewpoint::followOrientationChanged, this, &WbView3D::notifyFollowObjectAndRotationAction);
  connect(viewpoint, SIGNAL(virtualRealityHeadsetRequiresRender()), this, SLOT(renderNow()));
  if (viewpoint->followedSolid() && viewpoint->isFollowingOrientation())
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setChecked(true);
  else if (viewpoint->followedSolid())
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(true);

  cleanupPickers();
  mPicker = new WbWrenPicker();

  viewpoint->updateAspectRatio(mAspectRatio);

  // Re-initialize matter handles size
  WbWrenAbstractManipulator::setViewport(viewpoint->viewportWren());
  WbSelection::instance()->updateHandlesScale();

#ifdef _WIN32
  updateVirtualRealityHeadsetOverlay();
#endif

  // update handles size when viewpoint changes
  connect(viewpoint, &WbViewpoint::cameraParametersChanged, WbSelection::instance(), &WbSelection::updateHandlesScale);
}

void WbView3D::updateShadowState() {
  if (WbPreferences::instance()->value("OpenGL/disableShadows").toBool() == wr_config_are_shadows_enabled()) {
    wr_config_enable_shadows(!WbPreferences::instance()->value("OpenGL/disableShadows").toBool());
    renderLater();
  }
}

void WbView3D::setWorld(WbSimulationWorld *w) {
  WbWrenOpenGlContext::makeWrenCurrent();

  mLoadingWorldOverlay->setVisible(false);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse())
    WbVirtualRealityHeadset::instance()->setTextureOverlayVisible(false);
#endif
  mWorld = w;  // world is loaded!

  // apply optional rendering
  if (WbPreferences::instance()->value("View3d/hideAllCameraOverlays").toBool())
    setHideAllCameraOverlays(true);
  if (WbPreferences::instance()->value("View3d/hideAllRangeFinderOverlays").toBool())
    setHideAllRangeFinderOverlays(true);
  if (WbPreferences::instance()->value("View3d/hideAllDisplayOverlays").toBool())
    setHideAllDisplayOverlays(true);

  const WbPerspective *perspective = mWorld->perspective();
  setProjectionMode(stringToProjectionMode(perspective->projectionMode()), false);
  setRenderingMode(stringToRenderingMode(perspective->renderingMode()), false);
  mSelectionDisabled = perspective->isSelectionDisabled();
  mViewpointLocked = perspective->isViewpointLocked();

  enableOptionalRenderingFromPerspective();

  connect(mWorld, &WbSimulationWorld::destroyed, this, &WbView3D::cleanWorld);
  connect(mWorld, &WbWorld::viewpointChanged, this, &WbView3D::updateViewport);

  // Sets the solid follow up according to viewpoint's follow field
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  connect(viewpoint, &WbViewpoint::followInvalidated, this, &WbView3D::notifyFollowObjectAction);
  connect(viewpoint, &WbViewpoint::followOrientationChanged, this, &WbView3D::notifyFollowObjectAndRotationAction);
  connect(viewpoint, SIGNAL(virtualRealityHeadsetRequiresRender()), this, SLOT(renderNow()));
  viewpoint->startFollowUpFromField();
  if (viewpoint->followedSolid() && viewpoint->isFollowingOrientation())
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT_AND_ROTATE)->setChecked(true);
  else if (viewpoint->followedSolid())
    WbActionManager::instance()->action(WbActionManager::FOLLOW_OBJECT)->setChecked(true);

  // Prepares the contact point rendering (Note: WbControlledSimulation::instance() is valid after the call to
  // mMainWindow->loadWorld(mWorldName) in WbGuiApplication.cpp)
  // The constructor connects an update slot to the signal WbSimulationWorld::physicsStepEnded()
  mContactPointsRepresentation = new WbContactPointsRepresentation(mWrenRenderingContext);

  // Connects GUI-defined mode and rendering options to update methods for material of bounding objects
  const WbSimulationState *const simulationState = WbSimulationState::instance();
  connect(mWrenRenderingContext, &WbWrenRenderingContext::optionalRenderingChanged, mWorld,
          &WbSimulationWorld::checkNeedForBoundingMaterialUpdate, Qt::UniqueConnection);
  connect(simulationState, &WbSimulationState::modeChanged, mWorld, &WbSimulationWorld::checkNeedForBoundingMaterialUpdate,
          Qt::UniqueConnection);
  mWorld->checkNeedForBoundingMaterialUpdate();

  // Prepares the shape picker
  delete mPicker;
  delete mControllerPicker;
  mControllerPicker = NULL;
  mPicker = new WbWrenPicker();

  // Creates the fast mode overlay
  if (!mFastModeOverlay) {
    mFastModeOverlay = new WbWrenFullScreenOverlay("Fast mode", 128, true);
    mFastModeOverlay->attachToViewport(wr_scene_get_viewport(wr_scene_get_instance()));
  }
  if (simulationState->mode() == WbSimulationState::FAST)
    showFastModeOverlay();
  else
    hideFastModeOverlay();

#ifdef _WIN32
  // Creates the virtual reality headset overlay
  if (!mVirtualRealityHeadsetOverlay) {
    mVirtualRealityHeadsetOverlay = new WbWrenFullScreenOverlay("Headset preview disabled", 64, false);
    mVirtualRealityHeadsetOverlay->attachToViewport(wr_scene_get_viewport(wr_scene_get_instance()));
  }
  updateVirtualRealityHeadsetOverlay();
#endif

  // connect supervisor scene tree modifications to graphical updates
  const QList<WbRobot *> &robots = mWorld->robots();
  foreach (WbRobot *const robot, robots) {
    if (robot->supervisor())
      connect(robot->supervisorUtilities(), &WbSupervisorUtilities::worldModified, this,
              &WbView3D::handleWorldModificationFromSupervior);
  }

  // initialize matter handles size
  WbWrenAbstractManipulator::setViewport(wr_scene_get_viewport(wr_scene_get_instance()));
  WbSelection::instance()->updateHandlesScale();
  // update handles size when viewpoint changes
  connect(viewpoint, &WbViewpoint::cameraParametersChanged, WbSelection::instance(), &WbSelection::updateHandlesScale);
  connect(viewpoint, &WbViewpoint::refreshRequired, this, &WbView3D::renderLater);

  // signals that update the menu's ticks according to the status of the selection
  connect(WbSelection::instance(), &WbSelection::selectionChangedFromView3D, this, &WbView3D::onSelectionChanged);
  connect(WbSelection::instance(), &WbSelection::selectionChangedFromSceneTree, this, &WbView3D::onSelectionChanged);

  mAspectRatio = ((double)width()) / height();
  viewpoint->updateAspectRatio(mAspectRatio);
  updateWrenViewportDimensions();
  onSelectionChanged(WbSelection::instance()->selectedAbstractTransform());

  WbWrenOpenGlContext::doneWren();

  // first rendering without culling to make sure every meshes/textures are actually loaded on the GPU
  renderNow(false);
}

void WbView3D::restoreOptionalRendering(const QStringList &enabledCenterOfMassNodeNames,
                                        const QStringList &enabledCenterOfBuoyancyNodeNames,
                                        const QStringList &enabledSupportPolygonNodeNames) const {
  // restore node specific optional rendering from world properties
  WbSolid *solid = NULL;
  for (int i = 0; i < enabledCenterOfMassNodeNames.size(); ++i) {
    solid = WbSolid::findSolidFromUniqueName(enabledCenterOfMassNodeNames[i]);
    if (solid)
      solid->showGlobalCenterOfMassRepresentation(true);
  }

  for (int i = 0; i < enabledCenterOfBuoyancyNodeNames.size(); ++i) {
    solid = WbSolid::findSolidFromUniqueName(enabledCenterOfBuoyancyNodeNames[i]);
    if (solid)
      solid->showCenterOfBuoyancyRepresentation(true);
  }

  for (int i = 0; i < enabledSupportPolygonNodeNames.size(); ++i) {
    solid = WbSolid::findSolidFromUniqueName(enabledSupportPolygonNodeNames[i]);
    if (solid)
      solid->showSupportPolygonRepresentation(true);
  }
}

void WbView3D::enableOptionalRenderingFromPerspective() {
  // Enables optional rendering from preferences
  assert(mWorld);
  const WbPerspective *perspective = mWorld->perspective();
  WbActionManager *actionManager = WbActionManager::instance();
  actionManager->action(WbActionManager::COORDINATE_SYSTEM)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("CoordinateSystem"));
  actionManager->action(WbActionManager::BOUNDING_OBJECT)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("AllBoundingObjects"));
  actionManager->action(WbActionManager::CONTACT_POINTS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("ContactPoints"));
  actionManager->action(WbActionManager::CONNECTOR_AXES)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("ConnectorAxes"));
  actionManager->action(WbActionManager::JOINT_AXES)->setChecked(perspective->isGlobalOptionalRenderingEnabled("JointAxes"));
  actionManager->action(WbActionManager::RANGE_FINDER_FRUSTUMS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("RangeFinderFrustums"));
  actionManager->action(WbActionManager::LIDAR_RAYS_PATH)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("LidarRaysPaths"));
  actionManager->action(WbActionManager::LIDAR_POINT_CLOUD)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("LidarPointClouds"));
  actionManager->action(WbActionManager::CAMERA_FRUSTUM)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("CameraFrustums"));
  actionManager->action(WbActionManager::DISTANCE_SENSOR_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("DistanceSensorRays"));
  actionManager->action(WbActionManager::LIGHT_SENSOR_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("LightSensorRays"));
  actionManager->action(WbActionManager::LIGHT_POSITIONS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("LightPositions"));
  actionManager->action(WbActionManager::CENTER_OF_BUOYANCY)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("CenterOfBuoyancy"));
  actionManager->action(WbActionManager::PEN_PAINTING_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("PenPaintingRays"));
  actionManager->action(WbActionManager::CENTER_OF_MASS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("CenterOfMass"));
  actionManager->action(WbActionManager::SUPPORT_POLYGON)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("SupportPolygon"));
  actionManager->action(WbActionManager::SKIN_SKELETON)->setChecked(perspective->isGlobalOptionalRenderingEnabled("Skeleton"));
  actionManager->action(WbActionManager::RADAR_FRUSTUMS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("RadarFrustums"));
  actionManager->action(WbActionManager::PHYSICS_CLUSTERS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("PhysicsClusters"));
  actionManager->action(WbActionManager::BOUNDING_SPHERE)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("BoundingSphere"));
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_COORDINATE_SYSTEM,
                                                 perspective->isGlobalOptionalRenderingEnabled("CoordinateSystem"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS,
                                                 perspective->isGlobalOptionalRenderingEnabled("AllBoundingObjects"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CONTACT_POINTS,
                                                 perspective->isGlobalOptionalRenderingEnabled("ContactPoints"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CONNECTOR_AXES,
                                                 perspective->isGlobalOptionalRenderingEnabled("ConnectorAxes"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_JOINT_AXES,
                                                 perspective->isGlobalOptionalRenderingEnabled("JointAxes"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("RangeFinderFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIDAR_RAYS_PATHS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LidarRaysPaths"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIDAR_POINT_CLOUD,
                                                 perspective->isGlobalOptionalRenderingEnabled("LidarPointClouds"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CAMERA_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("CameraFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_RADAR_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("RadarFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_DISTANCE_SENSORS_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("DistanceSensorRays"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIGHT_SENSORS_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LightSensorRays"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIGHTS_POSITIONS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LightPositions"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_PEN_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("PenPaintingRays"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_SKIN_SKELETON,
                                                 perspective->isGlobalOptionalRenderingEnabled("Skeleton"), false);
  WbOdeDebugger::instance()->toggleDebugging(perspective->isGlobalOptionalRenderingEnabled("PhysicsClusters"));
}

void WbView3D::checkRendererCapabilities() {
  QString message;  // The displayed message to forge

  // 1. parameters which can be reduced
  bool disableShadows = false;
  bool disableCameraAntiAliasing = false;
  bool disableSMAA = false;
  bool disableGTAO = false;
  int reduceTextureQuality = 0;

  // 2. determine what has to be reduced
  if (!mWrenRenderingContext->isNvidiaRenderer() && !mWrenRenderingContext->isAmdRenderer() &&
      !mWrenRenderingContext->isIntelRenderer()) {
    message += tr("Webots has detected that your GPU vendor is '%1'. "
                  "A recent NVIDIA or AMD graphics adapter is highly recommended to run Webots smoothly. ")
                 .arg(wr_gl_state_get_vendor());

    if (mWrenRenderingContext->isMesaRenderer() || mWrenRenderingContext->isMicrosoftRenderer()) {
      message += tr("Webots has detected that your computer uses a slow 3D software rendering system. "
                    "It is strongly recommended to install the latest graphics drivers provided by your GPU manufacturer. "
                    "Webots will run much faster after the installation of the correct driver.");
    }

    message += '\n';

    disableShadows = true;
    disableCameraAntiAliasing = true;
    disableSMAA = true;
    disableGTAO = true;
    reduceTextureQuality = 1;
  }

  if (mWrenRenderingContext->isIntelRenderer()) {
    message += tr("Webots has detected that your system features an Intel GPU. "
                  "A recent NVIDIA or AMD graphics adapter is highly recommended to run Webots smoothly. ");
    message += '\n';

#ifndef __APPLE__
    int gpuGeneration = WbSysInfo::intelGPUGeneration(WbWrenOpenGlContext::instance()->functions());
    if (gpuGeneration < 5) {
      disableShadows = true;
      disableCameraAntiAliasing = true;
    }
#endif
  }
#ifndef __APPLE__
  else if (WbSysInfo::isAmdLowEndGpu(WbWrenOpenGlContext::instance()->functions())) {
    message += tr("Webots has detected that you are using an old AMD GPU. "
                  "A recent NVIDIA or AMD graphics adapter is highly recommended to run Webots smoothly. ");
    disableCameraAntiAliasing = true;
    disableSMAA = true;
    disableGTAO = true;
    reduceTextureQuality = 1;
  }
#endif

  // check GPU memory (not for Intel GPU, because the texture size has no impact on the rendring speed)
  if (mWrenRenderingContext->isNvidiaRenderer() || mWrenRenderingContext->isAmdRenderer()) {
    if (wr_gl_state_get_gpu_memory() == 2097152)
      WbPreferences::instance()->setValue("OpenGL/limitBakingResolution", true);
    else if (wr_gl_state_get_gpu_memory() < 2097152) {  // Less than 2Gb of GPU memory
      if (message.isEmpty()) {
        message += tr("Webots has detected that your GPU has less than 2Gb of memory. "
                      "A minimum of 2Gb of memory is recommended to use high-resolution textures. ");
        message += '\n';
      }
      if (wr_gl_state_get_gpu_memory() < 1048576)  // Less than 1Gb of GPU memory
        reduceTextureQuality = 2;
      else
        reduceTextureQuality = 1;
    }
  }

  // 3. apply the parameter reducing
  if (disableShadows) {
    message += "\n - ";
    message += tr("Shadows have been deactivated.");
    WbPreferences::instance()->setValue("OpenGL/disableShadows", true);
  }

  if (disableCameraAntiAliasing) {
    message += "\n - ";
    message += tr("The anti-aliasing on the Camera devices has been deactivated.");
    WbPreferences::instance()->setValue("OpenGL/disableCameraAntiAliasing", true);
  }

  if (disableSMAA) {
    message += "\n - ";
    message += tr("Main 3D view anti-aliasing has been de-activated.");
    WbPreferences::instance()->setValue("OpenGL/SMAA", false);
  }

  if (disableGTAO) {
    message += "\n - ";
    message += tr("Main 3D view global ambient occlusion has been de-activated.");
    WbPreferences::instance()->setValue("OpenGL/GTAO", 0);
  }

  if (reduceTextureQuality != 0) {
    message += "\n - ";
    message += tr("Texture quality has been reduced.");
    WbPreferences::instance()->setValue("OpenGL/textureQuality", 2 - reduceTextureQuality);
  }

  // 4. check OpenGL capabilities.
  if (!wr_gl_state_is_anisotropic_texture_filtering_supported()) {
    message += "\n - ";
    message += tr("Anisotropic texture filtering is not supported by the GPU.");
  }

  // 5. complete and display the message
  if (!message.isEmpty()) {
    message += "\n\n";
    if (disableShadows || disableCameraAntiAliasing || disableSMAA || disableGTAO || reduceTextureQuality)
      message += tr("You can try to re-activate some OpenGL features from the Webots preferences.");
    else
      message +=
        tr("If there are some 3D rendering issues, you can try to reduce some OpenGL features from the Webots preferences.");

    WbLog::warning(tr("System below the minimal requirements.") + "\n\n" + message, true);
  }
}

void WbView3D::initialize() {
  // prepare WREN rendering context
  WbWrenRenderingContext::setWrenRenderingContext(width(), height());
  mWrenRenderingContext = WbWrenRenderingContext::instance();

  // propagate main window refresh signals
  connect(this, &WbView3D::mainRenderingStarted, mWrenRenderingContext, &WbWrenRenderingContext::mainRenderingStarted);
  connect(this, &WbView3D::mainRenderingEnded, mWrenRenderingContext, &WbWrenRenderingContext::mainRenderingEnded);

  // refresh for example when the user change an optional rendering option or
  // the rendering device external window is closed
  connect(mWrenRenderingContext, &WbWrenRenderingContext::view3dRefreshRequired, this, &WbView3D::renderLater);

  if (wr_gl_state_is_initialized())
    return;

  WbWrenWindow::initialize();

  if (WbPreferences::instance()->value("Internal/firstLaunch").toBool())
    checkRendererCapabilities();

  wr_config_enable_shadows(!WbPreferences::instance()->value("OpenGL/disableShadows").toBool());

  // reset timer
  mLastRefreshTimer.start();

  WbRenderingDeviceWindowFactory::storeOpenGLContext(WbWrenOpenGlContext::instance());
}

void WbView3D::resizeWren(int width, int height) {
  if (!mWorld)
    return;

  if (mWrenRenderingContext)
    mWrenRenderingContext->setDimension(width, height);

  if (mFastModeOverlay && mFastModeOverlay->isVisible())
    rescaleFastModePanel();

  if (mLoadingWorldOverlay && mLoadingWorldOverlay->isVisible())
    mLoadingWorldOverlay->adjustSize();

  if (mVirtualRealityHeadsetOverlay && mVirtualRealityHeadsetOverlay->isVisible())
    mVirtualRealityHeadsetOverlay->adjustSize();

  if (!wr_gl_state_is_initialized())
    return;

  if (mWorld) {
    mAspectRatio = (double)width / height;
    mWorld->viewpoint()->updateAspectRatio(mAspectRatio);
  }

  WbWrenWindow::resizeWren(width, height);
}

void WbView3D::renderNow(bool culling) {
  // take screenshot if needed
  if (mScreenshotRequested) {
    mScreenshotRequested = false;
    emit screenshotReady(grabWindowBufferNow());
  }

  if (!wr_gl_state_is_initialized())
    initialize();

  if (mWorld) {
    emit mainRenderingStarted(mPhysicsRefresh);
#ifdef _WIN32
    if (WbVirtualRealityHeadset::isInUse()) {
      WbVirtualRealityHeadset::instance()->updateOrientationAndPosition();
      WbWrenOpenGlContext::makeWrenCurrent();
      if (mVirtualRealityHeadsetOverlay) {
        // on quit it might be possible that 'cleanupFullScreenOverlay' is called before the world actual destruction
        mVirtualRealityHeadsetOverlay->render();
      }
      wr_viewport_render_overlays(wr_scene_get_viewport(wr_scene_get_instance()));
      WbWrenWindow::blitMainFrameBufferToScreen();
      WbWrenOpenGlContext::instance()->swapBuffers(this);
      WbWrenOpenGlContext::doneWren();
    } else
#endif
      WbWrenWindow::renderNow(culling);
    mLastRefreshTimer.start();
    emit mainRenderingEnded(mPhysicsRefresh);

    // take screenshot if needed
    if (mScreenshotRequested) {
      mScreenshotRequested = false;
      emit screenshotReady(grabWindowBufferNow());
    }
  }
}

void WbView3D::remoteMouseEvent(QMouseEvent *event) {
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      mousePressEvent(event);
      break;
    case QEvent::MouseButtonRelease:
      mouseReleaseEvent(event);
      break;
    case QEvent::MouseMove:
      mouseMoveEvent(event);
      break;
    default:
      break;
  }
}

void WbView3D::remoteWheelEvent(QWheelEvent *event) {
  wheelEvent(event);
}

void WbView3D::selectNode(const QMouseEvent *event) {
  if (mSelectionDisabled)
    return;

  // Object selection:
  // - at first click select the top Matter node
  // - at second click on the same geometry select the picked Matter node
  // - further clicks on the same geometry will toggle between picked and top Matter nodes
  // exception in case of context menu shortcut where the selected Matter node is always used
  WbSelection *const selection = WbSelection::instance();
  if (!mPickedMatter) {
    selection->selectTransformFromView3D(NULL);  // sending NULL allows to unselect
    if (isContextMenuShortcut(event) && event->type() == QEvent::MouseButtonRelease)
      emit contextMenuRequested(event->globalPos());
    return;
  }

  const WbAbstractTransform *const selectedTransform = selection->selectedAbstractTransform();
  WbMatter *visiblePickedMatter = WbNodeUtilities::findUpperVisibleMatter(mPickedMatter);
  WbMatter *selectedMatter = NULL;
  if (isContextMenuShortcut(event))
    selectedMatter = visiblePickedMatter;
  else {
    const WbMatter *const previousTopMatter =
      selectedTransform != NULL ? WbNodeUtilities::findUppermostMatter(selectedTransform->baseNode()) : NULL;
    WbMatter *topMatter = WbNodeUtilities::findUppermostMatter(visiblePickedMatter);
    if (topMatter == NULL)
      topMatter = visiblePickedMatter;
    const int alt = event->modifiers() & Qt::AltModifier;
    if (visiblePickedMatter == selectedTransform) {
      if (alt)
        // do not change selection when starting force or torque drag
        return;
      if (topMatter == visiblePickedMatter) {
        // do not change selection if the picked node is already selected and it doesn't have any Matter ancestor
        selection->confirmSelectedTransformFromView3D();
        return;
      }
      selectedMatter = topMatter;
    } else if ((topMatter == previousTopMatter) || alt)
      selectedMatter = visiblePickedMatter;
    else
      selectedMatter = topMatter;
  }

  selection->selectTransformFromView3D(selectedMatter);

  if (WbSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty())
    WbVisualBoundingSphere::instance()->show(selectedMatter);

  if (isContextMenuShortcut(event) && event->type() == QEvent::MouseButtonRelease)
    emit contextMenuRequested(event->globalPos());
}

void WbView3D::mousePressEvent(QMouseEvent *event) {
  mLastButtonState = event->buttons();

  // Ignore if dragging handles
  // e.g. received on macOS when mouse moved outside application window area
  if (mDragTranslate || mDragRotate || (mDragTorque && !mDragTorque->isLocked()) || (mDragForce && !mDragForce->isLocked()) ||
      mDragVerticalAxisRotate)
    return;

  mMouseEventInitialized = true;
  updateMousesPosition(true, false);

  const QPoint &position = event->pos();

  // Overlays come first - special case for overlay resize and close (resize has priority)
  bool displayOverlayClicked = false;
  WbWrenTextureOverlay *overlay = NULL;
  if (!mDragOverlay) {
    WbRenderingDevice *renderingDevice = WbRenderingDevice::fromMousePosition(position.x(), position.y());
    if (renderingDevice) {
      overlay = renderingDevice->overlay();
      if (overlay) {
        displayOverlayClicked = true;

        if (overlay->isInsideResizeArea(position.x(), position.y())) {
          // reset double click timer for resize area
          delete mMousePressTime;
          mMousePressTime = NULL;

          mLastMouseCursor = cursor();
          setCursor(QCursor(Qt::SizeFDiagCursor));

          overlay->putOnTop();
          mDragOverlay = new WbDragResizeOverlayEvent(position, renderingDevice);
          connect(renderingDevice, &QObject::destroyed, this, &WbView3D::abortOverlayDrag);

          return;
        } else if (overlay->isInsideCloseButton(position.x(), position.y())) {
          renderingDevice->toggleOverlayVisibility(false, true);
          displayOverlayClicked = false;

          // reset double click timer on close area
          delete mMousePressTime;
          mMousePressTime = NULL;
          return;
        } else {
          mLastMouseCursor = cursor();
          setCursor(QCursor(Qt::ClosedHandCursor));

          mDragOverlay = new WbDragTranslateOverlayEvent(position, QPoint(width(), height()), renderingDevice);
          connect(renderingDevice, &QObject::destroyed, this, &WbView3D::abortOverlayDrag);
        }
      }
    }
  }

  // if we didn't close an overlay perform double-click check as normal
  if ((event->buttons() == Qt::LeftButton) && mMousePressTime) {
    int delay = mMousePressTime->elapsed();
    if (delay < QApplication::doubleClickInterval()) {
      delete mMousePressTime;
      mMousePressTime = NULL;
      mouseDoubleClick(event);
      return;
    }
  }
  delete mMousePressTime;
  mMousePressTime = new QTime(QTime::currentTime());
  mMousePressTime->start();
  mMousePressPosition = position;
  WbWrenWindow::mousePressEvent(event);

  if (!mWorld)
    return;

  cleanupWheel();

  // if we didn't close an overlay but still clicked on one (without this being
  // a double click), then handle this double click as normal and exit
  if (displayOverlayClicked) {
    overlay->putOnTop();
    return;
  }

  // Overlays come first
  if (!mDragOverlay) {
    WbRenderingDevice *renderingDevice = WbRenderingDevice::fromMousePosition(position.x(), position.y());
    if (renderingDevice) {
      WbWrenTextureOverlay *overlay = renderingDevice->overlay();
      if (overlay) {
        if (overlay->isInsideCloseButton(position.x(), position.y()))
          renderingDevice->toggleOverlayVisibility(false, true);
        else
          overlay->putOnTop();
        renderLater();
        return;
      }
    }
  }

  // clear picked matter, this will be set again later once the picked matter (if any) has been deduced
  mPickedMatter = NULL;

  // Picks the WbNode and retrieves the corresponding WbGeometry
  mWorld->viewpoint()->storePickedCoordinates(WbVector3(0, 0, 0));

  bool picked = mPicker->pick(event->pos().x(), event->pos().y());
  if (picked) {
    const int id = mPicker->selectedId();

    // Check if a transformation handle was picked
    if (id == -1)
      return;

    WbVector3 screenCoords = mPicker->screenCoordinates();
    screenCoords[0] = (screenCoords[0] / width()) * 2 - 1;
    screenCoords[1] = (screenCoords[1] / height()) * 2 - 1;
    WbVector3 center;
    mWorld->viewpoint()->toWorld(screenCoords, center);
    mWorld->viewpoint()->storePickedCoordinates(center);

    mPickedMatter = WbNodeUtilities::findUpperMatter(WbNode::findNode(id));
  } else
    mWorld->viewpoint()->storePickedCoordinates(mWorld->viewpoint()->position()->value());

  if (isContextMenuShortcut(event))
    return;

  // Handle bumpers
  WbTouchSensor *const touchSensor = dynamic_cast<WbTouchSensor *>(mPickedMatter);
  if (touchSensor && touchSensor->deviceType() == WbTouchSensor::BUMPER) {
    touchSensor->setGuiTouch(true);
    mTouchSensor = touchSensor;
    selectNode(event);
  }
}

void WbView3D::leaveEvent(QEvent *event) {
  setCursor(QCursor(Qt::ArrowCursor));
  if (mWheel)
    cleanupWheel();
  cleanupCameraRecognizedObjectsOverlayIfNeeded();
}

void WbView3D::mouseMoveEvent(QMouseEvent *event) {
  if (!mWorld)
    return;

  updateMousesPosition(false, true);

  const QPoint &position = event->pos();

  // do not change cursor shape while dragging an overlay
  if (mDragOverlay) {
    mDragOverlay->apply(position);
    renderLater();
    return;
  }

  // Overlay management comes first
  if (event->buttons() == Qt::NoButton) {
    // no mouse button is pressed
    WbRenderingDevice *const renderingDevice = WbRenderingDevice::fromMousePosition(position.x(), position.y());
    if (renderingDevice && renderingDevice->overlay()) {
      bool resizeArea = false;
      int u, v;
      renderingDevice->overlay()->convertMousePositionToIndex(position.x(), position.y(), u, v, resizeArea);
      if (WbSimulationState::instance()->isPaused()) {
        WbLog::status(renderingDevice->name() + ": " + renderingDevice->pixelInfo(u, v));
        WbCamera *camera = dynamic_cast<WbCamera *>(renderingDevice);
        if (camera) {
          if (mCameraUsingRecognizedObjectsOverlay != camera)
            cleanupCameraRecognizedObjectsOverlayIfNeeded();
          mCameraUsingRecognizedObjectsOverlay = camera;
          camera->updateRecognizedObjectsOverlay((double)position.x() / width(), (double)position.y() / height(), u, v);
          refresh();
        } else
          cleanupCameraRecognizedObjectsOverlayIfNeeded();
      } else
        cleanupCameraRecognizedObjectsOverlayIfNeeded();
      if (resizeArea)
        setCursor(QCursor(Qt::SizeFDiagCursor));
      else if (renderingDevice->overlay()->isInsideCloseButton(position.x(), position.y()))
        setCursor(QCursor(Qt::ArrowCursor));
      else
        setCursor(QCursor(Qt::CrossCursor));
    } else {
      cleanupCameraRecognizedObjectsOverlayIfNeeded();
      setCursor(QCursor(Qt::ArrowCursor));
      if (mDragForce == NULL && mDragTorque == NULL)
        WbLog::status("");
    }

    mLastMouseCursor = cursor();
    return;
  }

  if (!mMouseEventInitialized)
    // return if mouse pressed event was not executed
    return;

  // At least one mouse button is pressed, so a drag event is ongoing or has to be created

  // Checks whether there is an ongoing drag event and update it in this case
  if (mDragResize) {
    mDragResize->apply(position);
    renderLater();
    return;
  }

  if (mDragScale) {
    mDragScale->apply(position);
    renderLater();
    return;
  }

  if (mDragTranslate) {
    mDragTranslate->apply(position);
    renderLater();
    return;
  }

  if (mDragVerticalAxisRotate) {
    mDragVerticalAxisRotate->apply(position);
    renderLater();
    return;
  }

  if (mDragRotate) {
    mDragRotate->apply(position);
    renderLater();
    return;
  }

  if (mDragKinematics) {
    mDragKinematics->apply(position);
    renderLater();
    return;
  }

  if (mDragForce && !mDragForce->isLocked()) {
    mDragForce->apply(position);
    renderLater();
    return;
  } else if (mDragTorque && !mDragTorque->isLocked()) {
    mDragTorque->apply(position);
    renderLater();
    return;
  }

  // Overlays come first
  // Drag overlay even if modifier keys are pressed
  WbRenderingDevice *const renderingDevice = WbRenderingDevice::fromMousePosition(position.x(), position.y());
  if (renderingDevice) {
    WbWrenTextureOverlay *const overlay = renderingDevice->overlay();
    if (overlay) {
      overlay->putOnTop();
      if (overlay->isInsideResizeArea(position.x(), position.y()))
        mDragOverlay = new WbDragResizeOverlayEvent(position, renderingDevice);
      else
        mDragOverlay = new WbDragTranslateOverlayEvent(position, QPoint(width(), height()), renderingDevice);
      connect(renderingDevice, &QObject::destroyed, this, &WbView3D::abortOverlayDrag);
      return;
    }
  }

  WbViewpoint *const viewpoint = mWorld->viewpoint();

  // Translate, rotate, resize events come right after overlays
  const int translateHandle = mPicker->pickedTranslateHandle(), rotateHandle = mPicker->pickedRotateHandle(),
            resizeHandle = mPicker->pickedResizeHandle(), scaleHandle = mPicker->pickedScaleHandle();

  // Creates a new drag event according to keys (SHIFT, ALT) and buttons (LEFT, MIDDLE, RIGHT)
  const int shift = event->modifiers() & Qt::ShiftModifier;
  const int alt = event->modifiers() & Qt::AltModifier;

  int selective = !shift;
  bool resizeActive =
    WbSelection::instance()->resizeManipulatorEnabledFromSceneTree() || (event->modifiers() & Qt::ControlModifier);
  if (resizeHandle && resizeActive) {
    cleanupPhysicsDrags();

    WbBaseNode *pickedNode = WbSelection::instance()->selectedNode();
    WbGeometry *const pickedGeometry = dynamic_cast<WbGeometry *>(pickedNode);

    assert(pickedGeometry);
    if (!pickedGeometry)
      return;

    const int handleNumber = resizeHandle - 1;
    const int geometryType = pickedGeometry->nodeType();
    switch (geometryType) {
      case WB_NODE_SPHERE:
        mDragResize = new WbResizeSphereEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CYLINDER:
        if (selective)
          mDragResize = new WbResizeCylinderEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleCylinderEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CAPSULE:
        if (selective)
          mDragResize = new WbResizeCapsuleEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleCapsuleEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_BOX:
        if (selective)
          mDragResize = new WbResizeBoxEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleBoxEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_PLANE:
        if (selective)
          mDragResize = new WbResizePlaneEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescalePlaneEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_INDEXED_FACE_SET:
        if (selective)
          mDragResize = new WbResizeIndexedFaceSetEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleIndexedFaceSetEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CONE:
        if (selective)
          mDragResize = new WbResizeConeEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleConeEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_ELEVATION_GRID:
        if (selective)
          mDragResize = new WbResizeElevationGridEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleElevationGridEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
    }
    connect(mDragResize, &WbDragResizeHandleEvent::aborted, this, &WbView3D::abortResizeDrag);
    return;
  } else if (scaleHandle && resizeActive) {
    cleanupPhysicsDrags();
    WbBaseNode *pickedNode = WbSelection::instance()->selectedNode();
    WbAbstractTransform *pickedTransform = WbNodeUtilities::abstractTransformCast(pickedNode);
    assert(pickedTransform);
    if (dynamic_cast<WbSolid *>(pickedNode))
      selective = 0;
    const int handleNumber = scaleHandle - 1;

    if (selective)
      mDragScale = new WbDragScaleHandleEvent(position, viewpoint, handleNumber, pickedTransform);
    else
      mDragScale = new WbUniformScaleEvent(position, viewpoint, handleNumber, pickedTransform);

    connect(mDragScale, &WbDragScaleHandleEvent::aborted, this, &WbView3D::abortScaleDrag);
    return;
  }

  if (translateHandle) {
    cleanupPhysicsDrags();
    int handleNumber = translateHandle - 1;
    WbBaseNode *pickedNode = WbSelection::instance()->selectedNode();
    WbSolid *const pickedSolid = dynamic_cast<WbSolid *>(pickedNode);
    if (pickedSolid)
      mDragTranslate = new WbDragTranslateAlongAxisSolidEvent(position, size(), viewpoint, handleNumber, pickedSolid);
    else {
      WbAbstractTransform *pickedTransform = WbNodeUtilities::abstractTransformCast(pickedNode);
      assert(pickedTransform);
      mDragTranslate = new WbDragTranslateAlongAxisEvent(position, size(), viewpoint, handleNumber, pickedTransform);
    }
    return;
  } else if (rotateHandle) {
    cleanupPhysicsDrags();
    const int handleNumber = rotateHandle - 1;
    WbBaseNode *pickedNode = WbSelection::instance()->selectedNode();
    WbSolid *const pickedSolid = dynamic_cast<WbSolid *>(pickedNode);
    if (pickedSolid)
      mDragRotate = new WbDragRotateAroundAxisSolidEvent(position, size(), viewpoint, handleNumber, pickedSolid);
    else {
      WbAbstractTransform *pickedTransform = WbNodeUtilities::abstractTransformCast(pickedNode);
      assert(pickedTransform);
      mDragRotate = new WbDragRotateAroundAxisEvent(position, size(), viewpoint, handleNumber, pickedTransform);
    }
    return;
  }

  // Cases 1 SHIFT + CLICK
  // - LEFT CLICK  -> move the selected solid along horizontal plane
  // - RIGHT CLICK -> rotate the selected solid around world vertical axis
  // - MID CLICK   -> lift the selected solid
  if (shift) {
    selectNode(event);
    const WbSelection *const selection = WbSelection::instance();
    if (!selection->isObjectMotionAllowed())
      return;

    WbBaseNode *const selectedNode = dynamic_cast<WbBaseNode *>(selection->selectedAbstractTransform());
    WbTransform *const uppermostTransform = WbNodeUtilities::findUppermostTransform(selectedNode);
    WbSolid *const uppermostSolid = WbNodeUtilities::findUppermostSolid(selectedNode);
    Qt::MouseButtons buttons = event->buttons();
    if (buttons == Qt::MidButton || buttons == (Qt::LeftButton | Qt::RightButton)) {
      if (uppermostSolid)
        mDragKinematics = new WbDragVerticalSolidEvent(position, viewpoint, uppermostSolid);
      else
        mDragKinematics = new WbDragVerticalEvent(position, viewpoint, uppermostTransform);
    } else if (buttons == Qt::LeftButton) {
      if (uppermostSolid)
        mDragKinematics = new WbDragHorizontalSolidEvent(position, viewpoint, uppermostSolid);
      else
        mDragKinematics = new WbDragHorizontalEvent(position, viewpoint, uppermostTransform);
    } else if (buttons == Qt::RightButton) {
      if (uppermostSolid)
        mDragVerticalAxisRotate = new WbDragRotateAroundWorldVerticalAxisSolidEvent(position, viewpoint, uppermostSolid);
      else
        mDragVerticalAxisRotate = new WbDragRotateAroundWorldVerticalAxisEvent(position, viewpoint, uppermostTransform);
    }
  } else if (alt) {  // Case 2: ALT and CLICK -> add a force / torque to the selected solid
    WbSolid *const selectedSolid = dynamic_cast<WbSolid *>(mPickedMatter);
    if (!selectedSolid || selectedSolid->bodyMerger() == NULL)
      return;
    Qt::MouseButtons buttons = event->buttons();
    bool forceButtonPressed = buttons == Qt::LeftButton;
#ifdef __APPLE__
    bool torqueButtonPressed =
      buttons == Qt::RightButton || (buttons == Qt::LeftButton && event->modifiers() & Qt::MetaModifier);
#else
    bool torqueButtonPressed = buttons == Qt::RightButton;
#endif
    if (torqueButtonPressed || forceButtonPressed) {
      if (mDragTorque) {
        if (mDragTorque->isLocked()) {
          delete mDragTorque;
          mDragTorque = NULL;
        }
      }

      if (mDragForce) {
        if (mDragForce->isLocked()) {
          delete mDragForce;
          mDragForce = NULL;
        }
      }

      if (!mDragTorque && torqueButtonPressed) {
        WbSelection::instance()->disableActiveManipulator();
        mDragTorque = new WbDragTorqueEvent(size(), viewpoint, selectedSolid);
        connect(mDragTorque, &WbDragTorqueEvent::aborted, this, &WbView3D::abortPhysicsDrag);
        connect(mDragTorque, &WbDragTorqueEvent::destroyed, WbSelection::instance(), &WbSelection::restoreActiveManipulator);
      } else if (!mDragForce && forceButtonPressed) {
        WbSelection::instance()->disableActiveManipulator();
        mDragForce = new WbDragForceEvent(size(), viewpoint, selectedSolid);
        connect(mDragForce, &WbDragForceEvent::aborted, this, &WbView3D::abortPhysicsDrag);
        connect(mDragForce, &WbDragForceEvent::destroyed, WbSelection::instance(), &WbSelection::restoreActiveManipulator);
      }
    }
  } else if (!mViewpointLocked) {  // Case 3: CLICK only -> move the camera
    Qt::MouseButtons buttons = event->buttons();

    // For zoom and translation, we need the distance to the clicked object, if any.
    double distanceToPickPosition;
    if (mPicker->selectedId() != -1)
      distanceToPickPosition = (viewpoint->position()->value() - viewpoint->rotationCenter()).length();
    else
      distanceToPickPosition = viewpoint->position()->value().length();

    if (distanceToPickPosition < 0.001)
      distanceToPickPosition = 0.001;

    double scale = distanceToPickPosition * 2 * tan(viewpoint->fieldOfView()->value() / 2) / std::max(width(), height());

#ifdef __APPLE__
    if (buttons == Qt::RightButton || (buttons == Qt::LeftButton && event->modifiers() & Qt::MetaModifier))
#else
    if (buttons == Qt::RightButton)
#endif
      mDragKinematics = new WbTranslateViewpointEvent(position, viewpoint, scale);
    else if (buttons == Qt::MidButton || buttons == (Qt::LeftButton | Qt::RightButton))
      mDragKinematics = new WbZoomAndRotateViewpointEvent(position, viewpoint, 5 * scale);
    else if (buttons == Qt::LeftButton)
      mDragKinematics = new WbRotateViewpointEvent(position, viewpoint, mPicker->selectedId() != -1);
  }
}

void WbView3D::mouseDoubleClick(QMouseEvent *event) {
  if (!mWorld)
    return;

  const QPoint &mousePosition = event->pos();

  // Overlays come first
  // open external window
  WbRenderingDevice *const renderingDevice = WbRenderingDevice::fromMousePosition(mousePosition.x(), mousePosition.y());
  if (renderingDevice) {
    WbRenderingDeviceWindowFactory::instance()->showWindowForDevice(renderingDevice);
    return;
  }

  if (mSelectionDisabled)
    return;

  const bool picked = mPicker->pick(mousePosition.x(), mousePosition.y());
  if (picked) {
    const int id = mPicker->selectedId();
    if (id == -1)
      return;

    emit mouseDoubleClicked(event);

    WbNode *node = WbNode::findNode(id);
    WbRobot *pickedRobot = dynamic_cast<WbRobot *>(node);
    if (pickedRobot == NULL && node != NULL)
      pickedRobot = WbNodeUtilities::findRobotAncestor(node);
    if (pickedRobot) {
      mPickedMatter = pickedRobot;
      emit showRobotWindowRequest();
    } else
      mPickedMatter = WbNodeUtilities::findUpperMatter(node);
  }
}

bool WbView3D::isContextMenuShortcut(const QMouseEvent *event) {
#ifdef __APPLE__
  return (event->button() == Qt::RightButton && event->modifiers() == Qt::NoModifier) ||
         (event->button() == Qt::LeftButton && event->modifiers() & Qt::MetaModifier);
#else
  return event->button() == Qt::RightButton && (event->modifiers() == Qt::NoModifier);
#endif
}

void WbView3D::mouseReleaseEvent(QMouseEvent *event) {
  WbWrenWindow::mouseReleaseEvent(event);

  mLastButtonState = event->buttons();

  if (!mMouseEventInitialized)
    // mouse press event handled by another widget
    return;

  updateMousesPosition(true, false);

  setCursor(mLastMouseCursor);

  const bool wasNotInAnEvent = !mDragOverlay && !mDragKinematics && !mDragResize && !mDragScale && !mDragTranslate &&
                               !mDragVerticalAxisRotate && !mDragRotate && !mDragForce && !mDragTorque && !mTouchSensor;

  delete mDragOverlay;
  mDragOverlay = NULL;

  delete mDragKinematics;
  mDragKinematics = NULL;

  if (mDragResize) {
    mDragResize->addActionInUndoStack();
    delete mDragResize;
    mDragResize = NULL;
    if (mResizeHandlesDisabled)
      WbSelection::instance()->showResizeManipulatorFromView3D(false);
  }

  if (mDragScale) {
    mDragScale->addActionInUndoStack();
    delete mDragScale;
    mDragScale = NULL;
    if (mResizeHandlesDisabled)
      WbSelection::instance()->showResizeManipulatorFromView3D(false);
  }

  if (mDragTranslate) {
    delete mDragTranslate;
    mDragTranslate = NULL;
  }

  if (mDragVerticalAxisRotate) {
    delete mDragVerticalAxisRotate;
    mDragVerticalAxisRotate = NULL;
  }

  if (mDragRotate) {
    delete mDragRotate;
    mDragRotate = NULL;
  }

  const WbSimulationState *const sim = WbSimulationState::instance();
  if (sim->isPaused()) {
    if (mDragForce && !mDragForce->isLocked())
      mDragForce->lock();
    if (mDragTorque && !mDragTorque->isLocked())
      mDragTorque->lock();
  } else {
    delete mDragForce;
    mDragForce = NULL;
    delete mDragTorque;
    mDragTorque = NULL;
  }
  if (mTouchSensor) {
    mTouchSensor->setGuiTouch(false);
    mTouchSensor = NULL;
  }

  renderLater();

  if (wasNotInAnEvent)
    selectNode(event);
  else if (mMousePressTime) {  // test if we did a quick button press and release, possibly moving only slightly the mouse
    const int delay = mMousePressTime->elapsed();
    if (delay < QApplication::doubleClickInterval()) {  // the mouse button was released quickly after being pressed
      const QPoint diff = mMousePressPosition - event->pos();
      if (diff.manhattanLength() < 20)  // the mouse was moved by less than 20 pixels (determined empirically)
        selectNode(event);
    }
  }

  mPickedMatter = NULL;
  mMouseEventInitialized = false;
}

void WbView3D::handleModifierKey(QKeyEvent *event, bool pressed) {
  if (event->key() == Qt::Key_Control)
    enableResizeManipulator(pressed);
  else if (event->key() == Qt::Key_Shift)
    WbSelection::instance()->setUniformConstraintForResizeHandles(pressed);
}

void WbView3D::keyPressEvent(QKeyEvent *event) {
  // handle event in parent class
  if (event->key() == Qt::Key_Escape ||
      (event->modifiers() == Qt::CTRL && event->key() >= Qt::Key_0 && event->key() <= Qt::Key_4)) {
    QWindow::keyPressEvent(event);
    return;
  }

  // pass key event to robots if appropriate
  const int modifiers = (((event->modifiers() & Qt::SHIFT) == 0) ? 0 : WbRobot::mapSpecialKey(Qt::SHIFT)) +
                        (((event->modifiers() & Qt::CTRL) == 0) ? 0 : WbRobot::mapSpecialKey(Qt::CTRL)) +
                        (((event->modifiers() & Qt::ALT) == 0) ? 0 : WbRobot::mapSpecialKey(Qt::ALT));

  WbRobot *const currentRobot = getCurrentRobot();
  QList<WbRobot *> robotList;
  if (currentRobot)
    robotList.append(currentRobot);
  else
    robotList = mWorld->robots();

  foreach (WbRobot *robot, robotList)
    robot->keyPressed(event->text(), event->key(), modifiers);
  handleModifierKey(event, true);
  QWindow::keyPressEvent(event);
}

void WbView3D::keyReleaseEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Shift)
    cleanupWheel();

  // pass key event to robots
  if (mWorld) {
    WbRobot *const currentRobot = getCurrentRobot();
    QList<WbRobot *> robotList;
    if (currentRobot)
      robotList.append(currentRobot);
    else
      robotList = mWorld->robots();

    foreach (WbRobot *const robot, robotList)
      robot->keyReleased(event->text(), event->key());
  }
  handleModifierKey(event, false);
  QWindow::keyReleaseEvent(event);
}

void WbView3D::enableResizeManipulator(bool enabled) {
  if (enabled && WbSelection::instance()->showResizeManipulatorFromView3D(true))
    mResizeHandlesDisabled = false;
  else {
    if (mDragResize || mDragScale)
      mResizeHandlesDisabled = true;
    else
      WbSelection::instance()->showResizeManipulatorFromView3D(false);
  }
}

WbRobot *WbView3D::getCurrentRobot() const {
  if (!WbSelection::instance() || !WbSelection::instance()->selectedSolid())
    return NULL;

  WbRobot *const robot = WbSelection::instance()->selectedSolid()->robot();
  if (robot)
    return robot;

  const QList<WbRobot *> &robotList = mWorld->robots();
  if (robotList.size() == 1)
    return robotList.first();

  return NULL;
}

void WbView3D::wheelEvent(QWheelEvent *event) {
  if (!mWorld)
    return;

#ifndef __APPLE__  // bug in qt on Mac: -> QWheelEvent->orientation() is wrong when SHIFT + MOUSE_WHEEL_VERTICAL_SCROLL
  // Some mouse wheels can be scrolled horizontally
  if (event->orientation() != Qt::Vertical)
    return;
#endif

  WbViewpoint *const viewpoint = mWorld->viewpoint();
  if (event->modifiers() & Qt::ShiftModifier) {
    if (mWheel) {
      mWheel->apply(event->delta());
      renderLater();
      return;
    }
    // SHIFT and WHEEL MOUSE -> lift the selected solid in the 3D View
    WbBaseNode *const selectedNode = dynamic_cast<WbBaseNode *>(WbSelection::instance()->selectedAbstractTransform());
    WbSolid *const uppermostSolid = WbNodeUtilities::findUppermostSolid(selectedNode);
    if (!uppermostSolid || uppermostSolid->isLocked())
      return;
    mWheel = new WbWheelLiftSolidEvent(viewpoint, uppermostSolid);
    mWheel->apply(event->delta());
    renderLater();
  } else if (!mViewpointLocked) {
    // WHEEL MOUSE only -> zoom
    if (mProjectionMode == WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC) {
      if (event->delta() > 0)
        viewpoint->decOrthographicViewHeight();
      else
        viewpoint->incOrthographicViewHeight();
    }

    double distanceToPickPosition;
    const QPoint mousePosition = mapFromGlobal(QCursor::pos());
    if (mousePosition.x() < 0 || mousePosition.y() < 0 || mousePosition.x() >= width() || mousePosition.y() >= height())
      distanceToPickPosition = viewpoint->position()->value().length();
    else {
      if (mPicker->selectedId() != -1)
        distanceToPickPosition = (viewpoint->position()->value() - viewpoint->rotationCenter()).length();
      else
        distanceToPickPosition = viewpoint->position()->value().length();
      if (distanceToPickPosition < 0.001)
        distanceToPickPosition = 0.001;
    }

    const double scaleFactor = -0.02 * (event->delta() < 0.0 ? -1 : 1) * distanceToPickPosition;
    const WbVector3 zDisplacement(scaleFactor * viewpoint->orientation()->value().direction());
    WbSFVector3 *const position = viewpoint->position();
    position->setValue(position->value() + zDisplacement);
    if (!zDisplacement.isNull())
      mWorld->setModified();
    renderLater();
  }
}

// Cleanup methods

void WbView3D::cleanupEvents() {
  cleanupWheel();
  cleanupDrags();
}

void WbView3D::cleanupOptionalRendering() {
  delete mContactPointsRepresentation;
  mContactPointsRepresentation = NULL;
}

void WbView3D::cleanupWheel() {
  delete mWheel;
  mWheel = NULL;
}

void WbView3D::cleanupCameraRecognizedObjectsOverlayIfNeeded() {
  if (mCameraUsingRecognizedObjectsOverlay) {
    mCameraUsingRecognizedObjectsOverlay->clearRecognizedObjectsOverlay();
    mCameraUsingRecognizedObjectsOverlay = NULL;
    refresh();
  }
}

void WbView3D::cleanupDrags() {
  delete mDragOverlay;
  mDragOverlay = NULL;

  delete mDragKinematics;
  mDragKinematics = NULL;

  delete mDragResize;
  mDragResize = NULL;

  delete mDragScale;
  mDragScale = NULL;

  delete mDragTranslate;
  mDragTranslate = NULL;

  delete mDragVerticalAxisRotate;
  mDragVerticalAxisRotate = NULL;

  delete mDragRotate;
  mDragRotate = NULL;

  cleanupPhysicsDrags();
}

void WbView3D::abortPhysicsDrag() {
  cleanupPhysicsDrags();
  WbSelection::instance()->selectTransformFromView3D(NULL);
  WbLog::warning(tr("Solid out of world numeric bounds, mouse drag aborted"));
}

void WbView3D::abortResizeDrag() {
  delete mDragResize;
  mDragResize = NULL;
  WbSelection::instance()->selectTransformFromView3D(NULL);
  WbLog::warning(tr("The dimensions of the resized object exceeds world numeric bounds, mouse drag aborted"));
  if (mResizeHandlesDisabled)
    WbSelection::instance()->showResizeManipulatorFromView3D(false);
}

void WbView3D::abortScaleDrag() {
  delete mDragScale;
  mDragScale = NULL;
  WbSelection::instance()->selectTransformFromView3D(NULL);
  WbLog::warning(tr("The dimensions of the rescaled object exceeds world numeric bounds, mouse drag aborted"));
  if (mResizeHandlesDisabled)
    WbSelection::instance()->showResizeManipulatorFromView3D(false);
}

void WbView3D::abortOverlayDrag() {
  delete mDragOverlay;
  mDragOverlay = NULL;
}

void WbView3D::cleanupPhysicsDrags() {
  delete mDragForce;
  mDragForce = NULL;

  delete mDragTorque;
  mDragTorque = NULL;
}

void WbView3D::cleanupPickers() {
  delete mPicker;
  delete mControllerPicker;
  mPicker = NULL;
  mControllerPicker = NULL;
  mPickedMatter = NULL;
}

void WbView3D::unleashAndClean() {
  if (mDragForce) {
    mDragForce->applyToOde();
    delete mDragForce;
    mDragForce = NULL;
  }

  if (mDragTorque) {
    mDragTorque->applyToOde();
    delete mDragTorque;
    mDragTorque = NULL;
  }

  if (mDragForce || mDragTorque)
    renderLater();
}

void WbView3D::unleashPhysicsDrags() {
  const WbSimulationState *const sim = WbSimulationState::instance();
  if (sim->isPaused())
    return;

  unleashAndClean();
}
// Fast mode related methods

void WbView3D::rescaleFastModePanel() {
  mFastModeOverlay->adjustSize();
}

void WbView3D::showFastModeOverlay() {
  if (!mWorld || mFastModeOverlay->isVisible())
    return;

  disconnect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this, &WbView3D::refresh);

  rescaleFastModePanel();
  mFastModeOverlay->setVisible(true);

  mParentWidget->setEnabled(false);
  renderLater();

  WbRenderingDeviceWindowFactory::instance()->setWindowsEnabled(false);

  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::hideFastModeOverlay() {
  if (!mWorld || !mFastModeOverlay->isVisible())
    return;

  connect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this, &WbView3D::refresh,
          Qt::UniqueConnection);

  mFastModeOverlay->setVisible(false);

  mParentWidget->setEnabled(true);
  renderLater();

  WbRenderingDeviceWindowFactory::instance()->setWindowsEnabled(true);

  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::cleanupFullScreenOverlay() {
  delete mFastModeOverlay;
  mFastModeOverlay = NULL;
  delete mVirtualRealityHeadsetOverlay;
  mVirtualRealityHeadsetOverlay = NULL;
  delete mLoadingWorldOverlay;
  mLoadingWorldOverlay = NULL;
}

void WbView3D::updateVirtualRealityHeadsetOverlay() {
  if (!mWorld || !mVirtualRealityHeadsetOverlay)
    return;

  if (mFastModeOverlay->isVisible()) {
    mVirtualRealityHeadsetOverlay->setVisible(false);
    return;
  }

#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse()) {
    mVirtualRealityHeadsetOverlay->setVisible(true);
    mVirtualRealityHeadsetOverlay->setExternalTexture(WbVirtualRealityHeadset::instance()->visibleTexture());
    mParentWidget->setEnabled(false);
  } else {
#endif
    mVirtualRealityHeadsetOverlay->setVisible(false);
    mParentWidget->setEnabled(true);
#ifdef _WIN32
  }
#endif

  renderLater();
}

void WbView3D::handleWorldModificationFromSupervior() {
  // refresh only if simulation is paused (or stepped)
  const WbSimulationState *const sim = WbSimulationState::instance();
  if (sim->isPaused())
    refresh();
}
