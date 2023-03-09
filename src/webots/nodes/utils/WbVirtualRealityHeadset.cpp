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

#include "WbVirtualRealityHeadset.hpp"

#include "WbBackground.hpp"
#include "WbLensFlare.hpp"
#include "WbLog.hpp"
#include "WbMatrix4.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPreferences.hpp"
#include "WbRgb.hpp"
#include "WbSimulationState.hpp"
#include "WbWorld.hpp"
#include "WbWrenHdr.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"
#include "WbWrenSmaa.hpp"

#include <wren/camera.h>
#include <wren/frame_buffer.h>
#include <wren/node.h>
#include <wren/post_processing_effect.h>
#include <wren/scene.h>
#include <wren/texture.h>
#include <wren/texture_rtt.h>
#include <wren/transform.h>
#include <wren/viewport.h>

#include <openvr.h>
#include <openvr_capi.h>

#include <QtCore/QFile>
#include <QtCore/QStandardPaths>
#include <QtCore/QTimer>

static WrTexture *gLoadingTexture = NULL;

void WbVirtualRealityHeadset::setLoadingTexture(WrTexture *texture) {
  gLoadingTexture = texture;
  if (WbPreferences::instance()->value("VirtualRealityHeadset/enable").toBool() &&
      WbVirtualRealityHeadset::isSteamVRInstalled() && WbVirtualRealityHeadset::isHeadsetConnected()) {
    WbVirtualRealityHeadset *virtualRealityHeadset = WbVirtualRealityHeadset::instance();
    if (!virtualRealityHeadset->isValid()) {
      delete virtualRealityHeadset;
      virtualRealityHeadset = NULL;
    }
  }
}

WbVirtualRealityHeadset *WbVirtualRealityHeadset::cInstance = NULL;

WbVirtualRealityHeadset *WbVirtualRealityHeadset::instance() {
  if (!cInstance and WbVirtualRealityHeadset::isSteamVRInstalled() and WbVirtualRealityHeadset::isHeadsetConnected())
    cInstance = new WbVirtualRealityHeadset();
  return cInstance;
}

void WbVirtualRealityHeadset::cleanup() {
  if (cInstance) {
    delete cInstance;
    cInstance = NULL;
  }
}

bool WbVirtualRealityHeadset::isSteamVRInstalled() {
  // We manually check that openvrpaths.vrpat exists before calling 'VR_IsRuntimeInstalled',
  // otherwise OpenVR displays a warning if the file doesn't exists (i.e. SteamVR is not installed).
  // Warning: on macOS it will be '/.openvr/openvrpaths.vrpath'
  if (!QFile::exists(QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation) + "/openvr/openvrpaths.vrpath"))
    return false;
  return vr::VR_IsRuntimeInstalled();
}

bool WbVirtualRealityHeadset::isHeadsetConnected() {
  return vr::VR_IsHmdPresent();
}

bool WbVirtualRealityHeadset::isInUse() {
  return cInstance != NULL;
}

WbVirtualRealityHeadset::WbVirtualRealityHeadset() :
  mViewpointPosition(WbVector3(0.0, 0.0, 0.0)),
  mViewpointOrientation(WbRotation(0.0, 0.0, 1.0, 0.0)) {
  mSystem = NULL;
  mWrenCameras[LEFT] = NULL;
  mWrenCameras[RIGHT] = NULL;
  mWrenViewports[LEFT] = NULL;
  mWrenViewports[RIGHT] = NULL;
  mWrenFrameBuffer[LEFT] = NULL;
  mWrenFrameBuffer[RIGHT] = NULL;
  mLeftEyeSmaa = NULL;
  mRightEyeSmaa = NULL;
  mLeftEyeHdr = NULL;
  mRightEyeHdr = NULL;
  mFieldOfViewY[0] = 0.0;
  mFieldOfViewY[1] = 0.0;
  mAspectRatio[0] = 0.0;
  mAspectRatio[1] = 0.0;

  mTextureBounds = new vr::VRTextureBounds_t;
  mTextureReferences[LEFT] = new vr::Texture_t;
  mTextureReferences[RIGHT] = new vr::Texture_t;
  mOverlayHandle = NULL;

  mTimer = new QTimer(this);

  mTrackPosition = WbPreferences::instance()->value("VirtualRealityHeadset/trackPosition").toBool();
  mTrackOrientation = WbPreferences::instance()->value("VirtualRealityHeadset/trackOrientation").toBool();
  if (WbPreferences::instance()->value("VirtualRealityHeadset/visibleEye").toString() == "left")
    mDisplayedEye = LEFT;
  else if (WbPreferences::instance()->value("VirtualRealityHeadset/visibleEye").toString() == "right")
    mDisplayedEye = RIGHT;
  else
    mDisplayedEye = NONE;

  if (!vr::VR_IsRuntimeInstalled())
    return;

  if (!vr::VR_IsHmdPresent())
    return;

  // Error handeling variable
  vr::HmdError hmdError;

  // Initialize OpenVR
  mSystem = vr::VR_Init(&hmdError, vr::EVRApplicationType::VRApplication_Scene);

  // Check for errors during initialization
  switch (hmdError) {
    case vr::VRInitError_Init_HmdNotFound:
    case vr::VRInitError_Init_HmdNotFoundPresenceFailed:
      WbLog::warning(tr("Cannot find virtual reality headset. Please install SteamVR, launch it, and check virtual reality "
                        "headset USB and HDMI connection."));
      mSystem = NULL;
    case vr::VRInitError_None:
      break;
    default:
      WbLog::warning(
        tr("Error while initializing the virtual reality headset: '%1'").arg(vr::VR_GetVRInitErrorAsSymbol(hmdError)));
      mSystem = NULL;
  }

  // Check if VRCompositor is present
  if (!vr::VRCompositor()) {
    WbLog::warning(tr("Error while initializing the virtual reality headset compositor system."));
    mSystem = NULL;
  }

  // Check if VRCompositor is present
  if (!vr::VROverlay()) {
    WbLog::warning(tr("Error while initializing the virtual reality headset overlay system."));
    mSystem = NULL;
  }

  if (mSystem) {
    mCurrentPosition = WbVector3(0.0, 0.0, 0.0);
    mCurrentOrientation.setIdentity();
    // initializeTextureOverlay();
  } else
    vr::VR_Shutdown();
}

WbVirtualRealityHeadset::~WbVirtualRealityHeadset() {
  deleteWrenObjects();

  delete mTextureBounds;
  delete mTextureReferences[LEFT];
  delete mTextureReferences[RIGHT];
  delete mTimer;
  if (mSystem) {
    vr::VRCompositor()->ClearLastSubmittedFrame();
    vr::VR_Shutdown();
  }
}

bool WbVirtualRealityHeadset::isValid() const {
  return mSystem != NULL;
}

void WbVirtualRealityHeadset::initializeTextureOverlay() {
  if (!mOverlayHandle && gLoadingTexture) {
    mOverlayHandle = new vr::VROverlayHandle_t;
    vr::VROverlay()->CreateOverlay("load", "loading", mOverlayHandle);
    vr::Texture_t *overlayTexture = new vr::Texture_t;
    *overlayTexture = {reinterpret_cast<void *>(wr_texture_get_gl_name(gLoadingTexture)), vr::TextureType_OpenGL,
                       vr::ColorSpace_Gamma};
    vr::VROverlay()->SetOverlayTexture(*mOverlayHandle, overlayTexture);
    vr::VROverlay()->SetOverlayWidthInMeters(*mOverlayHandle, 0.5f);
    vr::HmdMatrix34_t notificationTransform = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -0.3f};
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(*mOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd,
                                                              &notificationTransform);
  }
}

void WbVirtualRealityHeadset::setTextureOverlayVisible(bool visible) {
  if (mOverlayHandle) {
    if (visible)
      vr::VROverlay()->ShowOverlay(*mOverlayHandle);
    else
      vr::VROverlay()->HideOverlay(*mOverlayHandle);
  }
}

void WbVirtualRealityHeadset::createWrenObjects(WrTransform *node, bool antiAliasing) {
  if (mSystem) {
    // create the texture to send to the VR system
    unsigned int recommendedWidth, recommendedHeight;
    mSystem->GetRecommendedRenderTargetSize(&recommendedWidth, &recommendedHeight);

    WbWrenOpenGlContext::makeWrenCurrent();

    // first camera for left eye
    mWrenCameras[LEFT] = wr_camera_new();
    wr_transform_attach_child(node, WR_NODE(mWrenCameras[LEFT]));
    mWrenViewports[LEFT] = wr_viewport_new();
    wr_viewport_set_camera(mWrenViewports[LEFT], mWrenCameras[LEFT]);
    wr_viewport_set_visibility_mask(mWrenViewports[LEFT], WbWrenRenderingContext::instance()->visibilityMask());

    WrTextureRtt *texture = wr_texture_rtt_new();
    wr_texture_set_internal_format(WR_TEXTURE(texture), WR_TEXTURE_INTERNAL_FORMAT_RGBA16F);

    mWrenFrameBuffer[LEFT] = wr_frame_buffer_new();
    wr_frame_buffer_set_size(mWrenFrameBuffer[LEFT], recommendedWidth, recommendedHeight);
    wr_frame_buffer_append_output_texture(mWrenFrameBuffer[LEFT], texture);

    texture = wr_texture_rtt_new();
    wr_texture_set_internal_format(WR_TEXTURE(texture), WR_TEXTURE_INTERNAL_FORMAT_RGB8);
    wr_frame_buffer_append_output_texture(mWrenFrameBuffer[LEFT], texture);

    wr_frame_buffer_enable_depth_buffer(mWrenFrameBuffer[LEFT], true);
    wr_frame_buffer_setup(mWrenFrameBuffer[LEFT]);
    wr_viewport_set_frame_buffer(mWrenViewports[LEFT], mWrenFrameBuffer[LEFT]);

    // second camera for right eye
    mWrenCameras[RIGHT] = wr_camera_new();
    wr_transform_attach_child(node, WR_NODE(mWrenCameras[RIGHT]));
    mWrenViewports[RIGHT] = wr_viewport_new();
    wr_viewport_set_camera(mWrenViewports[RIGHT], mWrenCameras[RIGHT]);
    wr_viewport_set_visibility_mask(mWrenViewports[RIGHT], WbWrenRenderingContext::instance()->visibilityMask());

    texture = wr_texture_rtt_new();
    wr_texture_set_internal_format(WR_TEXTURE(texture), WR_TEXTURE_INTERNAL_FORMAT_RGBA16F);

    mWrenFrameBuffer[RIGHT] = wr_frame_buffer_new();
    wr_frame_buffer_set_size(mWrenFrameBuffer[RIGHT], recommendedWidth, recommendedHeight);
    wr_frame_buffer_append_output_texture(mWrenFrameBuffer[RIGHT], texture);

    texture = wr_texture_rtt_new();
    wr_texture_set_internal_format(WR_TEXTURE(texture), WR_TEXTURE_INTERNAL_FORMAT_RGB8);
    wr_frame_buffer_append_output_texture(mWrenFrameBuffer[RIGHT], texture);

    wr_frame_buffer_enable_depth_buffer(mWrenFrameBuffer[RIGHT], true);
    wr_frame_buffer_setup(mWrenFrameBuffer[RIGHT]);
    wr_viewport_set_frame_buffer(mWrenViewports[RIGHT], mWrenFrameBuffer[RIGHT]);

    mLeftEyeSmaa = new WbWrenSmaa();
    mRightEyeSmaa = new WbWrenSmaa();

    if (antiAliasing) {
      mLeftEyeSmaa->setup(mWrenViewports[LEFT]);
      mRightEyeSmaa->setup(mWrenViewports[RIGHT]);
    }

    mLeftEyeHdr = new WbWrenHdr();
    mRightEyeHdr = new WbWrenHdr();

    mLeftEyeHdr->setup(mWrenViewports[LEFT]);
    mRightEyeHdr->setup(mWrenViewports[RIGHT]);

    WbWrenOpenGlContext::doneWren();

    const WbBackground *const background = WbBackground::firstInstance();
    if (background) {
      float color[] = {static_cast<float>(background->skyColor().red()), static_cast<float>(background->skyColor().green()),
                       static_cast<float>(background->skyColor().blue())};
      wr_viewport_set_clear_color_rgb(mWrenViewports[LEFT], color);
      wr_viewport_set_clear_color_rgb(mWrenViewports[RIGHT], color);
    }

    // Set the OpenGL texture geometry
    *mTextureBounds = {0.0, 0.0, 1.0, 1.0};
    *mTextureReferences[LEFT] = {reinterpret_cast<void *>(wr_texture_get_gl_name(
                                   WR_TEXTURE(wr_frame_buffer_get_output_texture(mWrenFrameBuffer[LEFT], 1)))),
                                 vr::TextureType_OpenGL, vr::ColorSpace_Gamma};
    *mTextureReferences[RIGHT] = {reinterpret_cast<void *>(wr_texture_get_gl_name(
                                    WR_TEXTURE(wr_frame_buffer_get_output_texture(mWrenFrameBuffer[RIGHT], 1)))),
                                  vr::TextureType_OpenGL, vr::ColorSpace_Gamma};

    setEyeView(mDisplayedEye);

    // get vertical field of view, aspect ratio and rotation for each eye
    float pfLeft, pfRight, pfTop, pfBottom;
    mSystem->GetProjectionRaw(vr::Eye_Left, &pfLeft, &pfRight, &pfTop, &pfBottom);
    mFieldOfViewY[LEFT] = 2.0 * atan((pfRight - pfLeft) / 2.0);
    mAspectRatio[LEFT] = (pfBottom - pfTop) / (pfRight - pfLeft);
    mEyeRotation[LEFT] = WbMatrix3(WbVector3(0.0, 1.0, 0.0), -atan(pfRight) - atan(pfLeft));
    mSystem->GetProjectionRaw(vr::Eye_Right, &pfLeft, &pfRight, &pfTop, &pfBottom);
    mFieldOfViewY[RIGHT] = 2.0 * atan((pfRight - pfLeft) / 2.0);
    mAspectRatio[RIGHT] = (pfBottom - pfTop) / (pfRight - pfLeft);
    mEyeRotation[RIGHT] = WbMatrix3(WbVector3(0.0, 1.0, 0.0), -atan(pfRight) - atan(pfLeft));

    // get translation for each eye due to IPD (Inter Pupilar Distance)
    // workaround to fix crash of 'GetEyeToHeadTransform' with mingw (https://github.com/ValveSoftware/openvr/issues/133)
    typedef void (vr::IVRSystem::*GetEyeToHeadTransformFuncPtr)(vr::HmdMatrix34_t *, vr::EVREye);
    GetEyeToHeadTransformFuncPtr get_eye_to_head_transform =
      reinterpret_cast<GetEyeToHeadTransformFuncPtr>(&vr::IVRSystem::GetEyeToHeadTransform);
    vr::HmdMatrix34_t transform;
    (mSystem->*get_eye_to_head_transform)(&transform, vr::Eye_Left);
    mEyeTranslation[LEFT] = WbVector3(transform.m[0][3], transform.m[1][3], transform.m[2][3]);
    (mSystem->*get_eye_to_head_transform)(&transform, vr::Eye_Right);
    mEyeTranslation[RIGHT] = WbVector3(transform.m[0][3], transform.m[1][3], transform.m[2][3]);

    // We should ideally do this in the constructor, but it seems too early for steamVR to handle it
    initializeTextureOverlay();
    setTextureOverlayVisible(true);

    applyAspectRatioToWren();
    applyFieldOfViewToWren();
    applyProjectionModeToWren();

    connect(mTimer, &QTimer::timeout, this, &WbVirtualRealityHeadset::renderRequired);

    WbSimulationState *simulationState = WbSimulationState::instance();
    connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbVirtualRealityHeadset::updateTimer);
    if (simulationState->isPaused() || simulationState->isStep())
      mTimer->start(1000.0 / WbWorld::instance()->worldInfo()->fps());
  }
}

WrTexture *WbVirtualRealityHeadset::visibleTexture() const {
  if (mSystem && mDisplayedEye != NONE)
    return WR_TEXTURE(wr_frame_buffer_get_output_texture(mWrenFrameBuffer[mDisplayedEye], 0));
  else
    return NULL;
}

void WbVirtualRealityHeadset::deleteWrenObjects() {
  delete mLeftEyeSmaa;
  delete mRightEyeSmaa;

  mLeftEyeSmaa = mRightEyeSmaa = NULL;

  delete mLeftEyeHdr;
  delete mRightEyeHdr;

  mLeftEyeHdr = mRightEyeHdr = NULL;

  if (mSystem) {
    for (int i = 0; i < EYE_NUMBER; ++i) {
      if (mWrenFrameBuffer[i]) {
        WrTextureRtt *texture = wr_frame_buffer_get_output_texture(mWrenFrameBuffer[i], 0);
        wr_frame_buffer_delete(mWrenFrameBuffer[i]);
        wr_texture_delete(WR_TEXTURE(texture));
      }
      if (mWrenViewports[i])
        wr_viewport_delete(mWrenViewports[i]);
      if (mWrenCameras[i])
        wr_node_delete(WR_NODE(mWrenCameras[i]));
    }
  }
  for (int i = 0; i < EYE_NUMBER; ++i) {
    mWrenViewports[i] = NULL;
    mWrenCameras[i] = NULL;
    mWrenFrameBuffer[i] = NULL;
  }
}

void WbVirtualRealityHeadset::updateOrientationAndPosition() {
  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::VIRTUAL_REALITY_HEADSET_RENDERING);

  vr::TrackedDevicePose_t trackedPoses[vr::k_unMaxTrackedDeviceCount];
  vr::VRCompositor()->WaitGetPoses(trackedPoses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
  vr::TrackedDevicePose_t hmdPose;
  if ((hmdPose = trackedPoses[vr::k_unTrackedDeviceIndex_Hmd]).bPoseIsValid && (mTrackPosition || mTrackOrientation)) {
    vr::HmdMatrix34_t matrix = hmdPose.mDeviceToAbsoluteTracking;

    // extract position
    mCurrentPosition.setX(matrix.m[0][3]);
    mCurrentPosition.setY(matrix.m[1][3]);
    mCurrentPosition.setZ(matrix.m[2][3]);

    // extract rotation matrix
    mCurrentOrientation = WbMatrix3(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[1][0], matrix.m[1][1],
                                    matrix.m[1][2], matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);

    setOrientation(mViewpointOrientation);
    setPosition(mViewpointPosition);
  }

  if (mWrenViewports[0] && mSystem) {
    WbWrenOpenGlContext::makeWrenCurrent();
    wr_scene_render_to_viewports(wr_scene_get_instance(), 2, mWrenViewports, NULL, true);
    WbWrenOpenGlContext::doneWren();
    vr::VRCompositor()->Submit(vr::Eye_Left, mTextureReferences[LEFT], mTextureBounds);
    vr::VRCompositor()->Submit(vr::Eye_Right, mTextureReferences[RIGHT], mTextureBounds);
    setTextureOverlayVisible(false);
  }
  updateTimer();

  if (log)
    log->stopMeasure(WbPerformanceLog::VIRTUAL_REALITY_HEADSET_RENDERING);
}

void WbVirtualRealityHeadset::updateTimer() {
  WbSimulationState *simulationState = WbSimulationState::instance();
  if (simulationState->isPaused() || simulationState->isStep())
    mTimer->start(1000.0 / WbWorld::instance()->worldInfo()->fps());
  else
    mTimer->stop();
}

void WbVirtualRealityHeadset::enablePositionTracking(bool enable) {
  mTrackPosition = enable;
  if (!enable)
    mCurrentPosition = WbVector3();
  setPosition(mViewpointPosition);
}

void WbVirtualRealityHeadset::enableOrientationTracking(bool enable) {
  mTrackOrientation = enable;
  if (!enable)
    mCurrentOrientation = WbMatrix3();
  setOrientation(mViewpointOrientation);
}

void WbVirtualRealityHeadset::setEyeView(EYE eye) {
  mDisplayedEye = eye;
}

void WbVirtualRealityHeadset::applyFieldOfViewToWren() {
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (mWrenCameras[i])
      wr_camera_set_fovy(mWrenCameras[i], mFieldOfViewY[i]);
  }
}

void WbVirtualRealityHeadset::setOrientation(const WbRotation &orientation) {
  mViewpointOrientation = orientation;
  WbMatrix3 rotation = orientation.toMatrix3();
  if (mTrackOrientation)
    rotation *= mCurrentOrientation;
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (mWrenCameras[i]) {
      WbRotation eyeRotation = WbRotation(rotation * mEyeRotation[i]);
      eyeRotation.normalize();
      float angleAxis[] = {static_cast<float>(eyeRotation.angle()), static_cast<float>(eyeRotation.x()),
                           static_cast<float>(eyeRotation.y()), static_cast<float>(eyeRotation.z())};
      wr_camera_set_orientation(mWrenCameras[i], angleAxis);
    }
  }
}

void WbVirtualRealityHeadset::setPosition(const WbVector3 &position) {
  mViewpointPosition = position;
  if (mTrackPosition)
    mViewpointPosition += mViewpointOrientation.toMatrix3() * mCurrentPosition;
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (mWrenCameras[i]) {
      WbVector3 offset = mEyeTranslation[i];
      if (mTrackOrientation)
        offset = (mViewpointOrientation.toMatrix3() * mCurrentOrientation) * offset;
      else
        offset = mViewpointOrientation.toMatrix3() * offset;
      float pos[] = {static_cast<float>(mViewpointPosition.x() + offset.x()),
                     static_cast<float>(mViewpointPosition.y() + offset.y()),
                     static_cast<float>(mViewpointPosition.z() + offset.z())};
      wr_camera_set_position(mWrenCameras[i], pos);
    }
  }
  mViewpointPosition = position;
}

void WbVirtualRealityHeadset::setNear(double nearValue) {
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (mWrenCameras[i])
      wr_camera_set_near(mWrenCameras[i], nearValue);
  }
}

void WbVirtualRealityHeadset::setFar(double farValue) {
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (mWrenCameras[i])
      wr_camera_set_far(mWrenCameras[i], farValue);
  }
}

void WbVirtualRealityHeadset::setupLensFlare(WbLensFlare *lensFlare) {
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (mWrenViewports[i])
      lensFlare->setup(mWrenViewports[i]);
  }
}

void WbVirtualRealityHeadset::applyAspectRatioToWren() {
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (mWrenCameras[i])
      wr_camera_set_aspect_ratio(mWrenCameras[i], mAspectRatio[i]);
  }
}

void WbVirtualRealityHeadset::applyRenderingModeToWren() {
  const bool wireframeIsSelected = WbWrenRenderingContext::instance()->renderingMode() == WbWrenRenderingContext::RM_WIREFRAME;
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (wireframeIsSelected)
      wr_viewport_set_polygon_mode(mWrenViewports[i], WR_VIEWPORT_POLYGON_MODE_LINE);
    else
      wr_viewport_set_polygon_mode(mWrenViewports[i], WR_VIEWPORT_POLYGON_MODE_FILL);
    wr_viewport_set_visibility_mask(mWrenViewports[i], WbWrenRenderingContext::instance()->visibilityMask());
  }
}

void WbVirtualRealityHeadset::applyProjectionModeToWren() {
  for (int i = 0; i < EYE_NUMBER; ++i) {
    if (mWrenCameras[i])
      wr_camera_set_projection_mode(mWrenCameras[i], WR_CAMERA_PROJECTION_MODE_PERSPECTIVE);
  }
}
