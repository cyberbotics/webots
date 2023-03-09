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

#ifndef WB_VIRTUAL_REALITY_HEADSET_HPP
#define WB_VIRTUAL_REALITY_HEADSET_HPP

#include "WbMatrix3.hpp"
#include "WbRotation.hpp"
#include "WbVector3.hpp"

#include <QtCore/QObject>

namespace vr {
  struct IVRSystem;
  struct VRTextureBounds_t;
  struct Texture_t;
  typedef uint64_t VROverlayHandle_t;
}  // namespace vr

struct WrCamera;
struct WrDrawableTexture;
struct WrViewport;
struct WrFrameBuffer;
struct WrTexture;
struct WrTransform;

class WbLensFlare;
class WbWrenSmaa;
class WbWrenHdr;
class QTimer;

class WbVirtualRealityHeadset : public QObject {
  Q_OBJECT

public:
  enum EYE { NONE = -1, LEFT, RIGHT, EYE_NUMBER };

  static void setLoadingTexture(WrTexture *texture);
  static bool isSteamVRInstalled();
  static bool isHeadsetConnected();
  static bool isInUse();

  static WbVirtualRealityHeadset *instance();
  static void cleanup();

  bool isValid() const;

  void setTextureOverlayVisible(bool visible);

  // public updates
  void applyAspectRatioToWren();
  void applyFieldOfViewToWren();
  void setPosition(const WbVector3 &position);
  void setOrientation(const WbRotation &orientation);
  void setNear(double nearValue);
  void setFar(double farValue);
  void setupLensFlare(WbLensFlare *lensFlare);
  void applyRenderingModeToWren();
  void applyProjectionModeToWren();

  WbWrenHdr *leftEyeHdr() const { return mLeftEyeHdr; }
  WbWrenHdr *rightEyeHdr() const { return mRightEyeHdr; }

  void deleteWrenObjects();
  void createWrenObjects(WrTransform *node, bool antiAliasing);

  WrTexture *visibleTexture() const;

  void enablePositionTracking(bool enable);
  void enableOrientationTracking(bool enable);
  const bool &isPositionTrackingEnabled() { return mTrackPosition; }
  const bool &isOrientationTrackingEnabled() { return mTrackOrientation; }
  void setEyeView(EYE eye);

  const WbVector3 &currentPosition() { return mCurrentPosition; }
  const WbMatrix3 &currentOrientation() { return mCurrentOrientation; }

public slots:
  void updateOrientationAndPosition();

private slots:
  void updateTimer();

private:
  explicit WbVirtualRealityHeadset();
  virtual ~WbVirtualRealityHeadset();
  void initializeTextureOverlay();

  bool mTrackPosition;
  bool mTrackOrientation;

  // used to send update to the headset even if view was not refreshed
  QTimer *mTimer;

  // eye displayed in the main view
  EYE mDisplayedEye;

  // properties of the headset
  double mFieldOfViewY[2];
  double mAspectRatio[2];
  WbVector3 mEyeTranslation[2];
  WbMatrix3 mEyeRotation[2];

  // viewpoint current position and rotationCenter
  WbVector3 mViewpointPosition;
  WbRotation mViewpointOrientation;

  // OpenVR stuff
  vr::IVRSystem *mSystem;
  vr::VRTextureBounds_t *mTextureBounds;
  vr::Texture_t *mTextureReferences[2];
  vr::VROverlayHandle_t *mOverlayHandle;

  WbVector3 mCurrentPosition;
  WbMatrix3 mCurrentOrientation;

  WrCamera *mWrenCameras[2];
  WrViewport *mWrenViewports[2];
  WrFrameBuffer *mWrenFrameBuffer[2];

  WbWrenSmaa *mLeftEyeSmaa;
  WbWrenSmaa *mRightEyeSmaa;
  WbWrenHdr *mLeftEyeHdr;
  WbWrenHdr *mRightEyeHdr;

  static WbVirtualRealityHeadset *cInstance;

signals:
  void renderRequired();
};

#endif
