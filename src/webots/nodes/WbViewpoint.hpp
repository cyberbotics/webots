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

#ifndef WB_VIEWPOINT_HPP
#define WB_VIEWPOINT_HPP

#include "WbBackground.hpp"
#include "WbBaseNode.hpp"
#include "WbMatrix3.hpp"
#include "WbQuaternion.hpp"
#include "WbRotation.hpp"
#include "WbSFBool.hpp"
#include "WbSFString.hpp"

struct WrCamera;
struct WrTexture;
struct WrViewport;

class WbAbstractPose;
class WbCoordinateSystem;
class WbLensFlare;
class WbRay;
class WbSolid;
class WbVector2;
class WbVirtualRealityHeadset;
class WbWrenBloom;
class WbWrenGtao;
class WbWrenSmaa;
class WbWrenHdr;

class QVariantAnimation;

class WbViewpoint : public WbBaseNode {
  Q_OBJECT

public:
  enum { FOLLOW_NONE, FOLLOW_TRACKING, FOLLOW_MOUNTED, FOLLOW_PAN_AND_TILT };

  // constructors and destructor
  explicit WbViewpoint(WbTokenizer *tokenizer = NULL);
  WbViewpoint(const WbViewpoint &other);
  explicit WbViewpoint(const WbNode &other);
  virtual ~WbViewpoint();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_VIEWPOINT; }
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;

  static QString followTypeToString(int type);
  static int followStringToType(const QString &type);

  // getters
  WrCamera *cameraWren() const { return mWrenCamera; }
  WrViewport *viewportWren() const { return mWrenViewport; }

  WbSFVector3 *position() const { return mPosition; }
  WbSFRotation *orientation() const { return mOrientation; }
  WbSFString *follow() const { return mFollow; }
  double aspectRatio() const { return mAspectRatio; }
  double orthographicViewHeight() const { return mOrthographicViewHeight; }
  bool isLocked() const { return mIsLocked; }
  bool rotationCenterIsLocked() const { return mRotationCenterIsLocked; }
  const WbVector3 &rotationCenter() { return mRotationCenter; }
  WbSolid *followedSolid() const { return mFollowedSolid; }
  int followType() const { return followStringToType(mFollowType->value()); }
  bool isFollowed(const WbSolid *solid) const;
  WbSFDouble *followSmoothness() const { return mFollowSmoothness; }
  WbSFDouble *fieldOfView() const { return mFieldOfView; }
  int projectionMode() const { return mProjectionMode; }
  float viewDistanceUnscaling(const WbVector3 &position) const;
  WbSFDouble *exposure() const { return mExposure; }

  // setters
  void decOrthographicViewHeight();
  void incOrthographicViewHeight();
  void setOrthographicViewHeight(double ovh);
  void lock() { mIsLocked = true; }
  void unlock() { mIsLocked = false; }
  void lockRotationCenter() { mRotationCenterIsLocked = true; }
  void unlockRotationCenter() { mRotationCenterIsLocked = false; }
  void startFollowUp(WbSolid *solid, bool updateField);
  void startFollowUpFromField();
  void setFollowType(int followType);
  void storePickedCoordinates(const WbVector3 &v) { mRotationCenter = v; }
  void setCoordinateSystemVisibility(bool visible);
  void restore();
  void save(const QString &id) override;
  void setPosition(const WbVector3 &position);
  void setProjectionMode(int projectionMode) {
    if (projectionMode != mProjectionMode) {
      mProjectionMode = projectionMode;
      emit cameraModeChanged();
    }
  }
  void lookAt(const WbVector3 &target, const WbVector3 &upVector);

  // fixed views
  void southView();
  void northView();
  void westView();
  void eastView();
  void topView();
  void bottomView();
  void objectFrontView();
  void objectBackView();
  void objectLeftView();
  void objectRightView();
  void objectTopView();
  void objectBottomView();

  // public cleanup
  void terminateFollowUp();

  // public updates
  void updateAspectRatio(double renderWindowAspectRatio);
  void updateFollowUp();
  void updateFollowSolidState();
  void updateOrthographicViewHeight();

  void setNodesVisibility(QList<const WbBaseNode *> nodes, bool visible);
  QList<const WbBaseNode *> getInvisibleNodes() const { return mInvisibleNodes; }
  void enableNodeVisibility(bool enabled);

  // Ray picking based on current projection mode
  void viewpointRay(int x, int y, WbRay &ray) const;
  WbVector3 pick(int x, int y, double z0) const;
  void toPixels(const WbVector3 &pos, WbVector2 &P) const;
  void toPixels(const WbVector3 &pos1, WbVector2 &P1, const WbVector3 &pos2, WbVector2 &P2) const;
  void toWorld(const WbVector3 &pos, WbVector3 &P) const;
  void eyeToPixels(const WbVector3 &eyePosition, WbVector2 &P) const;
  double zEye(const WbVector3 &pos) const;

  bool moveViewpointToObject(WbBaseNode *node);  // return true if node was valid

  // Virtual reality headset
  bool enableVirtualRealityHeadset(bool enable);
  void setVirtualRealityHeadsetAntiAliasing(bool enable);

  WbLensFlare *lensFlare() const;
  void updatePostProcessingEffects();
  void updatePostProcessingParameters();

public slots:
  void updateOptionalRendering(int optionalRendering);

protected:
  void exportNodeFields(WbWriter &writer) const override;

private:
  // user accessible fields
  WbSFDouble *mFieldOfView;
  WbSFRotation *mOrientation;
  WbSFVector3 *mPosition;
  WbSFString *mDescription;
  WbSFDouble *mNear;
  WbSFDouble *mFar;
  WbSFDouble *mExposure;
  WbSFString *mFollow;
  WbSFString *mFollowType;
  WbSFDouble *mFollowSmoothness;
  WbSFNode *mLensFlare;
  WbSFDouble *mAmbientOcclusionRadius;
  WbSFDouble *mBloomThreshold;

  // post-processing effects
  WbWrenSmaa *mWrenSmaa;
  WbWrenHdr *mWrenHdr;
  WbWrenGtao *mWrenGtao;
  WbWrenBloom *mWrenBloom;
  const float *mInverseViewMatrix;

  // to restore viewpoint
  QMap<QString, double> mSavedFieldOfView;
  QMap<QString, WbVector3> mSavedPosition;
  QMap<QString, WbRotation> mSavedOrientation;
  QMap<QString, QString> mSavedDescription;
  QMap<QString, double> mSavedNear;
  QMap<QString, double> mSavedFar;
  QMap<QString, double> mSavedOrthographicHeight;
  QMap<QString, QString> mSavedFollow;

  // follow solid stuff
  WbSolid *mFollowedSolid;
  WbVector3 mFollowedSolidPreviousPosition;
  WbMatrix3 mFollowedSolidReferenceRotation;
  WbRotation mViewPointReferenceRotation;
  WbVector3 mReferenceOffset;
  WbVector3 mEquilibriumVector;
  WbVector3 mVelocity;
  bool mNeedToUpdateFollowSolidState;

  // other variables
  int mProjectionMode;
  WrCamera *mWrenCamera;
  WrViewport *mWrenViewport;

  QList<const WbBaseNode *> mInvisibleNodes;
  bool mNodeVisibilityEnabled;

  WbCoordinateSystem *mCoordinateSystem;
  double mAspectRatio;
  double mFieldOfViewY;
  double mTanHalfFieldOfViewY;
  double mOrthographicViewHeight;
  WbVirtualRealityHeadset *mVirtualRealityHeadset;
  bool mFromOrthographic;

  bool mIsLocked;
  WbVector3 mRotationCenter;
  bool mRotationCenterIsLocked;
  bool mFollowChangedBySelection;
  bool mFollowChangedBySolidName;
  bool mFollowEmptiedByDestroyedSolid;

  // viewpoint translation animation
  WbVector3 mMoveToDirection;
  WbVector3 mInitialMoveToPosition;

  // viewpoint look-at animation
  WbQuaternion mLookAtInitialQuaternion;
  WbQuaternion mLookAtFinalQuaternion;

  // viewpoint orbit animation
  WbVector3 mCenterToViewpointUnitVector;
  WbVector3 mOrbitTargetUnitVector;
  WbVector3 *mFinalOrbitTargetPostion;
  WbQuaternion mInitialOrientationQuaternion;
  WbQuaternion mFinalOrientationQuaternion;
  WbQuaternion mInitialOrbitQuaternion;
  WbQuaternion mFinalOrbitQuaternion;
  double mOrbitRadius;

  // used to ensure view animations correspond to the world coordinate system
  WbQuaternion mSpaceQuaternion;

  // Qt animations
  QVariantAnimation *mTranslateAnimation;
  QVariantAnimation *mRotateAnimation;
  QVariantAnimation *mOrbitAnimation;

  // static non-const variables
  static int cViewpointCameraNumber;
  static int cCameraCounter;
  static int cZorder;

  // static const variables
  static const double INCREASE_FACTOR;
  static const double DECREASE_FACTOR;
  static const double X_OFFSET;
  static const double X_REL_ORTHOGRAPHIC;
  static const double Z_THRESHOLD;

  WbViewpoint &operator=(const WbViewpoint &);  // non copyable
  WbNode *clone() const override { return new WbViewpoint(*this); }

  // private initialization methods
  void init();
  void createCoordinateSystem();

  // private cleanup methods
  void clearCoordinateSystem();
  void deleteWrenObjects();

  // private apply methods
  void applyFieldOfViewToWren();
  void applyOrientationToWren();
  void applyPositionToWren();
  void applyNearToWren();
  void applyFarToWren();
  void applyRenderingModeToWren();
  void applyOptionalRenderingToWren(int optionalRendering);  // called whenever the optional rendering selection changes
  void applyOptionalRenderingToWren();                       // called once in createWrenObjects() to initialize options
  void applyOrthographicViewHeightToWren();
  // non-slot private updates
  void updateFieldOfViewY();
  void showCoordinateSystem(bool visible);

  void recomputeFollowField();
  void createCameraListenerIfNeeded();

  // can be used for any generic animated viewpoint movement
  void moveTo(const WbVector3 &targetPosition, const WbRotation &targetRotation);
  void orbitTo(const WbVector3 &targetUnitVector, const WbRotation &targetRotation,
               const WbAbstractPose *selectedObject = NULL);

  static WbAbstractPose *computeSelectedObjectPose();
  static WbRotation computeObjectViewRotation(const WbRotation &rotation, const WbAbstractPose *selectedObject);

private slots:
  void updateFieldOfView();
  void updateOrientation();
  void updatePosition();
  void updateNear();
  void updateFar();
  void updateExposure();
  void updateFollow();
  void updateRenderingMode();
  void updateCoordinateSystem();
  void updateFollowType();
  void updateLensFlare();
  void updateAmbientOcclusionRadius();
  void updateBloomThreshold();
  // cleanup
  void emptyFollow();
  // synchronize
  void synchronizeFollowWithSolidName();

  // used for each step of animated viewpoint movement
  void translateAnimationStep(const QVariant &value);
  void rotateAnimationStep(const QVariant &value);
  void translateOrbitAnimationStep(const QVariant &value);
  void rotateOrbitAnimationStep(const QVariant &value);
  void lookAtAnimationStep(const QVariant &value);
  void animateLookAtIfNeeded();
  void firstOrbitStep();
  void secondOrbitStep();

  // called when an animation completes successfully, deletes animations and
  // resets default view radius
  void resetAnimations();

signals:
  void followInvalidated(bool valid);
  void followTypeChanged(int type);
  void cameraParametersChanged();
  void refreshRequired();
  void nodeVisibilityChanged(const WbNode *node, bool visibility);
  void virtualRealityHeadsetRequiresRender();
  void cameraModeChanged();
};

#endif
