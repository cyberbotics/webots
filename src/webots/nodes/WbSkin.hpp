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

#ifndef WB_SKIN_HPP
#define WB_SKIN_HPP

#include "WbAbstractPose.hpp"
#include "WbBaseNode.hpp"
#include "WbDevice.hpp"
#include "WbSFString.hpp"

class WbBoundingSphere;
class WbDownloader;
class WbMFNode;

struct WrDynamicMesh;
struct WrMaterial;
struct WrRenderable;
struct WrSkeleton;
struct WrStaticMesh;

class WbSkin : public WbBaseNode, public WbAbstractPose, public WbDevice {
  Q_OBJECT

public:
  explicit WbSkin(WbTokenizer *tokenizer = NULL);
  WbSkin(const WbSkin &other);
  explicit WbSkin(const WbNode &other);
  virtual ~WbSkin();

  WbMFNode *appearanceField() const { return mAppearanceField; }

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SKIN; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(WbDataStream &stream) override;
  void writeConfigure(WbDataStream &) override;
  void createWrenObjects() override;
  const QString &deviceName() const override { return mName->value(); }
  int deviceNodeType() const override { return nodeType(); }
  void reset(const QString &id) override;
  void updateSegmentationColor(const WbRgb &color) override { setSegmentationColor(color); }

  void setScaleNeedUpdate() override;
  void setMatrixNeedUpdate() override { WbAbstractPose::setMatrixNeedUpdateFlag(); }

  void emitTranslationOrRotationChangedByUser() override {}

  // ray tracing
  WbBoundingSphere *boundingSphere() const override { return mBoundingSphere; }
  void recomputeBoundingSphere() const;

signals:
  void wrenMaterialChanged();

private:
  WbSkin &operator=(const WbSkin &);  // non copyable
  WbNode *clone() const override { return new WbSkin(*this); }
  void init();

  WbSFString *mName;
  WbSFString *mModelUrl;
  WbMFNode *mAppearanceField;
  WbMFNode *mBonesField;
  WbSFBool *mCastShadows;

  WbDownloader *mDownloader;
  bool mIsModelUrlValid;
  WrSkeleton *mSkeleton;
  WrTransform *mSkeletonTransform;
  WrTransform *mRenderablesTransform;
  QList<WbRotation> mInitialSkeletonOrientation;
  QList<WbVector3> mInitialSkeletonPosition;

  QVector<WrRenderable *> mRenderables;
  QStringList mMaterialNames;
  QVector<WrMaterial *> mMaterials;
  QVector<WrMaterial *> mSegmentationMaterials;
  QVector<WrMaterial *> mEncodeDepthMaterials;
  QVector<WrDynamicMesh *> mMeshes;

  WrStaticMesh *mBoneMesh;
  WrMaterial *mBoneMaterial;
  QVector<WrTransform *> mBoneTransforms;
  QMap<WrTransform *, WrTransform *> mBonesMap;

  bool mNeedConfigureAfterModelChanged;
  WbVector3 *mBonePositionRequest;
  WbRotation *mBoneOrientationRequest;
  bool mBonesWarningPrinted;

  WbSFVector3 *mScale;
  double mPreviousXscaleValue;
  mutable WbVector3 mAbsoluteScale;
  mutable bool mAbsoluteScaleNeedUpdate;

  // WREN manipulators
  void sanitizeScale();

  // Ray tracing
  mutable WbBoundingSphere *mBoundingSphere;

  void createWrenSkeleton();
  void deleteWrenSkeleton();

  void setBoneOrientation(int boneIndex, double ax, double ay, double az, double angle, bool absolute);
  void setBonePosition(int boneIndex, double x, double y, double z, bool absolute);

  bool createSkeletonFromWebotsNodes();
  WrTransform *createBoneRepresentation(const float *scale, const float *orientation, bool visible);

  QString modelPath() const;
  void updateModel();
  void applyToScale();
  void applyScaleToWren();

  void setSegmentationColor(const WbRgb &color);

  void updateAbsoluteScale() const;
  const WbVector3 &absoluteScale() const;

private slots:
  virtual void updateTranslation();
  virtual void updateRotation();
  virtual void updateScale();
  void updateModelUrl();
  void updateAppearance();
  void updateMaterial();
  void updateAppearanceName(const QString &newName, const QString &prevName);
  void updateBones();
  void updateCastShadows();
  void updateOptionalRendering(int option);
  void downloadUpdate();
};

#endif
