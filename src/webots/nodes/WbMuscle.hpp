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

#ifndef WB_MUSCLE_HPP
#define WB_MUSCLE_HPP

//
// Description: Graphical representation of an artificial muscle.
//              This node is a child node of LinearMotor and RotationalMotor
//              nodes. The mesh has a spheroid shape with constant volume that
//              is computed based on the the maximum radius field and the
//              minimal distance between the parent transform and endPoint solid
//              center positions.
//

#include "WbBaseNode.hpp"
#include "WbMatrix4.hpp"

struct WrTransform;
struct WrRenderable;
struct WrMaterial;
struct WrDynamicMesh;
struct WrTexture2d;

class QImage;

class WbMuscle : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbMuscle(WbTokenizer *tokenizer = NULL);
  WbMuscle(const WbMuscle &other);
  explicit WbMuscle(const WbNode &other);
  virtual ~WbMuscle();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_MUSCLE; }
  void createWrenObjects() override;
  void postFinalize() override;
  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override {
    return QList<const WbBaseNode *>() << this;
  };

  void animateMesh();

private slots:
  void updateVolume();
  void computeStretchedDimensions();

private:
  WbMuscle &operator=(const WbMuscle &);  // non copyable
  WbNode *clone() const override { return new WbMuscle(*this); }
  void init();

  void updateMeshCoordinates();
  void createMeshBuffers();
  void updateVisibility() const;
  void updateMaterial();

  WbSFDouble *mVolume;
  WbSFVector3 *mStartOffset;
  WbSFVector3 *mEndOffset;
  WbMFColor *mColors;
  WbSFBool *mCastShadows;
  WbSFBool *mVisible;
  // deprecated
  WbSFDouble *mMaxRadius;

  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrMaterial *mMaterial;
  WrTexture2d *mTexture;
  WrDynamicMesh *mMesh;
  QImage *mQImage;

  const WbPose *mParentPose;
  const WbSolid *mEndPoint;
  WbMatrix4 mMatrix;
  double mHeight;
  double mPreviousHeight;
  double mRadius;
  bool mDirectionInverted;

  double mStatus;  // idle = 0, contracting < 0, relaxing > 0
  double mMaterialStatus;

private slots:
  void updateEndPoint(WbBaseNode *node);
  void updateEndPointPosition();
  void updateCastShadows();
  void updateVisible();
  void updateStretchForce(double forcePercentage, bool immediateUpdate, int motorIndex);
  void stretch();
};

#endif
