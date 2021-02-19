// Copyright 1996-2021 Cyberbotics Ltd.
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

// Implemented node class representing an transmission joint

#ifndef WB_TRANSMISSION_JOINT_HPP
#define WB_TRANSMISSION_JOINT_HPP

#include "WbBaseNode.hpp"
#include "WbHingeJoint.hpp"

class WbBoundingSphere;

class WbTransmissionJoint : public WbHingeJoint {
  Q_OBJECT

public:
  explicit WbTransmissionJoint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbTransmissionJoint(WbTokenizer *tokenizer = NULL);
  WbTransmissionJoint(const WbTransmissionJoint &other);
  explicit WbTransmissionJoint(const WbNode &other);
  virtual ~WbTransmissionJoint();

  void preFinalize() override;
  void postFinalize() override;
  double position(int index = 1) const override;
  double initialPosition(int index = 1) const override;

  WbVector3 axis() const override;
  WbVector3 anchor() const override;
  WbVector3 axis2() const;
  WbVector3 anchor2() const;

  WbHingeJointParameters *hingeJointParameters2() const;
  void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const override;
  void computeStartPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const;

  const WbVector3 &zeroStartPointTranslation() const { return mStartPointZeroTranslation; }
  const WbRotation &zeroStartPointRotation() const { return mStartPointZeroRotation; }

  void setSolidStartPoint(WbSolid *solid);
  void setSolidStartPoint(WbSolidReference *solid);
  void setSolidStartPoint(WbSlot *slot);

  WbSolid *solidStartPoint() const;
  WbSolidReference *solidReferenceStartPoint() const;
public slots:
  void updateStartPoint();
  bool setJoint() override;

signals:
  void startPointChanged(WbBaseNode *node);

protected:
  WrTransform *mTransform2;
  WrRenderable *mRenderable2;
  WrStaticMesh *mMesh2;
  WrMaterial *mMaterial2;

  dJointID mJoint2;
  dJointID jointID() const { return mJoint2; }
  double mOdePositionOffset2;
  double mPosition2;                       // Keeps track of the joint position2 if JointParameters2 don't exist.
  bool mSpringAndDampingConstantsAxis1On;  // defines if there is spring and dampingConstant along this axis
  bool mSpringAndDampingConstantsAxis2On;
  double mInitialPosition2;
  bool mIsStartPointPositionChangedByJoint;

  void setupTransmission();

  void updateOdePositionOffset() override;
  void updateOdePositionOffset2();

  void updatePositionOf(int index, double position);
  // variables and methods about the startPoint Solid translation and rotation when joint position is 0
  WbVector3 mStartPointZeroTranslation;
  WbRotation mStartPointZeroRotation;

  void updateStartPointZeroTranslationAndRotation();
  void retrieveStartPointSolidTranslationAndRotation(WbVector3 &it, WbRotation &ir) const;

protected slots:
  void updateParameters() override;
  void updateParameters2();
  void updatePosition() override;
  void updatePosition2();
  void updateJointAxisRepresentation() override;

  void updateAxis() override;
  void updateAxis2();
  void updateAnchor() override;
  void updateAnchor2();

private:
  // enum { UNDEFINED, CLASSIC_GEAR, BEVEL_GEAR, CHAIN_DRIVE };
  int mTransmissionMode;
  dJointID mTransmission;

  void inferTransmissionMode();
  WbTransmissionJoint &operator=(const WbTransmissionJoint &);  // non copyable
  void init();
  WbSFNode *mParameters2;
  WbSFNode *mStartPoint;
  WbSFDouble *mBacklash;
  WbSFDouble *mMultiplier;
  void updateBacklash();
  void updateMultiplier();

  void applyToOdeAnchor();
  void applyToOdeAnchor2();
  void applyToOdeAxis() override;
  void applyToOdeAxis2();

  bool setJoint2();

private slots:
  void updateStartPointPosition();
  void updateBoundingSphere(WbBaseNode *subNode);
};

#endif
