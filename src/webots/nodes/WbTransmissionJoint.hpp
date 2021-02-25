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

struct dJointFeedback;

#include "WbJoint.hpp"

class WbRotationalMotor;
class WbHingeJointParameters;
class WbSolidReference;
class WbSlot;

class WbTransmissionJoint : public WbJoint {
  Q_OBJECT

public:
  explicit WbTransmissionJoint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbTransmissionJoint(WbTokenizer *tokenizer = NULL);
  WbTransmissionJoint(const WbTransmissionJoint &other);
  explicit WbTransmissionJoint(const WbNode &other);
  virtual ~WbTransmissionJoint();

  int nodeType() const override { return WB_NODE_TRANSMISSION_JOINT; }

  double position(int index = 1) const override;
  double initialPosition(int index = 1) const override;

  void preFinalize() override;
  void postFinalize() override;

  WbVector3 axis() const override;
  WbVector3 anchor() const override;
  WbVector3 axis2() const;
  WbVector3 anchor2() const;

  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;

  WbHingeJointParameters *hingeJointParameters() const;
  WbHingeJointParameters *hingeJointParameters2() const;

  WbSolidReference *solidReferenceStartPoint() const;
  WbSolid *solidStartPoint() const;
  void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const override;

  void printTransmissionConfig();
public slots:
  bool setJoint() override;
  bool setJoint2();

signals:

protected:
  bool mDummy;
  dJointID mJoint2;
  dBodyID body2;
  dGeomID geom2;

  dBodyID body1;
  dGeomID geom1;
  dJointFeedback *feedback;

  dJointID jointID() const { return mJoint2; }
  double mOdePositionOffset2;
  double mPosition2;  // Keeps track of the joint position2 if JointParameters2 don't exist.
  double mInitialPosition2;

  void dummyTransmission();  // dummyTransmission
  void setupTransmission();  // one-time setup

  void setupJoint2();

  void configureTransmission();  // configure parameters
  void updatePosition(double position) override;
  void updatePosition2(double position);

  void updateEndPointZeroTranslationAndRotation() override;
  void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) override;

  WbRotationalMotor *rotationalMotor() const;

  void setOdeJoint(dBodyID body, dBodyID parentBody) override;

protected slots:
  void updateParameters() override;
  void updateParameters2();
  void updatePosition() override;
  void updatePosition2();

  void updateAxis() override;
  void updateAxis2();
  void updateAnchor();
  void updateAnchor2();

  void updateJointAxisRepresentation() override;

  void updateOdePositionOffset() override;
  void updateOdePositionOffset2();

private:
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

  void applyToOdeMinAndMaxStop();

private slots:
  void updateStartPointPosition();
};

#endif
