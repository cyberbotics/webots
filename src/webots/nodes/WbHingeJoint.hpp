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

// Implemented node class representing an hinge (1 DOF, rotation along a choosen axis)

#ifndef WB_HINGE_JOINT_HPP
#define WB_HINGE_JOINT_HPP

#include "WbJoint.hpp"

class WbRotationalMotor;
class WbHingeJointParameters;

class WbHingeJoint : public WbJoint {
  Q_OBJECT

public:
  explicit WbHingeJoint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbHingeJoint(WbTokenizer *tokenizer = NULL);
  WbHingeJoint(const WbHingeJoint &other);
  explicit WbHingeJoint(const WbNode &other);
  virtual ~WbHingeJoint();

  int nodeType() const override { return WB_NODE_HINGE_JOINT; }
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void updateOdeWorldCoordinates() override;
  void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const override;

  void createWrenObjects() override;

  WbVector3 anchor() const override;
  // return the axis of the joint with coordinates relative to the parent Solid; defaults to unit x-axis
  WbVector3 axis() const override;
  void updateEndPointZeroTranslationAndRotation() override;

public slots:
  bool setJoint() override;
  void updatePosition() override;

protected:
  void setOdeJoint(dBodyID body, dBodyID parentBody) override;
  WbRotationalMotor *rotationalMotor() const;
  void updatePosition(double position) override;  // position change caused by the jerk of a statically based robot
  WbHingeJointParameters *hingeJointParameters() const;
  void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) override;

protected slots:
  void updateParameters() override;
  void updateMinAndMaxStop(double min, double max) override;
  void updateStopErp();
  void updateStopCfm();
  virtual void updateAnchor();
  void updateOptionalRendering(int option) override;

private slots:
  void updateSuspension();
  virtual void updateJointAnchorRepresentation();

private:
  WbHingeJoint &operator=(const WbHingeJoint &);  // non copyable
  void init();

  WrTransform *mAnchorTransform;
  WrRenderable *mAnchorRenderable;
  WrStaticMesh *mAnchorMesh;
  WrMaterial *mAnchorMaterial;

  void applyToOdeMinAndMaxStop() override;
  virtual void applyToOdeSuspension();
  void applyToOdeAxis() override;
  virtual void applyToOdeSuspensionAxis();
  void applyToOdeAnchor();
  void applyToOdeStopErp();
  void applyToOdeStopCfm();
};

#endif
