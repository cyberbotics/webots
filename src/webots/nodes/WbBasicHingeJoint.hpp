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

// Implemented node class representing a hinge (1 DOF, rotation along a choosen axis)

#ifndef WB_BASIC_HINGE_JOINT_HPP
#define WB_BASIC_HINGE_JOINT_HPP

#include "WbJoint.hpp"

class WbRotationalMotor;
class WbHingeJointParameters;

class WbBasicHingeJoint : public WbJoint {
  Q_OBJECT

public:
  explicit WbBasicHingeJoint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbBasicHingeJoint(WbTokenizer *tokenizer = NULL);
  WbBasicHingeJoint(const WbBasicHingeJoint &other);
  explicit WbBasicHingeJoint(const WbNode &other);
  virtual ~WbBasicHingeJoint();

  int nodeType() const override { return WB_NODE_BASIC_HINGE_JOINT; }
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void updateOdeWorldCoordinates() override;
  void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const override;

  WbVector3 anchor() const override;
  // return the axis of the joint with coordinates relative to the parent Solid; defaults to unit x-axis
  WbVector3 axis() const override;

public slots:
  bool setJoint() override;

protected:
  void setOdeJoint(dBodyID body, dBodyID parentBody) override;
  WbRotationalMotor *rotationalMotor() const;
  void updatePosition(double position) override;  // position change caused by the jerk of a statically based robot
  WbHingeJointParameters *hingeJointParameters() const;
  void updateEndPointZeroTranslationAndRotation() override;
  void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) override;

protected slots:
  void updatePosition() override;
  void updateParameters() override;
  void updateMinAndMaxStop(double min, double max) override;
  virtual void updateAnchor();

private slots:
  void updateSuspension();

private:
  void applyToOdeMinAndMaxStop() override;
  virtual void applyToOdeSuspension();
  void applyToOdeAxis() override;
  virtual void applyToOdeSuspensionAxis();
  void applyToOdeAnchor();
};

#endif
