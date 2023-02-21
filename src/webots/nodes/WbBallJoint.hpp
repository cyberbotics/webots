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

// Implemented node class representing a ball-and-socket joint (3 angular DOF)

#ifndef WB_BALL_JOINT_HPP
#define WB_BALL_JOINT_HPP

class WbBallJointParameters;
class WbVector3;

#include <cassert>
#include "WbHinge2Joint.hpp"

class WbBallJoint : public WbHinge2Joint {
  Q_OBJECT

public:
  virtual ~WbBallJoint();
  explicit WbBallJoint(WbTokenizer *tokenizer = NULL);
  WbBallJoint(const WbBallJoint &other);
  explicit WbBallJoint(const WbNode &other);
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  int nodeType() const override { return WB_NODE_BALL_JOINT; }
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void reset(const QString &id) override;
  void resetPhysics() override;
  void save(const QString &id) override;
  QVector<WbLogicalDevice *> devices() const override;
  dJointID jointID() const override { return mControlMotor; }
  bool resetJointPositions() override;
  void setPosition(double position, int index = 1) override;
  double position(int index = 1) const override;
  double initialPosition(int index = 1) const override;
  WbBallJointParameters *ballJointParameters() const;
  WbJointParameters *parameters3() const override;
  void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const override;
  WbMotor *motor3() const override;
  WbPositionSensor *positionSensor3() const;
  WbBrake *brake3() const;
  WbJointDevice *device3(int index) const;
  virtual int devices3Number() const;

  WbVector3 axis() const override;
  void updateEndPointZeroTranslationAndRotation() override;

public slots:
  bool setJoint() override;
  void updatePosition() override;

protected:
  WbVector3 axis2() const override;
  WbVector3 axis3() const;
  WbMFNode *mDevice3;  // JointDevices: logical position sensor device, a motor and brake, only one per type is allowed
  double mOdePositionOffset3;
  double mPosition3;  // Keeps track of the joint position3 if JointParameters3 don't exist.

  WbVector3 anchor() const override;  // defaults to the center of the Solid parent, i.e. (0, 0, 0) in relative coordinates
  void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) override;
  void updateOdePositionOffset() override;
  void updatePosition(double position) override;
  void updatePositions(double position, double position2, double position3);
  void writeExport(WbWriter &writer) const override;

protected slots:
  void addDevice2(int index) override;
  virtual void addDevice3(int index);
  void updateParameters() override;
  void updateJointAxisRepresentation() override;
  void checkMotorLimit();

private:
  WbBallJoint &operator=(const WbBallJoint &);  // non copyable
  WbRotationalMotor *rotationalMotor3() const;
  void updateParameters3();
  WbSFNode *mParameters3;
  QMap<QString, double> mSavedPositions3;
  dJointID mControlMotor;  // ODE angular motor used to control the ball joint
  void applyToOdeAxis() override;
  void applyToOdeMinAndMaxStop() override;
  void init();
};

#endif
