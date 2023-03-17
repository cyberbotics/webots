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

// Implemented node class representing an hinge2 (2 DOF, rotation along two choosen intersecting axes)

#ifndef WB_HINGE_2_JOINT_HPP
#define WB_HINGE_2_JOINT_HPP

#include "WbHingeJoint.hpp"

#include <QtCore/QMap>

class QString;

class WbHinge2Joint : public WbHingeJoint {
  Q_OBJECT

public:
  explicit WbHinge2Joint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbHinge2Joint(WbTokenizer *tokenizer = NULL);
  WbHinge2Joint(const WbHinge2Joint &other);
  explicit WbHinge2Joint(const WbNode &other);
  virtual ~WbHinge2Joint();

  void preFinalize() override;
  void postFinalize() override;
  int nodeType() const override { return WB_NODE_HINGE_2_JOINT; }
  void createWrenObjects() override;
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void reset(const QString &id) override;
  void resetPhysics() override;
  void save(const QString &id) override;
  QVector<WbLogicalDevice *> devices() const override;
  bool resetJointPositions() override;
  void setPosition(double position, int index = 1) override;
  double position(int index = 1) const override;
  double initialPosition(int index = 1) const override;
  WbJointParameters *parameters2() const override;
  void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const override;

  WbMotor *motor2() const override;
  WbPositionSensor *positionSensor2() const;
  WbBrake *brake2() const;
  WbJointDevice *device2(int index) const;
  virtual int devices2Number() const;
  void updateEndPointZeroTranslationAndRotation() override;

public slots:
  bool setJoint() override;

protected:
  virtual WbVector3 axis2() const;  // return the axis of the joint with coordinates relative to the parent Solid; defaults to
                                    // the rotation axis of the solid endpoint
  WbQuaternion endPointRotation() const;
  WbRotationalMotor *rotationalMotor2() const;
  WbMFNode *mDevice2;  // JointDevices: logical position sensor device, a motor and brake, only one per type is allowed
  double mOdePositionOffset2;
  double mPosition2;                       // Keeps track of the joint position2 if JointParameters2 don't exist.
  bool mSpringAndDampingConstantsAxis1On;  // defines if there is spring and dampingConstant along this axis
  bool mSpringAndDampingConstantsAxis2On;
  QMap<QString, double> mSavedPositions2;
  void updatePosition(double position) override;
  void updatePositions(double position, double position2);
  void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) override;
  void updateOdePositionOffset() override;
  void writeExport(WbWriter &writer) const override;

protected slots:
  virtual void addDevice2(int index);
  void updateParameters() override;
  void updatePosition() override;
  void updateMinAndMaxStop(double min, double max) override;
  void updateJointAxisRepresentation() override;

private:
  WbHinge2Joint &operator=(const WbHinge2Joint &);  // non copyable
  void updateParameters2();
  void init();
  WbSFNode *mParameters2;
  void applyToOdeAxis() override;
  void applyToOdeMinAndMaxStop() override;
};

#endif
