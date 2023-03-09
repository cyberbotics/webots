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

// Abstract node class representing the most general mechanical joint supporting position sensor or motor

#ifndef WB_JOINT_HPP
#define WB_JOINT_HPP

#include "WbBasicJoint.hpp"

#include <QtCore/QMap>

class QString;

class WbBrake;
class WbJointDevice;
class WbJointParameters;
class WbLogicalDevice;
class WbMotor;
class WbPositionSensor;

class WbJoint : public WbBasicJoint {
  Q_OBJECT

public:
  virtual ~WbJoint();

  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void reset(const QString &id) override;
  virtual void resetPhysics();
  void save(const QString &id) override;
  virtual QVector<WbLogicalDevice *> devices() const;

  WbJointParameters *parameters() const;
  virtual double position(int index = 1) const { return (index == 1) ? mPosition : NAN; }
  virtual double initialPosition(int index = 1) const { return (index == 1) ? mSavedPositions[stateId()] : NAN; }
  virtual void setPosition(double position, int index = 1);
  bool resetJointPositions() override;
  virtual WbJointParameters *parameters2() const { return NULL; }
  virtual WbJointParameters *parameters3() const { return NULL; }

  WbPositionSensor *positionSensor() const;
  WbMotor *motor() const;
  virtual WbMotor *motor2() const { return NULL; }
  virtual WbMotor *motor3() const { return NULL; }
  WbBrake *brake() const;

  WbJointDevice *device(int index) const;
  virtual int devicesNumber() const;

signals:
  void updateMuscleStretch(double forcePercentage, bool immediateUpdate, int motorIndex);

public slots:
  virtual void updatePosition() {}

protected:
  void writeExport(WbWriter &writer) const override;

  WbJoint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  WbJoint(const WbJoint &other);
  WbJoint(const WbNode &other);

  void setOdeJoint(dBodyID body, dBodyID parentBody) override;
  virtual WbVector3 axis()
    const;  // return the axis of the joint with coordinates relative to the parent Solid; defaults to z-axis
  virtual void updatePosition(double position) = 0;  // position change caused by the jerk of a statically based robot

  WbMFNode *mDevice;  // JointDevices: logical position sensor device, a motor and brake, only one per type is allowed
  double mPosition;   // Keeps track of the joint position if JointParameters doesn't exist.
  QMap<QString, double> mSavedPositions;  // position loaded from a .wbt file
  double mTimeStep;  // keep track of the argument of the last call of 'prePhysicsStep' (wich is then used in 'postPhysicsStep'
                     // to update the mPosition

  // Joint position offset between ODE and Webots
  // ODE automatically resets the joint offset to 0 when setting the axis and anchor values
  // so we keep track of the current status by storing it in Webots
  double mOdePositionOffset;
  virtual void updateOdePositionOffset();

protected slots:
  virtual void addDevice(int index);
  void updateParameters() override;
  virtual void updateMinAndMaxStop(double min, double max);
  virtual void updateAxis();
  void updateJointAxisRepresentation() override;
  const QString urdfName() const override;

private:
  WbJoint &operator=(const WbJoint &);  // non copyable
  void init();
  virtual void applyToOdeMinAndMaxStop() = 0;
  virtual void applyToOdeAxis() = 0;
};
#endif
