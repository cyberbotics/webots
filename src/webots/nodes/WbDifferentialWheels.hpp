// Copyright 1996-2019 Cyberbotics Ltd.
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

#ifndef WB_DIFFERENTIAL_WHEELS_HPP
#define WB_DIFFERENTIAL_WHEELS_HPP

#include "WbRobot.hpp"
#include "WbSFDouble.hpp"

class QDataStream;
class WbSensor;

class WbDifferentialWheels : public WbRobot {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbDifferentialWheels(WbTokenizer *tokenizer = NULL);
  WbDifferentialWheels(const WbDifferentialWheels &other);
  explicit WbDifferentialWheels(const WbNode &other);
  virtual ~WbDifferentialWheels();

  // Reimplemented methods
  int nodeType() const override { return WB_NODE_DIFFERENTIAL_WHEELS; }
  void preFinalize() override;
  void postFinalize() override;
  void prePhysicsStep(double ms) override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(QDataStream &) override;
  void writeConfigure(QDataStream &) override;
  bool updateJointChildren() override { return false; }
  void resetPhysics() override;
  void updateSensors() override;

  void powerOn(bool) override;  // when running out of battery, turn the motors and sensors off
  double targetSpeed(int i) const { return mTargetSpeed[i]; }
  double encoder(int i) const { return mPosition[i]; }
  void setEncoders(double l, double r) {
    mPosition[0] = l;
    mPosition[1] = r;
  }
  double encoderNoise() const { return mEncoderNoise->value(); }
  double speedUnit() const { return mSpeedUnit->value(); }
  WbSolid *rightWheel() const { return mRightWheel; }
  WbSolid *leftWheel() const { return mLeftWheel; }
  double axleLength() const { return mAxleLength->value(); }
  double wheelRadius() const { return mWheelRadius->value(); }

private:
  WbSFDouble *mMotorConsumption;
  WbSFDouble *mAxleLength;
  WbSFDouble *mWheelRadius;
  WbSFDouble *mMaxSpeed;
  WbSFDouble *mMaxAcceleration;
  WbSFDouble *mSpeedUnit;
  WbSFDouble *mSlipNoise;
  WbSFDouble *mEncoderNoise;
  WbSFDouble *mEncoderResolution;
  WbSFDouble *mMaxForce;

  // wheels
  WbSolid *mRightWheel;  // useful to be able to
  WbSolid *mLeftWheel;   // rotate the wheels

  // current stuff
  double mPosition[2];  // encoder position
  double mTargetSpeed[2];
  double mActualSpeed[2];
  WbSensor *mEncoderSensor;
  double mEncoderLastPosition[2];
  bool refreshEncoderSensorIfNeeded();

  // restoring stuff (kinematic collision)
  double mPreviousPosition[2];

  // last kinematic motion
  WbVector3 mLastKinematicMotion;

  bool mNeedToConfigure;

  WbDifferentialWheels &operator=(const WbDifferentialWheels &);  // non copyable
  WbNode *clone() const override { return new WbDifferentialWheels(*this); }
  void init();
  void setWheelSpeed(WbSolid *wheel, int wheelIndex, double ms);
  void addConfigureToStream(QDataStream &stream);

private slots:
  void findWheels();
  void updateSpeedUnit();
  void updateMaxSpeed();
};

#endif  // WB_DIFFERENTIAL_WHEELS_HPP
