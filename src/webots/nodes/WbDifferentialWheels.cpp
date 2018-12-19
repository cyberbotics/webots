// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbDifferentialWheels.hpp"
#include "WbKinematicDifferentialWheels.hpp"
#include "WbLog.hpp"
#include "WbMFDouble.hpp"
#include "WbMFNode.hpp"
#include "WbRandom.hpp"
#include "WbSensor.hpp"
#include "WbTokenizer.hpp"
#include "WbVersion.hpp"

#include "../../lib/Controller/api/messages.h"

#include <ode/ode.h>
#include <QtCore/QDataStream>
#include <cassert>

void WbDifferentialWheels::init() {
  warn(tr("The DifferentialWheels node is deprecated. Please use the Robot node instead with two "
          "HingeJoint, RotationalMotor and PositionSensor nodes. You can open a support ticket from "
          "the Help menu to get assistance on converting your DifferentialWheels robot."));

  for (int i = 0; i < 2; i++) {
    mPosition[i] = 0.0;
    mPreviousPosition[i] = 0.0;
    mEncoderLastPosition[i] = 0.0;
    mTargetSpeed[i] = 0.0;
    mActualSpeed[i] = 0.0;
  }

  mLastKinematicMotion = WbVector3(0, 0, 0);
  mEncoderSensor = NULL;
  mNeedToConfigure = false;
  mRightWheel = NULL;
  mLeftWheel = NULL;

  mAxleLength = findSFDouble("axleLength");
  mWheelRadius = findSFDouble("wheelRadius");
  mSlipNoise = findSFDouble("slipNoise");
  mEncoderNoise = findSFDouble("encoderNoise");
  mMotorConsumption = findSFDouble("motorConsumption");
  mMaxSpeed = findSFDouble("maxSpeed");
  mMaxAcceleration = findSFDouble("maxAcceleration");
  mSpeedUnit = findSFDouble("speedUnit");
  mEncoderResolution = findSFDouble("encoderResolution");
  mMaxForce = findSFDouble("maxForce");
}

WbDifferentialWheels::WbDifferentialWheels(WbTokenizer *tokenizer) : WbRobot("DifferentialWheels", tokenizer) {
  init();

  // in Webots >=6, the default speedUnit is 1.0 (instead of 0.1 previously)
  if (tokenizer && mSpeedUnit->value() == 1 && tokenizer->fileVersion() < WbVersion(6))
    mSpeedUnit->setValue(0.1);
}

WbDifferentialWheels::WbDifferentialWheels(const WbDifferentialWheels &other) : WbRobot(other) {
  init();
}

WbDifferentialWheels::WbDifferentialWheels(const WbNode &other) : WbRobot(other) {
  init();
}

WbDifferentialWheels::~WbDifferentialWheels() {
  delete mEncoderSensor;
}

void WbDifferentialWheels::preFinalize() {
  WbRobot::preFinalize();

  mEncoderSensor = new WbSensor();
  findWheels();
}

void WbDifferentialWheels::postFinalize() {
  WbRobot::postFinalize();

  connect(childrenField(), &WbMFNode::changed, this, &WbDifferentialWheels::findWheels);
  connect(mSpeedUnit, &WbSFDouble::changed, this, &WbDifferentialWheels::updateSpeedUnit);
  connect(mMaxSpeed, &WbSFDouble::changed, this, &WbDifferentialWheels::updateMaxSpeed);
}

void WbDifferentialWheels::updateSpeedUnit() {
  if (isControllerStarted())
    mNeedToConfigure = true;
}

void WbDifferentialWheels::updateMaxSpeed() {
  if (isControllerStarted())
    mNeedToConfigure = true;
}

void WbDifferentialWheels::handleMessage(QDataStream &stream) {
  const int before = stream.device()->pos();
  WbRobot::handleMessage(stream);
  if (stream.device()->pos() > before)
    return;

  unsigned char byte;
  stream >> (unsigned char &)byte;

  switch (byte) {
    case C_DIFFERENTIAL_WHEELS_SET_SPEED: {
      double left = 0.0, right = 0.0;
      stream >> (double &)left >> (double &)right;
      mTargetSpeed[0] = left * mSpeedUnit->value();
      mTargetSpeed[1] = right * mSpeedUnit->value();
      qBound(-mMaxSpeed->value(), mTargetSpeed[0], mMaxSpeed->value());
      qBound(-mMaxSpeed->value(), mTargetSpeed[1], mMaxSpeed->value());
      awake();
      return;
    }
    case C_DIFFERENTIAL_WHEELS_ENCODERS_SET_SAMPLING_PERIOD: {
      short rate;
      stream >> (short &)rate;
      mEncoderSensor->setRefreshRate(rate);
      return;
    }
    case C_DIFFERENTIAL_WHEELS_ENCODERS_SET_VALUE:
      stream >> (double &)mPosition[0] >> (double &)mPosition[1];
      return;
    default:
      assert(0);
  }
}

void WbDifferentialWheels::addConfigureToStream(QDataStream &stream) {
  stream << (short unsigned int)0;
  stream << (unsigned char)C_CONFIGURE;
  stream << (double)mMaxSpeed->value();
  stream << (double)mSpeedUnit->value();
  mNeedToConfigure = false;
}

void WbDifferentialWheels::writeAnswer(QDataStream &stream) {
  if (refreshEncoderSensorIfNeeded() || mEncoderSensor->hasPendingValue()) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_DIFFERENTIAL_WHEELS_GET_ENCODERS;
    stream << (double)mEncoderLastPosition[0] << (double)mEncoderLastPosition[1];

    mEncoderSensor->resetPendingValue();
  }

  if (mNeedToConfigure)
    addConfigureToStream(stream);

  WbRobot::writeAnswer(stream);
}

void WbDifferentialWheels::writeConfigure(QDataStream &stream) {
  mEncoderSensor->connectToRobotSignal(this);

  WbRobot::writeConfigure(stream);
  addConfigureToStream(stream);
}

void WbDifferentialWheels::updateSensors() {
  WbRobot::updateSensors();
  refreshEncoderSensorIfNeeded();
}

bool WbDifferentialWheels::refreshEncoderSensorIfNeeded() {
  if (isPowerOn() && mEncoderSensor->needToRefresh()) {
    mEncoderLastPosition[0] = mPosition[0];
    mEncoderLastPosition[1] = mPosition[1];
    mEncoderSensor->updateTimer();
  }
  return false;
}

void WbDifferentialWheels::powerOn(bool e) {
  WbRobot::powerOn(e);
  if (!e) {
    mTargetSpeed[0] = 0.0;
    mTargetSpeed[1] = 0.0;
    mActualSpeed[0] = 0.0;
    mActualSpeed[1] = 0.0;

    if (physics()) {
      if (mLeftWheel && mLeftWheel->joint())
        dJointSetHingeParam(mLeftWheel->joint(), dParamVel, 0);
      if (mRightWheel && mRightWheel->joint())
        dJointSetHingeParam(mRightWheel->joint(), dParamVel, 0);
    }
  }
}

void WbDifferentialWheels::findWheels() {
  mLeftWheel = NULL;
  mRightWheel = NULL;

  WbMFNode::Iterator it(children());
  while (it.hasNext()) {
    WbBaseNode *const node = dynamic_cast<WbBaseNode *>(it.next());
    if (node && node->nodeType() != WB_NODE_SOLID)
      continue;

    WbSolid *const solid = dynamic_cast<WbSolid *>(node);
    if (solid) {
      if (solid->name().compare("right wheel", Qt::CaseInsensitive) == 0)
        mRightWheel = solid;
      else if (solid->name().compare("left wheel", Qt::CaseInsensitive) == 0)
        mLeftWheel = solid;
    }
  }

  if (physics()) {
    if (!mRightWheel) {
      warn(tr("A Solid children named \"right wheel\" is missing."));
      return;
    }
    if (!mLeftWheel) {
      warn(tr("A Solid children named \"left wheel\" is missing."));
      return;
    }
    if (!mRightWheel->boundingObject()) {
      warn(tr("The Solid children named \"right wheel\" has no 'boundingObject'."));
      return;
    }
    if (!mLeftWheel->boundingObject()) {
      warn(tr("The Solid children named \"left wheel\" has no 'boundingObject'."));
      return;
    }
    if (!mRightWheel->physics()) {
      warn(tr("The Solid children named \"right wheel\" has no 'physics'."));
      return;
    }
    if (!mLeftWheel->physics()) {
      warn(tr("The Solid children named \"left wheel\" has no 'physics'."));
      return;
    }
  }
}

void WbDifferentialWheels::setWheelSpeed(WbSolid *wheel, int wheelIndex, double ms) {
  dJointID joint = wheel->joint();
  const double s = absoluteScale().x();
  double s4 = s * s;
  s4 *= s4;
  dJointSetHingeParam(joint, dParamFMax, s * s4 * mMaxForce->value());
  dJointSetHingeParam(joint, dParamVel, mTargetSpeed[wheelIndex]);

  const double noise = mEncoderNoise->value();
  if (noise >= 0.0) {  // if encoderNoise < 0, don't update encoders
    const double n = dJointGetHingeAngle(joint);
    double p = n - mPreviousPosition[wheelIndex];
    if (p > M_PI)
      p -= 2 * M_PI;
    else if (p < -M_PI)
      p += 2 * M_PI;

    mPreviousPosition[wheelIndex] = n;

    if (noise > 0.0)
      p *= 1.0 - noise + WbRandom::nextUniform() * noise * 2.0;

    mPosition[wheelIndex] += mEncoderResolution->value() * p;
  }
}

void WbDifferentialWheels::prePhysicsStep(double ms) {
  const bool power = battery().size() < 1 || (battery().size() > 0 && currentEnergy() > 0.0);

  if (!power) {
    powerOn(false);
    WbRobot::prePhysicsStep(ms);
    return;
  }

  if (!mKinematicDifferentialWheels) {
    if (mLeftWheel && mLeftWheel->joint())
      setWheelSpeed(mLeftWheel, 0, ms);
    if (mRightWheel && mRightWheel->joint())
      setWheelSpeed(mRightWheel, 1, ms);

    WbRobot::prePhysicsStep(ms);  // energy consumption

  } else {  // kinematics behavior
    for (int i = 0; i < 2; ++i) {
      if (mActualSpeed[i] == mTargetSpeed[i])
        continue;

      if (mActualSpeed[i] < mTargetSpeed[i]) {
        mActualSpeed[i] += mMaxAcceleration->value() * ms * 0.001;
        if (mActualSpeed[i] > mTargetSpeed[i])
          mActualSpeed[i] = mTargetSpeed[i];
      } else {
        mActualSpeed[i] -= mMaxAcceleration->value() * ms * 0.001;
        if (mActualSpeed[i] < mTargetSpeed[i])
          mActualSpeed[i] = mTargetSpeed[i];
      }
    }

    WbRobot::prePhysicsStep(ms);
    double noise = mSlipNoise->value();
    double noisy0 = mActualSpeed[0];
    double noisy1 = mActualSpeed[1];
    // qDebug() << "Actual Speed " << mActualSpeed[0] << " " << mActualSpeed[1];

    if (noise != 0.0) {
      noisy0 *= 1.0 - noise + WbRandom::nextUniform() * noise * 2.0;  // add noise to the commands
      noisy1 *= 1.0 - noise + WbRandom::nextUniform() * noise * 2.0;
    }

    // rotate the wheels
    const double t = ms * 0.001;
    if (mLeftWheel)
      mLeftWheel->setRotationAngle(mLeftWheel->rotation().angle() - noisy0 * t);
    if (mRightWheel)
      mRightWheel->setRotationAngle(mRightWheel->rotation().angle() - noisy1 * t);

    const double delta_direction =  // rotation angle
      (noisy1 - noisy0) * t * mWheelRadius->value() / (mAxleLength->value() * 2.0);
    const double a = rotation().angle() + delta_direction;               // [rad/s]
    const double v0 = -0.5 * (noisy0 + noisy1) * mWheelRadius->value();  // [m/s]

    const double v = v0 * t;
    if (delta_direction != 0.0) {
      mLastKinematicMotion = WbVector3(v * sin(a), 0.0, v * cos(a));
      setTranslation(translation() + mLastKinematicMotion);
      setRotationAngle(rotation().angle() + 2.0 * delta_direction);
      // translateAndRotate(v * sin(a), 0, v * cos(a), 2 * delta_direction);
    } else {
      mLastKinematicMotion = WbVector3(v * sin(a), 0.0, v * cos(a));
      setTranslation(translation() + mLastKinematicMotion);
    }

    // update incremental encoders
    noise = mEncoderNoise->value();
    if (noise >= 0.0) {  // if encoderNoise < 0, don't update encoders
      if (noise != 0.0) {
        noisy0 *= 1.0 - noise + WbRandom::nextUniform() * noise * 2.0;
        noisy1 *= 1.0 - noise + WbRandom::nextUniform() * noise * 2.0;
      }

      mPreviousPosition[0] = mPosition[0];
      mPreviousPosition[1] = mPosition[1];
      mPosition[0] += mEncoderResolution->value() * t * noisy0;
      mPosition[1] += mEncoderResolution->value() * t * noisy1;
    }

    updateOdeGeomPosition();
    printKinematicWarningIfNeeded();
  }

  if (battery().size() > 0 && currentEnergy() > 0.0) {
    const double e =
      currentEnergy() - 0.001 * ms * mMotorConsumption->value() * (fabs(mTargetSpeed[0]) + fabs(mTargetSpeed[1])) / 2.0;
    setCurrentEnergy(e > 0.0 ? e : 0.0);
  }
}

void WbDifferentialWheels::resetPhysics() {
  // reset wheels position / orentation
  if (mLeftWheel) {
    mLeftWheel->setTranslation(mLeftWheel->physicsResetTranslation());
    mLeftWheel->setRotation(mLeftWheel->physicsResetRotation());
    if (physics() && mLeftWheel->joint())
      mPreviousPosition[0] = dJointGetHingeAngle(mLeftWheel->joint());
  }
  if (mRightWheel) {
    mRightWheel->setTranslation(mRightWheel->physicsResetTranslation());
    mRightWheel->setRotation(mRightWheel->physicsResetRotation());
    if (physics() && mRightWheel->joint())
      mPreviousPosition[1] = dJointGetHingeAngle(mRightWheel->joint());
  }

  WbSolid::resetPhysics();
}
