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

//
//  WbMotor.cpp
//

#include "WbMotor.hpp"

#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbJoint.hpp"
#include "WbJointParameters.hpp"
#include "WbMuscle.hpp"
#include "WbPropeller.hpp"
#include "WbRobot.hpp"
#include "WbSensor.hpp"
#include "WbSolid.hpp"
#include "WbSoundEngine.hpp"
#include "WbTrack.hpp"
#include "WbUrl.hpp"

#include <ode/ode.h>

#include <QtCore/QDataStream>

#include <cassert>
#include <cmath>

#include "../../lib/Controller/api/messages.h"  // contains the definitions for the macros C_SET_SAMPLING_PERIOD, C_MOTOR_SET_POSITION, C_MOTOR_SET_VELOCITY ...

QList<const WbMotor *> WbMotor::cMotors;

void WbMotor::init() {
  mHasAllocatedJointFeedback = false;
  mForceOrTorqueSensor = NULL;
  mNeedToConfigure = false;
  mSoundClip = NULL;
  mForceOrTorqueLastValue = 0.0;
  mKinematicVelocitySign = 0;
  mRequestedDeviceTag = NULL;

  mMotorForceOrTorque = 0.0;
  mTargetVelocity = 0.0;
  mTargetPosition = 0.0;
  mCurrentVelocity = 0.0;
  mRawInput = 0.0;
  mUserControl = false;
  mErrorIntegral = 0.0;
  mPreviousError = 0.0;

  mMaxForceOrTorque = NULL;
  mAcceleration = findSFDouble("acceleration");
  mConsumptionFactor = findSFDouble("consumptionFactor");
  mControlPID = findSFVector3("controlPID");
  mMinPosition = findSFDouble("minPosition");
  mMaxPosition = findSFDouble("maxPosition");
  mMaxVelocity = findSFDouble("maxVelocity");
  mSound = findSFString("sound");
  mMuscles = findMFNode("muscles");
}

WbMotor::WbMotor(const QString &modelName, WbTokenizer *tokenizer) : WbJointDevice(modelName, tokenizer) {
  init();
}

WbMotor::WbMotor(const WbMotor &other) : WbJointDevice(other) {
  init();
}

WbMotor::WbMotor(const WbNode &other) : WbJointDevice(other) {
  init();
}

WbMotor::~WbMotor() {
  delete mForceOrTorqueSensor;
  cMotors.removeAll(this);
}

void WbMotor::preFinalize() {
  WbBaseNode::preFinalize();

  cMotors << this;

  updateMaxVelocity();
  updateMaxAcceleration();
  updateControlPID();
  updateMinAndMaxPosition();
  updateSound();

  mForceOrTorqueSensor = new WbSensor();

  mTargetVelocity = mMaxVelocity->value();
  const WbJoint *const j = joint();
  mTargetPosition = j ? position() : 0.0;
  mMotorForceOrTorque = mMaxForceOrTorque->value();
  updateMaxForceOrTorque();
}

void WbMotor::postFinalize() {
  WbJointDevice::postFinalize();
  assert(robot());
  if (!mMuscles->isEmpty() || robot()->maxEnergy() > 0)
    setupJointFeedback();

  WbMFIterator<WbMFNode, WbNode *> it(mMuscles);
  while (it.hasNext())
    dynamic_cast<WbMuscle *>(it.next())->postFinalize();

  connect(mMaxVelocity, &WbSFDouble::changed, this, &WbMotor::updateMaxVelocity);
  connect(mAcceleration, &WbSFDouble::changed, this, &WbMotor::updateMaxAcceleration);
  connect(mControlPID, &WbSFVector3::changed, this, &WbMotor::updateControlPID);
  connect(mMinPosition, &WbSFDouble::changed, this, &WbMotor::updateMinAndMaxPosition);
  connect(mMaxPosition, &WbSFDouble::changed, this, &WbMotor::updateMinAndMaxPosition);
  connect(mSound, &WbSFString::changed, this, &WbMotor::updateSound);
  connect(mMuscles, &WbSFNode::changed, this, &WbMotor::updateMuscles);
  connect(mMaxForceOrTorque, &WbSFDouble::changed, this, &WbMotor::updateMaxForceOrTorque);
}

void WbMotor::createWrenObjects() {
  WbJointDevice::createWrenObjects();
  WbMFIterator<WbMFNode, WbNode *> it(mMuscles);
  while (it.hasNext())
    dynamic_cast<WbMuscle *>(it.next())->createWrenObjects();
}

void WbMotor::setupJointFeedback() {
  const WbJoint *const j = joint();
  if (!j)
    return;
  dJointID jID = j->jointID();
  if (!jID)
    return;
  // check whether a feedback structure was already registered by a physics plugin
  // or a wb_motor_enable_force_feedback/wb_motor_enable_torque_feedback call,
  // otherwise create own structure
  dJointFeedback *fb = dJointGetFeedback(jID);
  if (fb)  // already set
    return;
  fb = new dJointFeedback;
  dJointSetFeedback(jID, fb);
}

double WbMotor::energyConsumption() const {
  return fabs(computeFeedback()) * mConsumptionFactor->value();
}

/////////////
// Updates //
/////////////

void WbMotor::updateMinAndMaxPosition() {
  if (mMaxPosition->value() == 0.0 && mMinPosition->value() == 0.0)
    // no limits
    return;

  WbJoint *parentJoint = dynamic_cast<WbJoint *>(parent());
  double p = 0.0;
  if (parentJoint && parentJoint->parameters())
    p = parentJoint->parameters()->position();

  // current joint position should lie between min and max position
  WbFieldChecker::checkDoubleIsGreaterOrEqual(this, mMaxPosition, p, p);
  WbFieldChecker::checkDoubleIsLessOrEqual(this, mMinPosition, p, p);

  mNeedToConfigure = true;
}

void WbMotor::updateMaxForceOrTorque() {
  WbFieldChecker::checkDoubleIsNonNegative(this, mMaxForceOrTorque, 10.0);
  mNeedToConfigure = true;
}

void WbMotor::updateMaxVelocity() {
  WbFieldChecker::checkDoubleIsNonNegative(this, mMaxVelocity, -mMaxVelocity->value());
  mNeedToConfigure = true;
}

void WbMotor::updateControlPID() {
  const WbVector3 &pid = mControlPID->value();
  const double p = pid.x();
  if (p <= 0.0)
    warn(tr("'controlP' (currently %1) must be positive.").arg(p));

  const double i = pid.y();
  if (i < 0.0)
    warn(tr("'controlI' (currently %1) must be non-negative.").arg(i));

  const double d = pid.z();
  if (d < 0.0)
    warn(tr("'controlD' (currently %1) must be non-negative.").arg(d));

  mErrorIntegral = 0.0;
  mPreviousError = 0.0;

  mNeedToConfigure = true;
}

void WbMotor::updateSound() {
  const QString &sound = mSound->value();
  if (sound.isEmpty())
    mSoundClip = NULL;
  else
    mSoundClip = WbSoundEngine::sound(WbUrl::computePath(this, "sound", sound));
  WbSoundEngine::clearAllMotorSoundSources();
}

void WbMotor::updateMuscles() {
  WbMFIterator<WbMFNode, WbNode *> it(mMuscles);
  while (it.hasNext()) {
    WbMuscle *muscle = dynamic_cast<WbMuscle *>(it.next());
    assert(muscle);
    connect(mMinPosition, &WbSFDouble::changed, muscle, &WbMuscle::updateRadius, Qt::UniqueConnection);
    connect(mMaxPosition, &WbSFDouble::changed, muscle, &WbMuscle::updateRadius, Qt::UniqueConnection);
  }
  setupJointFeedback();
}

void WbMotor::updateMaxAcceleration() {
  WbFieldChecker::checkDoubleIsNonNegativeOrDisabled(this, mAcceleration, -1, -1);
  mNeedToConfigure = true;
}

void WbMotor::setMaxVelocity(double v) {
  mTargetVelocity = v;
  awake();
}

void WbMotor::setMaxAcceleration(double acc) {
  mAcceleration->setValue(acc);
  awake();
}

double WbMotor::computeCurrentDynamicVelocity(double ms, double position) {
  if (mMotorForceOrTorque == 0.0) {  // no control
    mCurrentVelocity = 0.0;
    return mCurrentVelocity;
  }

  // compute required velocity
  double velocity = -mTargetVelocity;
  if (!std::isinf(mTargetPosition)) {  // PID-position control
    const double error = mTargetPosition - position;
    const double ts = ms * 0.001;
    mErrorIntegral += error * ts;
    const double errorDerivative = (error - mPreviousError) / ts;
    const double outputValue =
      mControlPID->value().x() * error + mControlPID->value().y() * mErrorIntegral + mControlPID->value().z() * errorDerivative;
    mPreviousError = error;
    if (fabs(outputValue) < mTargetVelocity)
      velocity = -outputValue;
    else if (error < 0.0)
      velocity = mTargetVelocity;
  }

  // try to get closer to velocity
  double acc = mAcceleration->value();
  if (acc == -1.0)
    mCurrentVelocity = velocity;  // use maximum force/torque
  else {
    const double ts = ms * 0.001;
    double d = fabs(velocity - mCurrentVelocity) / ts;  // requested acceleration
    if (d < acc)
      acc = d;
    if (velocity > mCurrentVelocity)
      mCurrentVelocity += acc * ts;
    else
      mCurrentVelocity -= acc * ts;
  }

  return mCurrentVelocity;
}

// run control without physics simulation
bool WbMotor::runKinematicControl(double ms, double &position) {
  static const double TARGET_POSITION_THRESHOLD = 1e-6;
  bool doNothing = false;
  if (std::isinf(mTargetPosition)) {  // velocity control
    const double maxp = mMaxPosition->value();
    const double minp = mMinPosition->value();
    doNothing = maxp != minp && ((position >= maxp && mTargetVelocity >= 0.0) || (position <= minp && mTargetVelocity <= 0.0));
  } else
    doNothing = fabs(mTargetPosition - position) < TARGET_POSITION_THRESHOLD;

  if (doNothing) {
    mCurrentVelocity = 0.0;
    mKinematicVelocitySign = 0;
    return false;
  }

  const double ts = ms * 0.001;
  double acc = mAcceleration->value();
  if (acc == -1.0 || acc > mMotorForceOrTorque)
    acc = mMotorForceOrTorque;
  static const double TARGET_VELOCITY_THRESHOLD = 1e-6;
  if (fabs(mTargetVelocity - mCurrentVelocity) > TARGET_VELOCITY_THRESHOLD) {
    // didn't reach the target velocity yet
    if (mTargetVelocity > mCurrentVelocity) {
      mCurrentVelocity += acc * ts;
      if (mCurrentVelocity > mTargetVelocity)
        mCurrentVelocity = mTargetVelocity;
    } else {
      mCurrentVelocity -= acc * ts;
      if (mCurrentVelocity < mTargetVelocity)
        mCurrentVelocity = mTargetVelocity;
    }
  }

  if (mTargetPosition > position) {
    mKinematicVelocitySign = 1;
    position += mCurrentVelocity * ts;
    if (position > mTargetPosition)
      position = mTargetPosition;
  } else {
    mKinematicVelocitySign = -1;
    position -= mCurrentVelocity * ts;
    if (position < mTargetPosition)
      position = mTargetPosition;
  }

  return true;
}

void WbMotor::powerOn(bool e) {  // called when running out of energy with e=false
  WbDevice::powerOn(e);
  if (!e)
    mMotorForceOrTorque = 0.0;
  else
    mMotorForceOrTorque = mMaxForceOrTorque->value();
}

/////////////
// Control //
/////////////

void WbMotor::addConfigureToStream(QDataStream &stream) {
  stream << (unsigned short)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)type();
  stream << (double)mMinPosition->value();
  stream << (double)mMaxPosition->value();
  stream << (double)mMaxVelocity->value();
  stream << (double)mAcceleration->value();
  stream << (double)mMaxForceOrTorque->value();
  stream << (double)mControlPID->value().x();
  stream << (double)mControlPID->value().y();
  stream << (double)mControlPID->value().z();
  mNeedToConfigure = false;
}

void WbMotor::writeConfigure(QDataStream &stream) {
  if (mForceOrTorqueSensor)
    mForceOrTorqueSensor->connectToRobotSignal(robot());
  addConfigureToStream(stream);
}

void WbMotor::enableMotorFeedback(int rate) {
  const WbJoint *const j = joint();

  if (j == NULL) {
    warn(tr("Feedback is available for motorized joints only"));
    return;
  }

  const WbSolid *const s = j->solidEndPoint();

  if (s == NULL) {
    warn(nodeType() == WB_NODE_ROTATIONAL_MOTOR ?
           tr("wb_motor_enable_torque_feedback(): cannot be invoked for a Joint with no end point.") :
           tr("wb_motor_enable_force_feedback(): cannot be invoked for a Joint with no end point."));
    return;
  }

  if (s->physics() == NULL) {
    warn(nodeType() == WB_NODE_ROTATIONAL_MOTOR ?
           tr("wb_motor_enable_torque_feedback(): cannot be invoked for a Joint whose end point has no Physics node.") :
           tr("wb_motor_enable_force_feedback(): cannot be invoked for a Joint whose end point has no Physics node."));
    return;
  }

  const WbSolid *const us = j->upperSolid();
  if (us->physics() == NULL) {
    warn(nodeType() == WB_NODE_ROTATIONAL_MOTOR ?
           tr("wb_motor_enable_torque_feedback(): cannot be invoked because the parent Solid has no Physics node.") :
           tr("wb_motor_enable_force_feedback(): cannot be invoked because the parent Solid has no Physics node."));
    return;
  }

  dJointID jID = j->jointID();
  if (!mForceOrTorqueSensor->isEnabled() && rate) {  // switch on feedback
    // check if a feedback structure was already registered by the physics plugin
    // or some energy measurement requirement, otherwise create own structure
    dJointFeedback *fb = dJointGetFeedback(jID);
    if (!fb) {
      fb = new dJointFeedback;
      dJointSetFeedback(jID, fb);
      mHasAllocatedJointFeedback = true;
    }
  } else if (mForceOrTorqueSensor->isEnabled() && !rate) {  // switching off feedback
    if (mHasAllocatedJointFeedback) {
      const dJointFeedback *const fb = dJointGetFeedback(jID);
      delete fb;
      dJointSetFeedback(jID, NULL);
      mHasAllocatedJointFeedback = false;
    }
  }

  mForceOrTorqueSensor->setRefreshRate(rate);
}

void WbMotor::handleMessage(QDataStream &stream) {
  short command;
  stream >> (short &)command;

  switch (command) {
    case C_MOTOR_SET_POSITION: {
      double position;
      stream >> (double &)position;
      setTargetPosition(position);
      break;
    }
    case C_MOTOR_SET_VELOCITY: {
      stream >> (double &)mTargetVelocity;
      const double m = mMaxVelocity->value();
      const bool isNegative = mTargetVelocity < 0.0;
      if ((isNegative ? -mTargetVelocity : mTargetVelocity) > m) {
        warn(tr("The requested velocity %1 exceeds 'maxVelocity' = %2.").arg(mTargetVelocity).arg(m));
        mTargetVelocity = isNegative ? -m : m;
      }
      awake();
      break;
    }
    case C_MOTOR_SET_ACCELERATION: {
      double acceleration;
      stream >> (double &)acceleration;
      setMaxAcceleration(acceleration);
      break;
    }
    case C_MOTOR_SET_FORCE: {
      if (!mUserControl)  // we were previously using motor force
        turnOffMotor();
      mUserControl = true;
      stream >> (double &)mRawInput;
      if (fabs(mRawInput) > mMotorForceOrTorque) {
        if (nodeType() == WB_NODE_ROTATIONAL_MOTOR)
          warn(tr("The requested motor torque %1 exceeds 'maxTorque' = %2").arg(mRawInput).arg(mMotorForceOrTorque));
        else
          warn(tr("The requested motor force %1 exceeds 'maxForce' = %2").arg(mRawInput).arg(mMotorForceOrTorque));
        mRawInput = mRawInput >= 0.0 ? mMotorForceOrTorque : -mMotorForceOrTorque;
      }
      awake();
      break;
    }
    case C_MOTOR_SET_AVAILABLE_FORCE: {
      stream >> (double &)mMotorForceOrTorque;
      const double m = mMaxForceOrTorque->value();
      if (mMotorForceOrTorque > m) {
        if (nodeType() == WB_NODE_ROTATIONAL_MOTOR)
          warn(tr("The requested available motor torque %1 exceeds 'maxTorque' = %2").arg(mMotorForceOrTorque).arg(m));
        else
          warn(tr("The requested available motor force %1 exceeds 'maxForce' = %2").arg(mMotorForceOrTorque).arg(m));

        mMotorForceOrTorque = m;
      }
      awake();
      break;
    }
    case C_MOTOR_SET_CONTROL_PID: {
      double controlP, controlI, controlD;
      stream >> (double &)controlP;
      stream >> (double &)controlI;
      stream >> (double &)controlD;
      mControlPID->setValue(controlP, controlI, controlD);
      awake();
      break;
    }
    case C_MOTOR_FEEDBACK: {
      short rate;
      stream >> rate;
      enableMotorFeedback(rate);
      break;
    }
    case C_MOTOR_GET_ASSOCIATED_DEVICE: {
      short deviceType;
      stream >> deviceType;
      assert(mRequestedDeviceTag == NULL);
      mRequestedDeviceTag = new WbDeviceTag[1];
      WbLogicalDevice *device = getSiblingDeviceByType(deviceType);
      mRequestedDeviceTag[0] = device ? device->tag() : 0;
      break;
    }
    default:
      assert(0);  // Invaild command.
  }
}

void WbMotor::writeAnswer(QDataStream &stream) {
  if (mForceOrTorqueSensor && (refreshSensorIfNeeded() || mForceOrTorqueSensor->hasPendingValue())) {
    stream << tag();
    stream << (unsigned char)C_MOTOR_FEEDBACK;
    stream << (double)mForceOrTorqueLastValue;
    mForceOrTorqueSensor->resetPendingValue();
  }
  if (mRequestedDeviceTag != NULL) {
    stream << tag();
    stream << (unsigned char)C_MOTOR_GET_ASSOCIATED_DEVICE;
    stream << (unsigned short)mRequestedDeviceTag[0];
    delete[] mRequestedDeviceTag;
    mRequestedDeviceTag = NULL;
  }
  if (mNeedToConfigure)
    addConfigureToStream(stream);
}

bool WbMotor::refreshSensorIfNeeded() {
  if (isPowerOn() && mForceOrTorqueSensor && mForceOrTorqueSensor->needToRefresh()) {
    mForceOrTorqueLastValue = computeFeedback();
    mForceOrTorqueSensor->updateTimer();
    return true;
  }
  return false;
}

void WbMotor::reset() {
  WbJointDevice::reset();

  turnOffMotor();

  mTargetVelocity = mMaxVelocity->value();
  mCurrentVelocity = 0.0;
  mUserControl = false;
  mRawInput = 0.0;
  const WbJoint *const j = joint();
  mTargetPosition = j ? position() : 0.0;
  mErrorIntegral = 0.0;
  mPreviousError = 0.0;
  mMotorForceOrTorque = mMaxForceOrTorque->value();
}

void WbMotor::awake() const {
  const WbJoint *const j = joint();
  if (j) {
    WbSolid *const s = j->solidEndPoint();
    if (s)
      s->awake();
    return;
  }

  const WbPropeller *const p = propeller();
  const WbTrack *const t = dynamic_cast<WbTrack *>(parent());
  if (p || t) {
    WbSolid *const s = upperSolid();
    assert(s);
    s->awake();
  }
}

void WbMotor::setTargetPosition(double tp) {
  const double maxp = mMaxPosition->value();
  const double minp = mMinPosition->value();
  mTargetPosition = tp;
  const bool velocityControl = std::isinf(mTargetPosition);
  if (maxp != minp && !velocityControl) {
    if (tp > maxp) {
      mTargetPosition = maxp;
      warn(QString("too big requested position: %1 > %2").arg(tp).arg(maxp));
    } else if (tp < minp) {
      mTargetPosition = minp;
      warn(QString("too low requested position: %1 < %2").arg(tp).arg(minp));
    }
  }

  mUserControl = false;
  awake();
}

void WbMotor::resetPhysics() {
  mCurrentVelocity = 0;
}
