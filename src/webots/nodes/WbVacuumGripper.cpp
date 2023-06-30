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

#include "WbVacuumGripper.hpp"

#include "WbBasicJoint.hpp"
#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSFDouble.hpp"
#include "WbSensor.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QList>

#include <ode/ode.h>

#include <cassert>

static const double MAX_STRENGTH = DBL_MAX / 2.0;

void WbVacuumGripper::init() {
  // init member variables
  mFixedJoint = 0;
  mSolid = NULL;
  mSensor = NULL;
  mNeedToReconfigure = false;

  // init fields
  mIsOn = findSFBool("isOn");
  mTensileStrength = findSFDouble("tensileStrength");
  mShearStrength = findSFDouble("shearStrength");
  mContactPoints = findSFInt("contactPoints");

  mIsInitiallyOn[stateId()] = mIsOn->value();
}

WbVacuumGripper::WbVacuumGripper(WbTokenizer *tokenizer) : WbSolidDevice("VacuumGripper", tokenizer) {
  init();
}

WbVacuumGripper::WbVacuumGripper(const WbVacuumGripper &other) : WbSolidDevice(other) {
  init();
}

WbVacuumGripper::WbVacuumGripper(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbVacuumGripper::~WbVacuumGripper() {
  // remove bonds (fixed joints)
  if (mSolid)
    detachFromSolid();
}

void WbVacuumGripper::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();

  updateIsOn();
  updateTensileStrength();
  updateShearStrength();
  updateContactPoints();
}

void WbVacuumGripper::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mIsOn, &WbSFBool::changed, this, &WbVacuumGripper::updateIsOn);
  connect(mTensileStrength, &WbSFDouble::changed, this, &WbVacuumGripper::updateTensileStrength);
  connect(mShearStrength, &WbSFDouble::changed, this, &WbVacuumGripper::updateShearStrength);
}

void WbVacuumGripper::updateIsOn() {
  if (mIsOn->isTrue())
    turnOn();
  else
    turnOff();

  mNeedToReconfigure = true;
}

void WbVacuumGripper::updateTensileStrength() {
  if (mTensileStrength->value() < 0.0 && mTensileStrength->value() != -1.0) {
    parsingWarn(tr("'tensileStrength' must be positive or -1 (infinite)."));
    mTensileStrength->setValue(-1.0);
  }
}

void WbVacuumGripper::updateShearStrength() {
  if (mShearStrength->value() < 0.0 && mShearStrength->value() != -1.0) {
    parsingWarn(tr("'shearStrength' must be positive or -1 (infinite)."));
    mShearStrength->setValue(-1.0);
  }
}

void WbVacuumGripper::updateContactPoints() {
  WbFieldChecker::resetIntIfNonPositive(this, mContactPoints, 3);
}

void WbVacuumGripper::createFixedJoint(WbSolid *other) {
  // retrieve body merger
  const dBodyID b1 = WbNodeUtilities::findBodyMerger(this);
  const dBodyID b2 = WbNodeUtilities::findBodyMerger(other);
  if (!b1 && !b2) {
    warn(tr(
      "VacuumGripper could not be attached because neither the VacuumGripper node nor the solid object have Physics nodes."));
    return;
  }

  mSolid = other;
  connect(mSolid, &WbSolid::destroyed, this, &WbVacuumGripper::destroyFixedJoint);

  // create fixed joint
  mFixedJoint = b1 ? dJointCreateFixed(dBodyGetWorld(b1), 0) : dJointCreateFixed(dBodyGetWorld(b2), 0);
  if (b1 && b2)
    dJointAttach(mFixedJoint, b1, b2);
  else if (b1)
    dJointAttach(mFixedJoint, NULL, b1);
  else if (b2)
    dJointAttach(mFixedJoint, NULL, b2);
  dJointSetFixed(mFixedJoint);

  // if necessary add feedback structure to joint
  if (mTensileStrength->value() != -1.0 || mShearStrength->value() != -1.0)
    dJointSetFeedback(mFixedJoint, new dJointFeedback);
}

void WbVacuumGripper::destroyFixedJoint() {
  // destroy ODE fixed joint and remove feedback structure
  dJointFeedback *fb = dJointGetFeedback(mFixedJoint);
  delete fb;
  dJointDestroy(mFixedJoint);
  mFixedJoint = NULL;
  mSolid = NULL;
}

void WbVacuumGripper::detachFromSolid() {
  assert(mSolid);
  WbSolid *attachedSolid = mSolid;

  disconnect(mSolid, &WbSolid::destroyed, this, &WbVacuumGripper::destroyFixedJoint);
  destroyFixedJoint();

  // detaching may cause some motion that wasn't possible when they were attached to each other
  // therefore we need to explicitely awake both of them in case they were idle
  // so that the physics engine can generate their motion accordingly
  awake();
  attachedSolid->awake();
}

double WbVacuumGripper::getEffectiveTensileStrength() const {
  if (mIsOn->isFalse())
    return 0.0;
  else if (mTensileStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mTensileStrength->value();
}

double WbVacuumGripper::getEffectiveShearStrength() const {
  if (mIsOn->isFalse())
    return 0.0;
  else if (mShearStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mShearStrength->value();
}

// check if the force exterted on the mFixedJoint exceeds the limit, if it does, detach the dBodies
void WbVacuumGripper::detachIfForceExceedStrength() {
  assert(mSolid && mFixedJoint);

  dJointFeedback *fb = dJointGetFeedback(mFixedJoint);
  if (!fb)
    return;  // user does not want rupture simulation

  // the tensile direction corresponds to the positive x-axis
  // compute how much of the measured force is aligned with the x-axis
  const WbVector3 f1(fb->f1);
  const double xforce = xAxis().dot(f1);

  // check for tensile rupture
  // we are interested only in the positive x-direction
  const double tension = xforce < 0.0 ? 0.0 : xforce;
  if (tension > getEffectiveTensileStrength()) {
    detachFromSolid();
    return;
  }

  // check for shear rupture
  const double maxShear = getEffectiveShearStrength();
  if (maxShear < MAX_STRENGTH) {
    // find shear force (using Pythagoras theorem)
    double magnitude = f1.length();
    double shearing = sqrt(magnitude * magnitude - xforce * xforce);
    if (shearing > maxShear) {
      detachFromSolid();
      return;
    }
  }
}

void WbVacuumGripper::prePhysicsStep(double ms) {
  mCollidedSolidList.clear();

  if (mFixedJoint)
    detachIfForceExceedStrength();

  // call base class
  WbSolidDevice::prePhysicsStep(ms);
}

void WbVacuumGripper::postPhysicsStep() {
  WbSolidDevice::postPhysicsStep();

  if (isWaitingForConnection() && !mCollidedSolidList.isEmpty())
    attachToSolid();
}

bool WbVacuumGripper::isWaitingForConnection() {
  return mIsOn->isTrue() && !mSolid;
}

void WbVacuumGripper::addCollidedSolid(WbSolid *solid, const double depth) {
  assert(solid);
  if (mSolid)
    return;  // ignore if already connected to another object
  mCollidedSolidList << std::pair<WbSolid *, const double>(solid, depth);
}

void WbVacuumGripper::attachToSolid() {
  // search for solid to connect to
  double maxDepth = 0;
  WbSolid *solid = NULL;
  QListIterator<std::pair<WbSolid *, const double>> it(mCollidedSolidList);
  while (it.hasNext()) {
    std::pair<WbSolid *, const double> item = it.next();
    if (item.second > maxDepth) {
      maxDepth = item.second;
      solid = item.first;
    }
  }

  if (solid)
    createFixedJoint(solid);
}

void WbVacuumGripper::turnOn() {
  mIsOn->setTrue();
  if (!mSolid)
    attachToSolid();
}

void WbVacuumGripper::turnOff() {
  mIsOn->setValue(false);
  if (mSolid)
    detachFromSolid();
}

void WbVacuumGripper::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_VACUUM_GRIPPER_GET_PRESENCE:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    case C_VACUUM_GRIPPER_TURN_ON:
      turnOn();
      return;
    case C_VACUUM_GRIPPER_TURN_OFF:
      turnOff();
      return;
    default:
      assert(0);
  }
}

void WbVacuumGripper::computeValue() {
  mValue = mSolid != NULL;
}

bool WbVacuumGripper::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void WbVacuumGripper::reset(const QString &id) {
  WbSolidDevice::reset(id);
  mIsOn->setValue(mIsInitiallyOn[id]);
  if (mSolid)
    detachFromSolid();
  mNeedToReconfigure = true;
}

void WbVacuumGripper::save(const QString &id) {
  WbSolidDevice::save(id);
  mIsInitiallyOn[id] = mIsOn->value();
}

void WbVacuumGripper::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    computeValue();
    stream << (unsigned short int)tag();
    stream << (unsigned char)C_VACUUM_GRIPPER_GET_PRESENCE;  // return if an object is connected
    stream << (unsigned char)(mValue ? 1 : 0);
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void WbVacuumGripper::writeConfigure(WbDataStream &) {
  if (robot())
    mSensor->connectToRobotSignal(robot());
}

void WbVacuumGripper::addConfigure(WbDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (unsigned char)(mIsOn->value() ? 1 : 0);
  mNeedToReconfigure = false;
}
