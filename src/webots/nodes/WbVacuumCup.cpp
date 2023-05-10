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

#include "WbVacuumCup.hpp"

#include "WbDataStream.hpp"
#include "WbSFDouble.hpp"
#include "WbSensor.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QList>

#include <ode/ode.h>

#include <cassert>

static const double MAX_STRENGTH = DBL_MAX / 2.0;

void WbVacuumCup::init() {
  // init member variables
  mFixedJoint = 0;
  mSolid = NULL;
  mSensor = NULL;
  mNeedToReconfigure = false;

  // init fields
  mIsOn = findSFBool("isOn");
  mTensileStrength = findSFDouble("tensileStrength");
  mShearStrength = findSFDouble("shearStrength");

  mIsInitiallyOn[stateId()] = mIsOn->value();
}

WbVacuumCup::WbVacuumCup(WbTokenizer *tokenizer) : WbSolidDevice("VacuumCup", tokenizer) {
  init();
}

WbVacuumCup::WbVacuumCup(const WbVacuumCup &other) : WbSolidDevice(other) {
  init();
}

WbVacuumCup::WbVacuumCup(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbVacuumCup::~WbVacuumCup() {
  // remove bonds (fixed joints)
  if (mSolid)
    detachFromSolid();

  // if (areWrenObjectsInitialized())
  //   deleteWrenObjects();
}

void WbVacuumCup::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();

  updateIsOn();
  updateTensileStrength();
  updateShearStrength();
}

void WbVacuumCup::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mIsOn, &WbSFBool::changed, this, &WbVacuumCup::updateIsOn);
  connect(mTensileStrength, &WbSFDouble::changed, this, &WbVacuumCup::updateTensileStrength);
  connect(mShearStrength, &WbSFDouble::changed, this, &WbVacuumCup::updateShearStrength);
}

void WbVacuumCup::updateIsOn() {
  if (mIsOn->isTrue())
    turnOn();
  else
    turnOff();

  mNeedToReconfigure = true;
}

void WbVacuumCup::updateTensileStrength() {
  if (mTensileStrength->value() < 0.0 && mTensileStrength->value() != -1.0) {
    parsingWarn(tr("'tensileStrength' must be positive or -1 (infinite)."));
    mTensileStrength->setValue(-1.0);
  }
}

void WbVacuumCup::updateShearStrength() {
  if (mShearStrength->value() < 0.0 && mShearStrength->value() != -1.0) {
    parsingWarn(tr("'shearStrength' must be positive or -1 (infinite)."));
    mShearStrength->setValue(-1.0);
  }
}

/*
// return the angle (in the range [0 pi]) between two approximately normalized vectors
// this function should be used when it is known that v1 and v2 have approx lentgh = 1.0
// In this case it is faster than: acos((v1.v2)/|v1|.|v2|)
static inline double unitVectorsAngle(const WbVector3 &v1, const WbVector3 &v2) {
  double cos = v1.dot(v2);
  if (cos >= 1.0)
    return 0.0;
  else if (cos <= -1.0)
    return M_PI;
  else
    return acos(cos);
}

// rotate vector v by quaternion q
static inline void rotateVector(const dQuaternion q, WbVector3 &v) {
  double v1 = v[0];
  double v2 = v[1];
  double v3 = v[2];
  double t2 = q[0] * q[1];
  double t3 = q[0] * q[2];
  double t4 = q[0] * q[3];
  double t5 = -q[1] * q[1];
  double t6 = q[1] * q[2];
  double t7 = q[1] * q[3];
  double t8 = -q[2] * q[2];
  double t9 = q[2] * q[3];
  double t10 = -q[3] * q[3];
  v[0] = 2.0 * ((t8 + t10) * v1 + (t6 - t4) * v2 + (t3 + t7) * v3) + v1;
  v[1] = 2.0 * ((t4 + t6) * v1 + (t5 + t10) * v2 + (t9 - t2) * v3) + v2;
  v[2] = 2.0 * ((t7 - t3) * v1 + (t2 + t9) * v2 + (t5 + t8) * v3) + v3;
}

// rotate "this" connector's parent dBody by q
// and rotate "other" connector's parent dBody by inverse of q
void WbConnector::rotateBodies(WbConnector *other, const dQuaternion q) {
  dBodyID b1 = upperSolid()->bodyMerger();
  dBodyID b2 = other->upperSolid()->bodyMerger();

  // get current quaternions of bodies
  const dReal *q1 = b1 ? dBodyGetQuaternion(b1) : NULL;
  const dReal *q2 = b2 ? dBodyGetQuaternion(b2) : NULL;

  // multiply quaternions (rotation)
  dQuaternion q1n, q2n;
  if (q1)
    dQMultiply0(q1n, q, q1);  // q
  if (q2)
    dQMultiply1(q2n, q, q2);  // inverse of q

  // change current bodies orientation
  if (q1)
    dBodySetQuaternion(b1, q1n);
  if (q2)
    dBodySetQuaternion(b2, q2n);
}

// return the vrml origin ([0 0 0] point) of the connector in world (global) coordinate system
void WbConnector::getOriginInWorldCoordinates(dReal out[3]) const {
  const WbVector3 &globalTranslation = matrix().translation();
  out[0] = globalTranslation[0];
  out[1] = globalTranslation[1];
  out[2] = globalTranslation[2];
}

// shift both connectors (parent) bodies such that the connectors VRML origins match
// the shift is performed halfway by each body
void WbConnector::snapOrigins(WbConnector *other) {
  dBodyID b1 = upperSolid()->bodyMerger();
  dBodyID b2 = other->upperSolid()->bodyMerger();

  // get positions of connector 1 and 2
  dVector3 p1, p2;
  getOriginInWorldCoordinates(p1);
  other->getOriginInWorldCoordinates(p2);

  // each body must be shifted towards the other by half the distance
  dReal h[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
  if (b1 && b2) {
    for (int i = 0; i < 3; ++i)
      h[i] /= 2.0;
  }

  // retrive current body positions and shift them
  if (b1) {
    const dReal *d1 = dBodyGetPosition(b1);
    dBodySetPosition(b1, d1[0] + h[0], d1[1] + h[1], d1[2] + h[2]);
  }
  if (b2) {
    const dReal *d2 = dBodyGetPosition(b2);
    dBodySetPosition(b2, d2[0] - h[0], d2[1] - h[1], d2[2] - h[2]);
  }
}

// temporarily change body position and orientation so that the fixed joint
// will be created with the adjusted ("snapped") relative position and
// orientation between the two bodies
void WbConnector::snapNow(WbConnector *other) {
  // rotate bodies such that x-axes become aligned and return corresponding quaternion
  dQuaternion qa;
  snapXAxes(other, qa);

  // z-axes of connector 1 and 2
  // z1 and z2 have unit length
  WbVector3 z1 = zAxis();
  WbVector3 z2 = other->zAxis();

  // aq = inversion of qa rotation
  dQuaternion aq = {qa[0], -qa[1], -qa[2], -qa[3]};

  // rotate y vectors to take into account previous rotation carried out by snapXAxes()
  rotateVector(qa, z1);
  rotateVector(aq, z2);

  // now mSnap rotational alignement (z-axes)
  snapRotation(other, z1, z2);

  // finally shift bodies such that the CS origins match
  snapOrigins(other);
}
*/

void WbVacuumCup::createFixedJoint(WbSolid *other) {
  const dBodyID b1 = upperSolid()->bodyMerger();
  const dBodyID b2 = other->bodyMerger();
  assert(b1 && b2);

  mSolid = other;
  connect(mSolid, &WbSolid::destroyed, this, &WbVacuumCup::destroyFixedJoint);

  // create fixed joint
  mFixedJoint = dJointCreateFixed(dBodyGetWorld(b1), 0);
  if (b2)
    dJointAttach(mFixedJoint, b1, b2);
  else
    dJointAttach(mFixedJoint, NULL, b1);
  dJointSetFixed(mFixedJoint);

  // if necessary add feedback structure to joint
  if (mTensileStrength->value() != -1.0 || mShearStrength->value() != -1.0)
    dJointSetFeedback(mFixedJoint, new dJointFeedback);
}

void WbVacuumCup::destroyFixedJoint() {
  // destroy ODE fixed joint and remove feedback structure
  dJointFeedback *fb = dJointGetFeedback(mFixedJoint);
  delete fb;
  dJointDestroy(mFixedJoint);
  mFixedJoint = NULL;
  mSolid = NULL;
}

void WbVacuumCup::detachFromSolid() {
  assert(mSolid);
  WbSolid *attachedSolid = mSolid;

  disconnect(mSolid, &WbSolid::destroyed, this, &WbVacuumCup::destroyFixedJoint);
  destroyFixedJoint();

  // detaching may cause some motion that wasn't possible when they were attached to each other
  // therefore we need to explicitely awake both of them in case they were idle
  // so that the physics engine can generate their motion accordingly
  awake();
  attachedSolid->awake();
}

double WbVacuumCup::getEffectiveTensileStrength() const {
  if (mIsOn->isFalse())
    return 0.0;
  else if (mTensileStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mTensileStrength->value();
}

double WbVacuumCup::getEffectiveShearStrength() const {
  if (mIsOn->isFalse())
    return 0.0;
  else if (mShearStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mShearStrength->value();
}

// check if the force exterted on the mFixedJoint exceeds the limit, if it does, detach the dBodies
void WbVacuumCup::detachIfForceExceedStrength() {
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

void WbVacuumCup::prePhysicsStep(double ms) {
  mCollidedSolidList.clear();

  if (mFixedJoint)
    detachIfForceExceedStrength();

  // call base class
  WbSolidDevice::prePhysicsStep(ms);
}

void WbVacuumCup::postPhysicsStep() {
  WbSolidDevice::postPhysicsStep();

  if (isWaitingForConnection() && !mCollidedSolidList.isEmpty())
    attachToSolid();
}

bool WbVacuumCup::isWaitingForConnection() {
  return mIsOn->isTrue() && !mSolid;
}

void WbVacuumCup::addCollidedSolid(WbSolid *solid, const double depth) {
  assert(solid);
  if (mSolid)
    return;  // ignore if already connected to another object
  mCollidedSolidList << std::pair<WbSolid *, const double>(solid, depth);
}

void WbVacuumCup::attachToSolid() {
  const dBodyID b1 = upperSolid()->bodyMerger();
  if (!b1) {
    warn(tr("Vacuum cup is disabled because none of its parent nodes have Physics nodes."));
    return;
  }

  // search for solid to connect to
  const WbSolid *solidAncestor = topSolid();
  double maxDepth = 0;
  WbSolid *solid = NULL;
  QListIterator<std::pair<WbSolid *, const double>> it(mCollidedSolidList);
  while (it.hasNext()) {
    std::pair<WbSolid *, const double> item = it.next();
    if (!item.first->bodyMerger() || solidAncestor == item.first->topSolid())
      // cannot connect to an object without physics or
      // that has a common ancestor solid node
      continue;
    if (item.second > maxDepth) {
      maxDepth = item.second;
      solid = item.first;
    }
  }

  if (solid)
    createFixedJoint(solid);
}

void WbVacuumCup::turnOn() {
  mIsOn->setTrue();
  if (!mSolid)
    attachToSolid();
}

void WbVacuumCup::turnOff() {
  mIsOn->setValue(false);
  if (mSolid)
    detachFromSolid();
}

void WbVacuumCup::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_VACUUM_CUP_GET_PRESENCE:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    case C_VACUUM_CUP_TURN_ON:
      turnOn();
      return;
    case C_VACUUM_CUP_TURN_OFF:
      turnOff();
      return;
    default:
      assert(0);
  }
}

void WbVacuumCup::computeValue() {
  mValue = mSolid != NULL;
}

bool WbVacuumCup::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void WbVacuumCup::reset(const QString &id) {
  WbSolidDevice::reset(id);
  mIsOn->setValue(mIsInitiallyOn[id]);
  if (mSolid)
    detachFromSolid();
  mNeedToReconfigure = true;
}

void WbVacuumCup::save(const QString &id) {
  WbSolidDevice::save(id);
  mIsInitiallyOn[id] = mIsOn->value();
}

void WbVacuumCup::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    computeValue();
    stream << (unsigned short int)tag();
    stream << (unsigned char)C_VACUUM_CUP_GET_PRESENCE;  // return if an object is connected
    stream << (unsigned char)(mValue ? 1 : 0);
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void WbVacuumCup::writeConfigure(WbDataStream &) {
  if (robot())
    mSensor->connectToRobotSignal(robot());
}

void WbVacuumCup::addConfigure(WbDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (unsigned char)(mIsOn->value() ? 1 : 0);
  mNeedToReconfigure = false;
}
