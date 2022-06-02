// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbConnector.hpp"

#include "WbMFNode.hpp"
#include "WbMFVector3.hpp"
#include "WbOdeContext.hpp"
#include "WbPhysics.hpp"
#include "WbRobot.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSensor.hpp"
#include "WbSimulationState.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/config.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QList>

#include <ode/ode.h>

#include <cassert>

static const double MAX_STRENGTH = DBL_MAX / 2.0;

static QList<WbConnector *> gConnectorList;

// next time that mouse motion is allowed
// static long nextMotionTime = 0;

void WbConnector::init() {
  // add myself to the list of connector
  gConnectorList.append(this);

  // init member variables
  mFaceType = UNKNOWN;
  mMinDist2 = -1.0;
  mPeer = NULL;
  mFixedJoint = 0;
  mStartup = true;
  mSensor = NULL;
  mIsJointInversed = false;
  mNeedToReconfigure = false;

  // init fields
  mType = findSFString("type");
  mIsLocked = findSFBool("isLocked");
  mAutoLock = findSFBool("autoLock");
  mUnilateralLock = findSFBool("unilateralLock");
  mUnilateralUnlock = findSFBool("unilateralUnlock");
  mDistanceTolerance = findSFDouble("distanceTolerance");
  mAxisTolerance = findSFDouble("axisTolerance");
  mRotationTolerance = findSFDouble("rotationTolerance");
  mNumberOfRotations = findSFInt("numberOfRotations");
  mSnap = findSFBool("snap");
  mTensileStrength = findSFDouble("tensileStrength");
  mShearStrength = findSFDouble("shearStrength");

  mIsInitiallyLocked[stateId()] = mIsLocked->value();
}

WbConnector::WbConnector(WbTokenizer *tokenizer) : WbSolidDevice("Connector", tokenizer) {
  init();
}

WbConnector::WbConnector(const WbConnector &other) : WbSolidDevice(other) {
  init();
}

WbConnector::WbConnector(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbConnector::~WbConnector() {
  // remove bonds (fixed joints)
  if (mPeer)
    detachFromPeer();

  if (areWrenObjectsInitialized())
    deleteWrenObjects();

  gConnectorList.removeOne(this);
}

void WbConnector::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();

  updateType();
  updateIsLocked();
  updateDistanceTolerance();
  updateAxisTolerance();
  updateRotationTolerance();
  updateNumberOfRotations();
  updateTensileStrength();
  updateShearStrength();
}

void WbConnector::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mType, &WbSFString::changed, this, &WbConnector::updateType);
  connect(mIsLocked, &WbSFBool::changed, this, &WbConnector::updateIsLocked);
  connect(mDistanceTolerance, &WbSFBool::changed, this, &WbConnector::updateDistanceTolerance);
  connect(mAxisTolerance, &WbSFDouble::changed, this, &WbConnector::updateAxisTolerance);
  connect(mRotationTolerance, &WbSFDouble::changed, this, &WbConnector::updateRotationTolerance);
  connect(mNumberOfRotations, &WbSFInt::changed, this, &WbConnector::updateNumberOfRotations);
  connect(mTensileStrength, &WbSFDouble::changed, this, &WbConnector::updateTensileStrength);
  connect(mShearStrength, &WbSFDouble::changed, this, &WbConnector::updateShearStrength);
}

void WbConnector::updateType() {
  const QString &type = mType->value();
  if (type == "symmetric")
    mFaceType = SYMMETRIC;
  else if (type == "active")
    mFaceType = ACTIVE;
  else if (type == "passive")
    mFaceType = PASSIVE;
  else {
    parsingWarn(tr("Unknown 'type' \"%1\": locking disabled.").arg(type));
    mFaceType = UNKNOWN;
  }
}

void WbConnector::updateIsLocked() {
  if (mFaceType == PASSIVE) {
    if (mIsLocked->isTrue()) {
      parsingWarn(tr("Passive connectors cannot be locked."));
      mIsLocked->setFalse();
    }
    return;
  }

  if (mIsLocked->isTrue())
    lock();
  else
    unlock();

  mNeedToReconfigure = true;
}

void WbConnector::updateNumberOfRotations() {
  if (mNumberOfRotations->value() < 0) {
    parsingWarn(tr("'numberOfRotations' must be positive or zero."));
    mNumberOfRotations->setValue(0);
  }
  applyOptionalRenderingToWren();
}

void WbConnector::updateDistanceTolerance() {
  if (mDistanceTolerance->value() < 0.0) {
    parsingWarn(tr("'distanceTolerance' must be positive or zero."));
    mDistanceTolerance->setValue(0.0);
  }
  mMinDist2 = mDistanceTolerance->value() * mDistanceTolerance->value();
}

void WbConnector::updateAxisTolerance() {
  if (mAxisTolerance->clip(0.0, M_PI))
    parsingWarn(tr("'axisTolerance' must be between 0 and pi."));
}

void WbConnector::updateRotationTolerance() {
  if (mRotationTolerance->clip(0.0, M_PI))
    parsingWarn(tr("'rotationTolerance' must between 0 and pi."));
}

void WbConnector::updateTensileStrength() {
  if (mTensileStrength->value() < 0.0 && mTensileStrength->value() != -1.0) {
    parsingWarn(tr("'tensileStrength' must be positive or -1 (infinite)."));
    mTensileStrength->setValue(-1.0);
  }
}

void WbConnector::updateShearStrength() {
  if (mShearStrength->value() < 0.0 && mShearStrength->value() != -1.0) {
    parsingWarn(tr("'shearStrength' must be positive or -1 (infinite)."));
    mShearStrength->setValue(-1.0);
  }
}

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

// rotate both (parent) bodies such that the connectors x-axes
// become anti-parallel (collinear but in opposite directions)
// each body performs half of the necessary rotation
// output: q, the half rotation quaternion
void WbConnector::snapXAxes(WbConnector *other, dQuaternion q) {
  // x-axes of connector 1 and 2
  WbVector3 x1 = xAxis();
  WbVector3 x2 = -other->xAxis();

  // find rotation axis using cross product of x-axes
  WbVector3 w = x1.cross(x2);

  // if x1 and x2 are collinear we are already x-aligned
  if (w.isNull())
    return;  // nothing to do

  if (upperSolid()->bodyMerger() && other->upperSolid()->bodyMerger())  // rotate b1 and b2 towards each other halfway
    dQFromAxisAndAngle(q, w[0], w[1], w[2], unitVectorsAngle(x1, x2) / 2.0);
  else  // rotate only one body (the other one is static)
    dQFromAxisAndAngle(q, w[0], w[1], w[2], unitVectorsAngle(x1, x2));
  rotateBodies(other, q);
}

// search for possible rotational alignment matching alpha angle
// (thanks to problem symmetry we need to look only in 180°)
// input: alpha angle (angle between z-vectors of connectors)
// returns: -1.0 if no matching z-alignment was found
double WbConnector::findClosestRotationalAlignment(double alpha) const {
  int n = mNumberOfRotations->value();
  double angleStep = 2.0 * M_PI / n;
  double t = mRotationTolerance->value();
  double beta = 0.0;
  for (int i = 0; i < n / 2 + 1; i++) {
    if (alpha > beta - t && alpha < beta + t)
      return beta;
    else
      beta += angleStep;
  }

  return -1.0;
}

// rotate both (parent) bodies such that the connectors z-axes
// correspond to the closest allowed rotational alignment
// each body performs half of the necessary rotation
void WbConnector::snapRotation(WbConnector *other, const WbVector3 &z1, const WbVector3 &z2) {
  // if n == 0 we don't need to mSnap
  const int n = mNumberOfRotations->value();
  if (n == 0)
    return;  // nothing to do

  // use dot product to find angle of rotation
  // z1.z2 = |z1|*|z2| * cos(alpha)
  // (but |z1| == |z2| == 1.0)
  double alpha = unitVectorsAngle(z1, z2);

  // if the vectors are collinear (parallel) there is nothing to do
  if (alpha == 0.0)
    return;

  // find w rotation axis from z1 to z2
  WbVector3 w = z1.cross(z2);

  // special case: if z1 and z2 are anti-parallel we set w manually
  if (w.isNull()) {
    w[0] = 0.0;
    w[1] = 0.0;
    w[2] = 1.0;
    alpha = M_PI;
  }

  // search for possible rotational alignment
  // we should always find a rotational alignment
  const double beta = findClosestRotationalAlignment(alpha);
  assert(beta != -1.0);

  dQuaternion q;
  if (upperSolid()->bodyMerger() && other->upperSolid()->bodyMerger())
    dQFromAxisAndAngle(q, w[0], w[1], w[2], (alpha - beta) / 2.0);  // rotate b1 and b2 towards each other halfway
  else
    dQFromAxisAndAngle(q, w[0], w[1], w[2], alpha - beta);  // rotate b1 or b2 towards the other
  rotateBodies(other, q);
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

  // retrieve current body positions
  const dReal *d1 = b1 ? dBodyGetPosition(b1) : matrix().translation().ptr();
  const dReal *d2 = b2 ? dBodyGetPosition(b2) : other->matrix().translation().ptr();

  // each body must be shifted towards the other by half the distance
  dReal h[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
  if (b1 && b2) {
    for (int i = 0; i < 3; ++i)
      h[i] /= 2.0;
  }

// gcc 12.1.0 on Windows is raising a false positive warning here about dangling pointers
#pragma GCC diagnostic push
#ifdef _WIN32
#pragma GCC diagnostic ignored "-Wdangling-pointer"
#endif
  // shift bodies
  if (b1)
    dBodySetPosition(b1, d1[0] + h[0], d1[1] + h[1], d1[2] + h[2]);
  if (b2)
    dBodySetPosition(b2, d2[0] - h[0], d2[1] - h[1], d2[2] - h[2]);
#pragma GCC diagnostic pop
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

// this function must be called once the connectors are aligned
void WbConnector::createFixedJoint(WbConnector *other) {
  dBodyID b1 = upperSolid()->bodyMerger();
  dBodyID b2 = other->upperSolid()->bodyMerger();
  if (!b1 && !b2) {
    warn(tr("Connectors could not be attached because none of their parent nodes have Physics nodes."));
    return;
  }

  // marriage
  mPeer = other;
  other->mPeer = this;

  // create fixed joint: now the "snapped" relationship between the dBodies is registered
  mFixedJoint = b1 ? dJointCreateFixed(dBodyGetWorld(b1), 0) : dJointCreateFixed(dBodyGetWorld(b2), 0);
  if (b1 && b2)
    dJointAttach(mFixedJoint, b1, b2);
  else if (b1)
    dJointAttach(mFixedJoint, NULL, b1);
  else if (b2) {
    mIsJointInversed = true;
    dJointAttach(mFixedJoint, NULL, b2);
  } else
    assert(0);
  dJointSetFixed(mFixedJoint);

  // if necessary add feedback structure to joint
  if (mTensileStrength->value() != -1.0 || mShearStrength->value() != -1.0 || mPeer->mTensileStrength->value() != -1.0 ||
      mPeer->mShearStrength->value() != -1.0)
    dJointSetFeedback(mFixedJoint, new dJointFeedback);
}

void WbConnector::attachTo(WbConnector *other) {
  assert(!mPeer);

  // if other connector is already attached: give up
  if (other->mPeer)
    return;

  // either mUnilateralLock or the other's side agreement is required to lock
  if (!(mUnilateralLock->isTrue() || other->mIsLocked->isTrue()))
    return;

  dBodyID b1 = upperSolid()->bodyMerger();
  dBodyID b2 = other->upperSolid()->bodyMerger();
  if (!b1 && !b2) {
    warn(tr("Connectors could not be attached because none of their parent nodes have Physics nodes."));
    return;
  }

  if (mSnap->isTrue()) {
    // store current position and orientation
    dReal p1[3], p2[3];
    dQuaternion q1, q2;
    if (b1) {
      memcpy(p1, dBodyGetPosition(b1), 3 * sizeof(dReal));
      memcpy(q1, dBodyGetQuaternion(b1), sizeof(dQuaternion));
    }
    if (b2) {
      memcpy(p2, dBodyGetPosition(b2), 3 * sizeof(dReal));
      memcpy(q2, dBodyGetQuaternion(b2), sizeof(dQuaternion));
    }

    // move the bodies to the snapped position
    snapNow(other);
    // attach now !
    createFixedJoint(other);
    // restore original position and orientation
    if (b1) {
      dBodySetPosition(b1, p1[0], p1[1], p1[2]);
      dBodySetQuaternion(b1, q1);
    }
    if (b2) {
      dBodySetPosition(b2, p2[0], p2[1], p2[2]);
      dBodySetQuaternion(b2, q2);
    }
  } else
    createFixedJoint(other);
}

// destroy ODE fixed joint and remove feedback structure
void WbConnector::destroyFixedJoint() {
  dJointFeedback *fb = dJointGetFeedback(mFixedJoint);
  delete fb;
  dJointDestroy(mFixedJoint);
  mFixedJoint = NULL;
}

void WbConnector::detachFromPeer() {
  assert(mPeer);

  // only one fixed joint is used to hold the two bodies together
  // find on which side of the connection it is and destroy it
  if (mFixedJoint)
    destroyFixedJoint();
  else
    mPeer->destroyFixedJoint();

  // detaching connectors may cause some motion that wasn't possible when they were attached to each other
  // therefore we need to explicitely awake both of them in case they were idle
  // so that the physics engine can generate their motion accordingly
  awake();
  mPeer->awake();

  // divorce
  mPeer->mPeer = NULL;
  mPeer = NULL;
}

double WbConnector::getEffectiveTensileStrength() const {
  if (mIsLocked->isFalse())
    return 0.0;
  else if (mTensileStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mTensileStrength->value();
}

double WbConnector::getEffectiveShearStrength() const {
  if (mIsLocked->isFalse())
    return 0.0;
  else if (mShearStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mShearStrength->value();
}

// check if the force exterted on the mFixedJoint exceeds the limit, if it does, detach the connectors
// Note that this function is called for just one of the two connectors that build up each connection
void WbConnector::detachIfForceExceedStrength() {
  assert(mPeer && mFixedJoint);

  dJointFeedback *fb = dJointGetFeedback(mFixedJoint);
  if (!fb)
    return;  // user does not want rupture simulation

  // the tensile direction corresponds to the positive x-axis
  // compute how much of the measured force is aligned with the x-axis
  const WbVector3 f1(fb->f1);
  const double xforce = mIsJointInversed ? -xAxis().dot(f1) : xAxis().dot(f1);

  // check for tensile rupture
  double maxTension = getEffectiveTensileStrength() + mPeer->getEffectiveTensileStrength();

  // we are interested only in the positive x-direction
  double tension = xforce < 0.0 ? 0.0 : xforce;
  if (tension > maxTension) {
    detachFromPeer();
    return;
  }

  // check for shear rupture
  double maxShear = getEffectiveShearStrength() + mPeer->getEffectiveShearStrength();
  if (maxShear < MAX_STRENGTH) {
    // find shear force (using Pythagoras theorem)
    double magnitude = f1.length();
    double shearing = sqrt(magnitude * magnitude - xforce * xforce);
    if (shearing > maxShear) {
      detachFromPeer();
      return;
    }
  }
}

void WbConnector::prePhysicsStep(double ms) {
  bool skipAttach = mPeer != NULL;
  if (mFixedJoint)
    detachIfForceExceedStrength();

  // handle pre-locked mStartup case (must be done once only)
  if (mStartup) {
    if (mIsLocked->isTrue())
      lock();
    mStartup = false;
  }

  // call baseclass
  WbSolidDevice::prePhysicsStep(ms);

  // autolocking (if not just detached)
  if (!skipAttach && !mPeer && mAutoLock->isTrue() && mIsLocked->isTrue()) {
    WbConnector *presence = detectPresence();
    if (presence)
      attachTo(presence);
  }
}

// locking required by controller
void WbConnector::lock() {
  if (mFaceType == PASSIVE) {
    parsingWarn(tr("Passive connectors cannot lock."));
    return;
  }

  mIsLocked->setTrue();

  if (!mPeer) {
    WbConnector *presence = detectPresence();
    if (presence)
      attachTo(presence);
  }
}

// unlocking required by controller
void WbConnector::unlock() {
  if (mFaceType == PASSIVE) {
    parsingWarn(tr("Passive connectors cannot lock."));
    return;
  }

  mIsLocked->setValue(false);

  if (mPeer && (mUnilateralUnlock->isTrue() || mPeer->mIsLocked->isFalse()))
    detachFromPeer();
}

// search in the whole list a connector that meets the
// compatibility and alignment criteria to attach
WbConnector *WbConnector::detectPresence() const {
  double min2 = 99999.9;
  WbConnector *result = NULL;
  foreach (WbConnector *c, gConnectorList) {
    if (c != this && isReadyToAttachTo(c)) {
      double dist2 = getDistance2(c);
      if (dist2 <= mMinDist2) {  // is near enough ?
        if (dist2 < min2) {
          min2 = dist2;
          result = c;
        }
      }
    }
  }

  return result;
}

bool WbConnector::isCompatibleWith(const WbConnector *other) const {
  int a = faceType();
  int b = other->faceType();

  // test face mType compatibility
  if (!((a == SYMMETRIC && b == SYMMETRIC) || (a == ACTIVE && b == PASSIVE) || (a == PASSIVE && b == ACTIVE)))
    return false;

  // test model compatibility
  return model() == other->model();
}

// returns true if this connector and the other connectors x-axes are parallel (but with opposite directions)
// In other words, the angle between them must be 180° with some tolerance
bool WbConnector::isXAlignedWith(const WbConnector *other) const {
  // the vector [ matrix[8], matrix[9], matrix[10] ] represents a connectors x-axis
  // orientation in global coordinate system, its length is approximately 1.0
  return unitVectorsAngle(xAxis(), other->xAxis()) > M_PI - mAxisTolerance->value();
}

// returns true if this connector and the other connector's z-axes are
// rotationally aligned according to mNumberOfRotations and mRotationTolerance
bool WbConnector::isZAlignedWith(const WbConnector *other) const {
  // if n == 0 any rotational alignment is fine
  if (mNumberOfRotations->isZero())
    return true;

  // compare the connectors z-axis orientation in global coordinate system
  // (the vector [ matrix[4], matrix[5], matrix[6] ] represents a connectors z-axis
  // orientation in global coordinate system, its length is approximately 1.0)
  double alpha = unitVectorsAngle(zAxis(), other->zAxis());

  // search for matching alignment
  return findClosestRotationalAlignment(alpha) != -1.0;
}

double WbConnector::getDistance2(const WbConnector *other) const {
  return (matrix().translation() - other->matrix().translation()).length2();
}

bool WbConnector::isAlignedWith(const WbConnector *other) const {
  return isXAlignedWith(other) && isZAlignedWith(other);
}

bool WbConnector::isReadyToAttachTo(const WbConnector *other) const {
  return isCompatibleWith(other) && isAlignedWith(other);
}

void WbConnector::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_CONNECTOR_GET_PRESENCE:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    case C_CONNECTOR_LOCK:
      lock();
      return;
    case C_CONNECTOR_UNLOCK:
      unlock();
      return;
    default:
      assert(0);
  }
}

void WbConnector::computeValue() {
  if (faceType() == PASSIVE)
    mValue = -1;
  else if (mPeer)
    mValue = 1;
  else
    mValue = detectPresence() ? 1 : 0;
}

bool WbConnector::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void WbConnector::reset(const QString &id) {
  WbSolidDevice::reset(id);
  mIsLocked->setValue(mIsInitiallyLocked[id]);
  if (mPeer)
    detachFromPeer();
  mStartup = true;
}

void WbConnector::save(const QString &id) {
  WbSolidDevice::save(id);
  mIsInitiallyLocked[id] = mIsLocked->value();
}

void WbConnector::writeAnswer(QDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    computeValue();
    stream << (unsigned short int)tag();
    stream << (unsigned char)C_CONNECTOR_GET_PRESENCE;
    stream << (unsigned short int)mValue;
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void WbConnector::writeConfigure(QDataStream &) {
  if (robot())
    mSensor->connectToRobotSignal(robot());
}

void WbConnector::addConfigure(QDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (unsigned char)(mIsLocked->value() ? 1 : 0);
  mNeedToReconfigure = false;
}

// converts a rotation from quaternion to euler axis/angle representation
// input: normalized quaternion 'q' in ODE compatible format:  [ w x y z ]
// output: euler axis and angle 'aa' (VRML-like) format: [ x y z alpha ]
static inline void quaternionToAxesAndAngle(const double q[4], double aa[4]) {
#ifndef NDEBUG  // ensure that the quaternion is normalized as it should be
  double nn = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  assert(nn > 0.9999999999 && nn < 1.0000000001);
#endif
  // if q[0] is slightly greater than 1 or slightly lower than -1, acos(q[0]) will return nan
  // unfortunately, due to floating point rounding, that happens even if the quaternion was just normalized
  if (q[0] >= 1.0)
    aa[3] = 0.0;
  else if (q[0] <= -1.0)
    aa[3] = 2.0 * M_PI;
  else
    aa[3] = 2.0 * acos(q[0]);

  if (aa[3] < 0.0001) {  // if aa[3] is close to zero, then the direction of the axis is not important
    aa[0] = 0.0;
    aa[1] = 1.0;
    aa[2] = 0.0;
    aa[3] = 0.0;
  } else {  // normalise axes
    double n = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    aa[0] = q[1] / n;
    aa[1] = q[2] / n;
    aa[2] = q[3] / n;
  }
}

// ---------- below: auto-assembly mechanism ------------------------------------------------------------

void WbConnector::assembleAxes(WbConnector *other) {
  // we need to apply the rotation to the whole solid not only the Connector
  WbSolid *solid = topSolid();

  // find current roation of the solid: q
  dQuaternion q;
  dQFromAxisAndAngle(q, solid->rotation().x(), solid->rotation().y(), solid->rotation().z(), solid->rotation().angle());

  // x-axes of both connectors
  WbVector3 x1 = xAxis();
  WbVector3 x2 = -other->xAxis();

  // find rotation axis w using cross product of x-axes
  WbVector3 w = x1.cross(x2);

  // if x1 and x2 are collinear we are already x-aligned
  if (!w.isNull()) {
    // set quaternion (r) to represent the required rotation
    dQuaternion r, k;
    dQFromAxisAndAngle(r, w[0], w[1], w[2], unitVectorsAngle(x1, x2));

    // rotate q by r and store result in k
    dQMultiply0(k, r, q);

    // quaternion to axes/angle
    double e[4];
    quaternionToAxesAndAngle(k, e);

    // move recursively all the solid parts
    solid->setRotation(e[0], e[1], e[2], e[3]);
    solid->resetPhysics();

    // q = k
    memcpy(q, k, sizeof(dQuaternion));
  }

  // if n == 0 we don't need to rotate
  int n = mNumberOfRotations->value();
  if (n) {
    // z-axes of connector 1 and 2
    // z1 and z2 have unit length
    WbVector3 z1 = zAxis();
    WbVector3 z2 = other->zAxis();

    // find required angle of rotation from z1 to z2
    double alpha = unitVectorsAngle(z1, z2);

    // if the y vectors are parallel we don't need to rotate
    if (alpha) {
      // find w, the rotation axis from z1 to z2
      w = z1.cross(z2);

      // special case: if z1 and z2 are anti-parallel:
      // rotate of 180° around x-axis
      if (w.isNull()) {
        w[0] = 0.0;
        w[1] = 0.0;
        w[2] = 1.0;
        alpha = M_PI;
      }

      // search for possible rotational alignment
      // we should always find a rotational alignment
      double beta = findClosestRotationalAlignment(alpha);
      assert(beta != -1.0);

      // set quaternion (r) to represent the required rotation
      dQuaternion r, k;
      dQFromAxisAndAngle(r, w[0], w[1], w[2], alpha - beta);

      // rotate q by r and store result in k
      dQMultiply0(k, r, q);

      double e[4];
      quaternionToAxesAndAngle(k, e);
      solid->setRotation(e[0], e[1], e[2], e[3]);
      solid->resetPhysics();
    }
  }

  // position of both connectors
  WbVector3 p1 = matrix().translation();
  WbVector3 p2 = other->matrix().translation();

  // translation from connector 1 to connector
  WbVector3 t = p2 - p1;

  // if necessary translate whole solid
  if (!t.isNull())
    solid->translate(t[0], t[1], t[2]);

  // update ODE bodies
  // solid->setBody();
}

void WbConnector::assembleWith(WbConnector *other) {
  assembleAxes(other);

  if (mAutoLock->isTrue())
    mIsLocked->setValue(true);

  if (mIsLocked->isTrue())
    createFixedJoint(other);
}

void WbConnector::hasMoved() {
  // see who is close now
  WbConnector *presence = detectPresence();

  if (!presence && mPeer)
    detachFromPeer();
  else if (presence && !mPeer) {
    assembleWith(presence);
    // block object motion for 800 milliseconds
    // nextMotionTime = wxGetLocalTimeMillis() + 800;
  }
}

// look recursively through Solid and notify each Connector
void WbConnector::solidHasMoved(WbSolid *solid) {
  // when the simulation is running we don's allow changes
  if (WbSimulationState::instance()->isFast())
    return;

  WbConnector *connector = dynamic_cast<WbConnector *>(solid);
  if (connector)
    connector->hasMoved();
  else {
    const QVector<WbSolid *> &solidChildren = solid->solidChildren();
    foreach (WbSolid *solid, solidChildren)
      solidHasMoved(solid);
  }
}

void WbConnector::updateLineScale() {
  if (areWrenObjectsInitialized()) {
    const float ls = wr_config_get_line_scale();
    const float scale[3] = {ls, ls, ls};
    wr_transform_set_scale(mTransform, scale);
  }
}

/*
bool WbConnector::isAllowingMouseMotion() {
  return wxGetLocalTimeMillis() > nextMotionTime;
}*/

void WbConnector::createWrenObjects() {
  mTransform = wr_transform_new();
  mAxesTransform = wr_transform_new();
  mRotationsTransform = wr_transform_new();
  // Connector axes: X = red, Z = blue, Y = black
  const float colors[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};

  for (int i = 0; i < 3; ++i) {
    mMaterial[i] = wr_phong_material_new();
    wr_phong_material_set_color(mMaterial[i], colors[i]);
    wr_material_set_default_program(mMaterial[i], WbWrenShaders::lineSetShader());
  }

  // Axes (X & Z only)
  const float axesCoordinates[2][6] = {{0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f}};

  for (int i = 0; i < 2; ++i) {
    mAxisMesh[i] = wr_static_mesh_line_set_new(2, axesCoordinates[i], NULL);

    mAxisRenderable[i] = wr_renderable_new();
    wr_renderable_set_cast_shadows(mAxisRenderable[i], false);
    wr_renderable_set_receive_shadows(mAxisRenderable[i], false);
    wr_renderable_set_visibility_flags(mAxisRenderable[i], WbWrenRenderingContext::VF_CONNECTOR_AXES);
    wr_renderable_set_drawing_mode(mAxisRenderable[i], WR_RENDERABLE_DRAWING_MODE_LINES);
    wr_renderable_set_mesh(mAxisRenderable[i], WR_MESH(mAxisMesh[i]));
    wr_renderable_set_material(mAxisRenderable[i], mMaterial[i], NULL);

    wr_transform_attach_child(mAxesTransform, WR_NODE(mAxisRenderable[i]));
  }
  wr_transform_attach_child(mTransform, WR_NODE(mAxesTransform));

  // Rotation alignements (mesh is constructed in 'applyOptionalRenderingToWren')
  mRotationsRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mRotationsRenderable, false);
  wr_renderable_set_receive_shadows(mRotationsRenderable, false);
  wr_renderable_set_visibility_flags(mRotationsRenderable, WbWrenRenderingContext::VF_CONNECTOR_AXES);
  wr_renderable_set_drawing_mode(mRotationsRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_material(mRotationsRenderable, mMaterial[2], NULL);
  mRotationsMesh = NULL;

  wr_transform_attach_child(mRotationsTransform, WR_NODE(mRotationsRenderable));
  wr_transform_attach_child(mTransform, WR_NODE(mRotationsTransform));

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::optionalRenderingChanged, this,
          &WbConnector::updateOptionalRendering);
  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this, &WbConnector::updateLineScale);

  WbSolidDevice::createWrenObjects();

  wr_transform_attach_child(wrenNode(), WR_NODE(mTransform));
  applyOptionalRenderingToWren();
  updateOptionalRendering(WbWrenRenderingContext::VF_CONNECTOR_AXES);
}

void WbConnector::updateOptionalRendering(int option) {
  if (option == WbWrenRenderingContext::VF_CONNECTOR_AXES) {
    if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(option))
      wr_node_set_visible(WR_NODE(mTransform), true);
    else
      wr_node_set_visible(WR_NODE(mTransform), false);
  }
}

void WbConnector::applyOptionalRenderingToWren() {
  if (!areWrenObjectsInitialized())
    return;

  wr_static_mesh_delete(mRotationsMesh);
  mRotationsMesh = NULL;

  // draw rotational alignments in black
  // the first aligmnent is the z-axis so we skip it
  const int n = mNumberOfRotations->value();
  if (n > 1) {
    float *vertices = new float[(n - 1) * 3 * 2];
    const float angleStep = 2.0f * M_PI / n;
    for (int i = 1; i < n; ++i) {
      const float angle = angleStep * i;
      const float y = 0.4f * sin(angle);
      const float z = 0.4f * cos(angle);

      const int idx = (i - 1) * 6;
      // Segment origin
      vertices[idx] = 0.0f;
      vertices[idx + 1] = 0.0f;
      vertices[idx + 2] = 0.0f;

      // Segment orientation
      vertices[idx + 3] = 0.0f;
      vertices[idx + 4] = y;
      vertices[idx + 5] = z;
    }

    mRotationsMesh = wr_static_mesh_line_set_new((n - 1) * 2, vertices, NULL);
    wr_renderable_set_mesh(mRotationsRenderable, WR_MESH(mRotationsMesh));
    wr_node_set_visible(WR_NODE(mRotationsTransform), true);

    delete[] vertices;
  } else
    wr_node_set_visible(WR_NODE(mRotationsTransform), false);

  updateLineScale();
}

void WbConnector::deleteWrenObjects() {
  wr_node_delete(WR_NODE(mTransform));
  wr_node_delete(WR_NODE(mAxesTransform));
  wr_node_delete(WR_NODE(mAxisRenderable[0]));
  wr_node_delete(WR_NODE(mAxisRenderable[1]));
  wr_node_delete(WR_NODE(mRotationsTransform));
  wr_node_delete(WR_NODE(mRotationsRenderable));
  wr_static_mesh_delete(mAxisMesh[0]);
  wr_static_mesh_delete(mAxisMesh[1]);
  wr_static_mesh_delete(mRotationsMesh);

  for (int i = 0; i < 3; ++i)
    wr_material_delete(mMaterial[i]);
}
