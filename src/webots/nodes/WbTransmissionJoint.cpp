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
#include "WbTransmissionJoint.hpp"

#include "WbBrake.hpp"
#include "WbFieldChecker.hpp"
#include "WbHingeJointParameters.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbOdeUtilities.hpp"
#include "WbRotationalMotor.hpp"
#include "WbSlot.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

// Constructors

void WbTransmissionJoint::init() {
  mJoint2 = NULL;
  mTransmission = NULL;
  mTransmissionMode = -1;
  mParameters2 = findSFNode("jointParameters2");
  mStartPoint = findSFNode("startPoint");
  mBacklash = findSFDouble("backlash");
  mMultiplier = findSFDouble("multiplier");
  mStartPoint = findSFNode("startPoint");

  body2 = NULL;
  // hidden field
  mPosition2 = findSFDouble("position2")->value();
  mOdePositionOffset2 = mPosition2;
  mInitialPosition2 = mPosition2;

  mDummy = false;
}

WbTransmissionJoint::WbTransmissionJoint(const QString &modelName, WbTokenizer *tokenizer) : WbJoint(modelName, tokenizer) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(WbTokenizer *tokenizer) : WbJoint("TransmissionJoint", tokenizer) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(const WbTransmissionJoint &other) : WbJoint(other) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(const WbNode &other) : WbJoint(other) {
  init();
}

WbTransmissionJoint::~WbTransmissionJoint() {
  // delete feedback;
}

void WbTransmissionJoint::preFinalize() {
  printf("preFinalize\n");
  WbJoint::preFinalize();
}

void WbTransmissionJoint::postFinalize() {
  printf("postFinalize\n");
  WbJoint::postFinalize();

  connect(mParameters2, &WbSFNode::changed, this, &WbTransmissionJoint::updateParameters2);
  connect(mBacklash, &WbSFDouble::changed, this, &WbTransmissionJoint::updateBacklash);
  connect(mMultiplier, &WbSFDouble::changed, this, &WbTransmissionJoint::updateMultiplier);

  if (!solidStartPoint())
    setupJoint2();  // create dummy body

  inferTransmissionMode();

  if (mDummy)
    dummyTransmission();
}

void WbTransmissionJoint::updateStartPointPosition() {
  printf("UNDEFIEND: updateStartPointPosition\n");
}

void WbTransmissionJoint::prePhysicsStep(double ms) {
  if (mDummy)
    return;

  /*
  const dReal *lv = dBodyGetLinearVel(solidEndPoint()->body());
  const dReal *av = dBodyGetAngularVel(solidEndPoint()->body());
  const dReal *lv2 = dBodyGetLinearVel(body2);
  const dReal *av2 = dBodyGetAngularVel(body2);
  printf("B1V  %.10f %.10f %.10f | %.10f %.10f %.10f\n", lv[0], lv[1], lv[2], av[0], av[1], av[2]);
  printf("B2V  %.10f %.10f %.10f | %.10f %.10f %.10f\n", lv2[0], lv2[1], lv2[2], av2[0], av2[1], av2[2]);

  const dReal *pos = dBodyGetPosition(solidEndPoint()->body());
  const dReal *pos2 = dBodyGetPosition(body2);
  const dReal *rot = dBodyGetRotation(solidEndPoint()->body());
  const dReal *rot2 = dBodyGetRotation(body2);

  printf("B1P  %.10f %.10f %.10f\n", pos[0], pos[1], pos[2]);
  printf("B2P  %.10f %.10f %.10f\n", pos2[0], pos2[1], pos2[2]);

  printf("B1R  %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", rot[0], rot[1], rot[2], rot[3], rot[4], rot[5],
         rot[6], rot[7], rot[8], rot[9], rot[10], rot2[11]);
  printf("B2R  %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n\n", rot2[0], rot2[1], rot2[2], rot2[3], rot2[4],
         rot2[5], rot2[6], rot2[7], rot2[8], rot2[9], rot2[10], rot2[11]);
  */

  if (mTransmissionMode == -1) {
    printf("shouldn't execture physics step if transmission isn't defined\n");
  }
  // printf("prePhysicsStep\n");

  assert(solidEndPoint() && mJoint2);
  WbRotationalMotor *const rm = rotationalMotor();
  WbHingeJointParameters *const p = hingeJointParameters();

  if (rm && rm->userControl()) {
    printf("TODO - undefined yet\n");
  } else {
    // ODE motor torque (user velocity/position control)
    double currentVelocity = rm ? rm->computeCurrentDynamicVelocity(ms, mPosition2) : 0.0;  // pos or pos2 ??

    // if user desires staticFriction, it has to specify it on hingeJointParameters. Give warning if set on
    // hingeJointParameters2. This value is applied to the driving axis.
    double fMax = p ? p->staticFriction() : 0.0;
    fMax = qMax(fMax * abs(mMultiplier->value()), rm ? rm->torque() * abs(mMultiplier->value()) : 0.0);
    const double s = upperTransform()->absoluteScale().x();
    double s4 = s * s;
    s4 *= s4;
    dJointSetHingeParam(mJoint2, dParamFMax, s * s4 * fMax);
    dJointSetHingeParam(mJoint2, dParamVel, -currentVelocity);
  }
  mTimeStep = ms;
  // printf("prePhysicsStep done\n");
}

void WbTransmissionJoint::postPhysicsStep() {
  if (mDummy) {
    // double ar1 = dJointGetHingeAngleRate(mJoint);
    // ouble ar2 = dJointGetHingeAngleRate(mJoint2);
    // printf("anglerate: %.10lf | %.10lf\n", ar1, ar2);
    return;
  }
  // printf("postPhysicsStep\n");
  assert(mJoint && mJoint2);
  WbRotationalMotor *const rm = rotationalMotor();
  if (rm && rm->isPIDPositionControl()) {  // if controlling in position we update position using directly the angle feedback
    printf("TODO - undefined yet\n");
  } else {
    double angleRate = dJointGetHingeAngleRate(mJoint);
    mPosition += -angleRate * mTimeStep / 1000.0;
    double angleRate2 = dJointGetHingeAngleRate(mJoint2);
    mPosition2 += angleRate2 * mTimeStep / 1000.0;
  }
  WbHingeJointParameters *const p = hingeJointParameters();
  if (p)
    p->setPositionFromOde(mPosition);
  WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (p2)
    p2->setPositionFromOde(mPosition2);

  // double ar1 = dJointGetHingeAngleRate(mJoint);
  // double ar2 = dJointGetHingeAngleRate(mJoint2);
  // printf("T anglerate: %.10lf | %.10lf\n", ar1, ar2);
  // printf("T mPosition = %lf, mPosition2 = %lf\n", mPosition, mPosition2);
  // printf("postPhysicsStep done\n");
}

void WbTransmissionJoint::reset() {
  // reset endpoint
  WbBaseNode::reset();
  WbNode *const p = mParameters->value();
  WbNode *const e = mEndPoint->value();
  if (p)
    p->reset();
  if (e)
    e->reset();

  for (int i = 0; i < mDevice->size(); ++i)
    mDevice->item(i)->reset();

  setPosition(mInitialPosition, 1);

  // NB: WbRotationalMotor::turnOffMotor resets dParamFMax to 0 even if a staticFriction is specified because for
  // transmissions having non-zero dParamFMax on the output axis causes errors.

  // reset startPoint/dummy body2
  if (body2 != NULL) {  // && !solidStartPoint()
    // using dummy body
    dJointSetHingeParam(mJoint2, dParamVel, 0.0);
    // this might be unnecessary if prePhysicsStep enforces it every time
    WbHingeJointParameters *const p = hingeJointParameters();
    dJointSetHingeParam(mJoint2, dParamFMax, p ? p->staticFriction() * abs(mMultiplier->value()) : 0.0);
    dBodySetLinearVel(body2, 0, 0, 0);
    dBodySetAngularVel(body2, 0, 0, 0);
    // const dReal *pos = dBodyGetPosition(solidEndPoint()->body());
    // ^ not working, not updated yet?... need a better way. Maybe store initial pos+rot of endpoint and reload it here
    dBodySetPosition(body2, 0.0, 0.12501, 0.07);
    dMatrix3 R;
    dRSetIdentity(R);
    dBodySetRotation(body2, R);

  } else if (solidStartPoint()) {
    printf("TODO reset startpoint\n");
  }

  WbNode *const p2 = mParameters2->value();
  if (p2)
    p2->reset();

  setPosition(mInitialPosition2, 2);
}

void WbTransmissionJoint::setPosition(double position, int index) {
  // set position of mJoint
  if (index == 1) {
    mPosition = position;
    mOdePositionOffset = position;
    WbJointParameters *const p = hingeJointParameters();
    if (p)
      p->setPosition(mPosition);
    WbMotor *const m = motor();
    if (m)
      m->setTargetPosition(position);
  }
  // set position of mJoint2
  if (index == 2) {
    mPosition2 = position;
    mOdePositionOffset = position;
    WbJointParameters *const p2 = hingeJointParameters2();
    if (p2)
      p2->setPosition(mPosition2);
  }
}
void WbTransmissionJoint::save() {
  WbJoint::save();

  // save mJoint2
  printf("TODO in save\n");
}

WbHingeJointParameters *WbTransmissionJoint::hingeJointParameters() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters->value());
}

WbHingeJointParameters *WbTransmissionJoint::hingeJointParameters2() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters2->value());
}

WbRotationalMotor *WbTransmissionJoint::rotationalMotor() const {
  WbRotationalMotor *motor = NULL;
  for (int i = 0; i < mDevice->size(); ++i) {
    motor = dynamic_cast<WbRotationalMotor *>(mDevice->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

WbVector3 WbTransmissionJoint::anchor() const {
  static const WbVector3 DEFAULT_ANCHOR(0.0, 0.0, 0.0);
  const WbHingeJointParameters *const p = hingeJointParameters();
  return p ? p->anchor() : DEFAULT_ANCHOR;
}

WbVector3 WbTransmissionJoint::anchor2() const {
  static const WbVector3 DEFAULT_ANCHOR(0.0, 0.0, 0.0);
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  return p2 ? p2->anchor() : DEFAULT_ANCHOR;
}

WbVector3 WbTransmissionJoint::axis() const {
  static const WbVector3 DEFAULT_AXIS(1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p = hingeJointParameters();
  return p ? p->axis() : DEFAULT_AXIS;
}

WbVector3 WbTransmissionJoint::axis2() const {
  static const WbVector3 DEFAULT_AXIS(1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  return p2 ? p2->axis() : DEFAULT_AXIS;
}

bool WbTransmissionJoint::setJoint() {
  printf("setJoint\n");
  if (!WbBasicJoint::setJoint()) {
    parsingWarn(tr("TransmissionJoint requires the solid endpoint to have physics enabled."));
    return false;
  }

  if (mJoint == NULL) {
    mJoint = dJointCreateHinge(WbOdeContext::instance()->world(), 0);
    printf("created mJoint hinge\n");
  }
  const WbSolid *const s = solidEndPoint();
  setOdeJoint(s ? s->body() : NULL, upperSolid()->bodyMerger());

  return true;
}

bool WbTransmissionJoint::setJoint2() {
  printf("UNDEFINED - setJoint2\n");
  // TODO
  return true;
}

WbSolidReference *WbTransmissionJoint::solidReferenceStartPoint() const {
  WbSlot *slot = dynamic_cast<WbSlot *>(mStartPoint->value());
  if (slot) {
    WbSlot *childrenSlot = slot->slotEndPoint();
    if (childrenSlot)
      return childrenSlot->solidReferenceEndPoint();
    else
      return NULL;
  } else
    return dynamic_cast<WbSolidReference *>(mStartPoint->value());
}

WbSolid *WbTransmissionJoint::solidStartPoint() const {
  WbSlot *slot = dynamic_cast<WbSlot *>(mStartPoint->value());
  if (slot) {
    WbSlot *childrenSlot = slot->slotEndPoint();
    if (childrenSlot) {
      WbSolid *solid = childrenSlot->solidEndPoint();
      if (solid)
        return solid;

      WbSolidReference *solidReference = childrenSlot->solidReferenceEndPoint();
      if (solidReference)
        return solidReference->solid();
    }
  } else {
    WbSolid *solid = dynamic_cast<WbSolid *>(mStartPoint->value());
    if (solid)
      return solid;

    const WbSolidReference *const solidReference = dynamic_cast<WbSolidReference *>(mStartPoint->value());
    if (solidReference)
      return solidReference->solid();
  }

  return NULL;
}

void WbTransmissionJoint::setOdeJoint(dBodyID body, dBodyID parentBody) {
  WbJoint::setOdeJoint(body, parentBody);
  // compute and set the anchor point
  applyToOdeAnchor();
}

void WbTransmissionJoint::applyToOdeAxis() {
  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  WbVector3 a = m4.sub3x3MatrixDot(axis());
  dJointSetHingeAxis(mJoint, a.x(), a.y(), a.z());
}

void WbTransmissionJoint::applyToOdeAxis2() {
  printf("UNDEFINED - applyToOdeAxis2\n");
}

void WbTransmissionJoint::applyToOdeAnchor() {
  assert(mJoint);

  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &t = m4 * anchor();
  dJointSetHingeAnchor(mJoint, t.x(), t.y(), t.z());
}

void WbTransmissionJoint::applyToOdeAnchor2() {
  printf("UNDEFINED - applyToOdeAnchor2\n");
}

void WbTransmissionJoint::updateOdePositionOffset() {
  double newValue = position();
  mOdePositionOffset = newValue;
}

void WbTransmissionJoint::updateOdePositionOffset2() {
  double newValue = position(2);
  mOdePositionOffset2 = newValue;
}

double WbTransmissionJoint::position(int index) const {
  switch (index) {
    case 1:
      return mPosition;
    case 2:
      return mPosition2;
    default:
      return NAN;
  }
}

void WbTransmissionJoint::updateAxis() {
  printf("updateAxis\n");
  // update the current endPoint pose based on the new axis value
  updatePosition();

  if (mJoint)
    applyToOdeAxis();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  inferTransmissionMode();
}

void WbTransmissionJoint::updateAxis2() {
  printf("UNDEFINED for startPoint - updateAxis2\n");
  // update the current startPoint pose based on the new axis value
  /*
  updatePosition2();

  if (mJoint2)
    applyToOdeAxis2();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();
  */
  inferTransmissionMode();
}

void WbTransmissionJoint::updateAnchor() {
  printf("updateAnchor\n");
  // update the current endPoint pose based on the new anchor value
  updatePosition();

  if (mJoint)
    applyToOdeAnchor();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  inferTransmissionMode();
}

void WbTransmissionJoint::updateAnchor2() {
  printf("UNDEFINED for startPoint - updateAnchor2\n");
  // update the current startPoint pose based on the new anchor value
  /*
  updatePosition2();

  if (mJoint2)
    applyToOdeAnchor2();

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();
  */
  inferTransmissionMode();
}

double WbTransmissionJoint::initialPosition(int index) const {
  switch (index) {
    case 1:
      return mInitialPosition;
    case 2:
      return mInitialPosition2;
    default:
      return NAN;
  }
}

// Updates

void WbTransmissionJoint::updateParameters() {
  printf("updateParameters\n");
  WbHingeJointParameters *const p = hingeJointParameters();
  if (p) {
    mOdePositionOffset = p->position();
    mPosition = mOdePositionOffset;
    connect(p, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor, Qt::UniqueConnection);
  }

  // updateAxis(); // causes issues by putting solid in a weird place
  // updateAnchor();
}

void WbTransmissionJoint::updateParameters2() {
  printf("updateParameters2\n");
  WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (p2) {
    connect(p2, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis2, Qt::UniqueConnection);
    connect(p2, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor2, Qt::UniqueConnection);
  }
  // updateAxis2();
  // updateAnchor2();
}

void WbTransmissionJoint::updatePosition() {
  const WbHingeJointParameters *const p = hingeJointParameters();

  if (solidReference() == NULL && solidEndPoint())
    updatePosition(p ? p->position() : mPosition);
}

void WbTransmissionJoint::updatePosition2() {
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (mJoint2)
    updatePosition2(p2 ? p2->position() : mPosition2);
}

void WbTransmissionJoint::updatePosition(double position) {
  WbSolid *const s = solidEndPoint();
  assert(s);
  // called after an artificial move
  mPosition = position;
  WbMotor *m = motor();
  if (m && !m->isConfigureDone())
    m->setTargetPosition(position);
  WbVector3 translation;
  WbRotation rotation;
  computeEndPointSolidPositionFromParameters(translation, rotation);
  if (!translation.almostEquals(s->translation()) || !rotation.almostEquals(s->rotation())) {
    mIsEndPointPositionChangedByJoint = true;
    s->setTranslationAndRotation(translation, rotation);
    s->resetPhysics();
    mIsEndPointPositionChangedByJoint = false;
  }
}

void WbTransmissionJoint::updatePosition2(double position) {
  printf("UNDEFINED - updatePosition2\n");
  // TODO
}

void WbTransmissionJoint::updateEndPointZeroTranslationAndRotation() {
  if (solidEndPoint() == NULL)
    return;

  WbRotation ir;
  WbVector3 it;
  retrieveEndPointSolidTranslationAndRotation(it, ir);

  WbQuaternion qMinus;
  const double angle = mPosition;
  if (WbMathsUtilities::isZeroAngle(angle)) {
    // In case of a zero angle, the quaternion axis is undefined, so we keep track of the original one
    mEndPointZeroRotation = ir;
  } else {
    const WbVector3 &ax = axis().normalized();
    qMinus = WbQuaternion(ax, -angle);
    const WbQuaternion &q = ir.toQuaternion();
    WbQuaternion qNormalized = qMinus * q;
    if (qNormalized.w() != 1.0)
      qNormalized.normalize();
    mEndPointZeroRotation = WbRotation(qNormalized);
    if (mEndPointZeroRotation.angle() == 0.0)
      mEndPointZeroRotation = WbRotation(ax.x(), ax.y(), ax.z(), 0.0);
  }
  const WbVector3 &an = anchor();
  mEndPointZeroTranslation = qMinus * (it - an) + an;
}

void WbTransmissionJoint::computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const {
  const WbVector3 &ax = axis().normalized();
  const WbQuaternion q(ax, mPosition);
  const WbQuaternion iq(mEndPointZeroRotation.toQuaternion());
  WbQuaternion qp(q * iq);
  if (qp.w() != 1.0)
    qp.normalize();
  rotation.fromQuaternion(qp);
  if (rotation.angle() == 0.0)
    rotation = WbRotation(ax.x(), ax.y(), ax.z(), 0.0);
  const WbVector3 &a = anchor();
  translation = q * (mEndPointZeroTranslation - a) + a;
}

void WbTransmissionJoint::updateBacklash() {
  WbFieldChecker::resetDoubleIfNegative(this, mBacklash, 0.0);
  printf("new backlash %f\n", mBacklash->value());

  inferTransmissionMode();
}

void WbTransmissionJoint::updateMultiplier() {
  if (mMultiplier->isZero()) {
    mMultiplier->setValue(1);
    parsingWarn(tr("'multiplier' must be different from zero, setting it back to 1."));
  }

  if (mMultiplier->value() == 1 && mBacklash->isZero())
    parsingWarn(tr("For multipliers of mangitude 1 and zero backlash consider using a single hinge instead as the transmission "
                   "might introduce imprecisions."));

  printf("new multiplier = %f\n", mMultiplier->value());
  inferTransmissionMode();
}

void WbTransmissionJoint::inferTransmissionMode() {
  printf("inferTransmissionMode\n");
  mTransmissionMode = -1;

  const bool isCodirectional = axis().normalized().almostEquals(axis2().normalized());
  if (mMultiplier->value() < 0.0 && isCodirectional)
    mTransmissionMode = dTransmissionParallelAxes;
  else if (mMultiplier->value() > 0.0 && isCodirectional)
    mTransmissionMode = dTransmissionChainDrive;
  else {
    // determine if they intersect
    const bool isCoplanar = fabs(axis().cross(axis2()).dot(anchor() - anchor2())) < 1e-10;
    const bool isParallel = fabs(axis().cross(axis2()).length2()) < 1e-10;
    if (isCoplanar && !isParallel)
      mTransmissionMode = dTransmissionIntersectingAxes;
  }

  switch (mTransmissionMode) {
    case dTransmissionParallelAxes:
      printf("geartype = CLASSIC GEAR\n");
      break;
    case dTransmissionChainDrive:
      printf("geartype = CHAIN DRIVE\n");
      break;
    case dTransmissionIntersectingAxes:
      printf("geartype = BEVEL GEAR\n");
      break;
    default:
      printf("geartype = UNDEFINED\n");
  }

  configureTransmission();
}

void WbTransmissionJoint::setupJoint2() {
  printf("setupJoint2\n");

  if (body2) {
    printf("already exists... why?\n");
    return;
  }

  if (solidEndPoint() == NULL)
    return;

  // assert(solidEndPoint());

  body2 = dBodyCreate(WbOdeContext::instance()->world());
  dBodySetFiniteRotationMode(body2, 1);

  if (!dBodyIsKinematic(solidEndPoint()->body())) {
    printf("setting mass to body2\n");
    dMass mass;
    dMassSetBox(&mass, 0.0001, 1, 1, 1);  // if startPoint isn't defined, give body2 a negligeable mass
    dBodySetMass(body2, &mass);
    mJoint2 = dJointCreateHinge(WbOdeContext::instance()->world(), 0);
    dJointAttach(mJoint2, body2, 0);

    const dReal *pos = dBodyGetPosition(solidEndPoint()->body());
    dBodySetPosition(body2, pos[0], pos[1], pos[2]);
    // dBodySetPosition(body2, 0, 0.125, 0.07);  // TO REMOVE, see ^

    dMatrix3 R;
    dRSetIdentity(R);
    dBodySetRotation(body2, R);

    const WbVector3 &ax2 = axis2();
    const WbVector3 &an2 = anchor2();
    dJointSetHingeAnchor(mJoint2, an2.x(), an2.y(), an2.z());
    dJointSetHingeAxis(mJoint2, ax2.x(), ax2.y(), ax2.z());
  } else {
    parsingWarn(tr("Solid endPoint must have physics enabled."));
  }

  printf("setupJoint2 done\n");
}

void WbTransmissionJoint::dummyTransmission() {
  printf("!!!!!!!!!!!! USING DUMMY !!!!!!!!!!!!!!!!\n");
  // set body
  body1 = dBodyCreate(WbOdeContext::instance()->world());
  body2 = dBodyCreate(WbOdeContext::instance()->world());

  dBodySetFiniteRotationMode(body1, 1);
  dBodySetFiniteRotationMode(body2, 1);

  // geom1 = dCreateCylinder(WbOdeContext::instance()->space(), 0.05, 0.1);
  // geom2 = dCreateCylinder(WbOdeContext::instance()->space(), 0.05, 0.1);

  // dGeomSetBody(geom1, body1);
  // dGeomSetBody(geom2, body2);

  dMass mass;
  dMassSetCylinder(&mass, 100, 3, 0.2, 0.5);
  dBodySetMass(body1, &mass);
  dBodySetMass(body2, &mass);

  mJoint = dJointCreateHinge(WbOdeContext::instance()->world(), 0);
  mJoint2 = dJointCreateHinge(WbOdeContext::instance()->world(), 0);

  dJointAttach(mJoint, body1, 0);
  dJointAttach(mJoint2, body2, 0);

  mTransmission = dJointCreateTransmission(WbOdeContext::instance()->world(), 0);
  dJointAttach(mTransmission, body1, body2);
  // dJointSetFeedback(mTransmission, feedback);

  dMatrix3 R;
  dBodySetPosition(body1, 2, 0, 1);
  dBodySetPosition(body2, -2, 0, 1);

  dRSetIdentity(R);
  dBodySetRotation(body1, R);
  dBodySetRotation(body2, R);

  dJointSetHingeAnchor(mJoint2, -2, 0, 1);
  dJointSetHingeAxis(mJoint2, 0, 0, 1);

  dJointSetHingeAnchor(mJoint, 2, 0, 1);
  dJointSetHingeAxis(mJoint, 0, 0, 1);

  dJointSetTransmissionMode(mTransmission, dTransmissionChainDrive);
  dJointSetTransmissionAnchor1(mTransmission, 2, 0, 1);
  dJointSetTransmissionAnchor2(mTransmission, -2, 0, 1);
  dJointSetTransmissionRadius1(mTransmission, 1);
  dJointSetTransmissionRadius2(mTransmission, 1);

  dJointSetTransmissionAxis(mTransmission, 0, 0, 1);
  dJointSetTransmissionBacklash(mTransmission, 0.0);

  dJointSetHingeParam(mJoint2, dParamVel, 5);
  dJointSetHingeParam(mJoint2, dParamFMax, 10);

  dJointSetHingeParam(mJoint, dParamVel, 0);
  dJointSetHingeParam(mJoint, dParamFMax, 0);

  // necessary to have similar behavior between input and output
  dJointSetHingeParam(mJoint, dParamSuspensionCFM, 1e-10);
  dJointSetHingeParam(mJoint2, dParamSuspensionCFM, 1e-10);
  dJointSetTransmissionParam(mTransmission, dParamCFM, 1e-10);

  dBodySetLinearVel(body1, 0, 0, 0);
  dBodySetLinearVel(body2, 0, 0, 0);
  dBodySetAngularVel(body1, 0, 0, 0);
  dBodySetAngularVel(body2, 0, 0, 0);
}

void WbTransmissionJoint::configureTransmission() {
  printf("configureTransmission\n");
  if (mTransmissionMode == -1) {
    printf("invalid transmission mode, early exit\n");
    return;
  }

  if (mParameters->value() == NULL || mParameters2->value() == NULL) {
    printf("jointParameters or jointParameters2 not defined, early exit\n");
    return;
  }

  if (!mTransmission) {
    if (!solidEndPoint()->body() || !body2) {
      printf("body1 or body2 not defined yet, early exit\n");
      return;
    }
    mTransmission = dJointCreateTransmission(WbOdeContext::instance()->world(), 0);
    dJointAttach(mTransmission, solidEndPoint()->body(), body2);
  }

  const WbVector3 &ax1 = axis();
  const WbVector3 &ax2 = axis2();
  const WbVector3 &an1 = anchor();
  const WbVector3 &an2 = anchor2();

  dJointSetTransmissionMode(mTransmission, mTransmissionMode);

  // configure transmission
  dJointSetTransmissionAnchor1(mTransmission, an1.x(), an1.y(), an1.z());
  dJointSetTransmissionAnchor2(mTransmission, an2.x(), an2.y(), an2.z());

  dJointSetTransmissionBacklash(mTransmission, mBacklash->value());

  if (mTransmissionMode == dTransmissionParallelAxes) {
    dJointSetTransmissionRatio(mTransmission, abs(mMultiplier->value()));  // only accepts positive values, but by using
    dJointSetTransmissionAxis(mTransmission, ax1.x(), ax1.y(), ax1.z());   // parallelAxis we invert the direction
  }
  if (mTransmissionMode == dTransmissionChainDrive) {
    dJointSetTransmissionRadius1(mTransmission, 1.0);
    dJointSetTransmissionRadius2(mTransmission, mMultiplier->value());
    dJointSetTransmissionAxis(mTransmission, ax1.x(), ax1.y(), ax1.z());
  }
  if (mTransmissionMode == dTransmissionIntersectingAxes) {
    dJointSetTransmissionAxis1(mTransmission, ax1.x(), ax1.y(), ax1.z());
    dJointSetTransmissionAxis2(mTransmission, ax2.x(), ax2.y(), ax2.z());
  }

  dJointSetHingeParam(mJoint, dParamSuspensionCFM, 1e-10);
  dJointSetHingeParam(mJoint2, dParamSuspensionCFM, 1e-10);
  dJointSetTransmissionParam(mTransmission, dParamCFM, 1e-10);

  printTransmissionConfig();
}

void WbTransmissionJoint::printTransmissionConfig() {
  if (!mTransmission) {
    printf("transmission not defined yet\n");
    return;
  }
  printf("-- TRANSMISSION CONFIG ----------------------------------------------------------------------------------\n");

  switch (mTransmissionMode) {
    case 0:
      printf("Inferred mode = CLASSIC_GEAR (multiplier < 0? axis match? anchors differ?)\n");
      break;
    case 1:
      printf("Inferred mode = BEVEL_GEAR (axis differ?)\n");
      break;
    case 2:
      printf("Inferred mode = CHAIN DRIVE (multiplier > 0? axis match?)\n");
      break;
    default:
      printf("Inferred mode = UNDEFINED\n");
  }

  switch (dJointGetTransmissionMode(mTransmission)) {
    case 0:
      printf("Transmission mode = CLASSIC_GEAR\n");
      break;
    case 1:
      printf("Transmission mode = BEVEL_GEAR\n");
      break;
    case 2:
      printf("Transmission mode = CHAIN DRIVE\n");
      break;
    default:
      printf("Transmission mode = UNDEFINED\n");
  }

  dVector3 jAn;
  dVector3 jAn2;
  dVector3 jAx;
  dVector3 jAx2;
  dJointGetHingeAnchor(mJoint, jAn);
  dJointGetHingeAnchor(mJoint2, jAn2);
  dJointGetHingeAxis(mJoint, jAx);
  dJointGetHingeAxis(mJoint2, jAx2);

  printf("Joint anchors\n");
  printf("> mJoint : %f %f %f\n", jAn[0], jAn[1], jAn[2]);
  printf("> mJoint2: %f %f %f\n", jAn2[0], jAn2[1], jAn2[2]);
  printf("Joint axis\n");
  printf("> mJoint : %f %f %f\n", jAx[0], jAx[1], jAx[2]);
  printf("> mJoint2: %f %f %f\n", jAx2[0], jAx2[1], jAx2[2]);

  dVector3 tAx;
  dVector3 tAx2;
  dVector3 tAn;
  dVector3 tAn2;

  dJointGetTransmissionAnchor1(mTransmission, tAn);
  dJointGetTransmissionAnchor2(mTransmission, tAn2);
  dJointGetTransmissionAxis1(mTransmission, tAx);
  dJointGetTransmissionAxis2(mTransmission, tAx2);

  printf("Transmission anchors\n");
  printf("> anchor : %f %f %f\n", tAn[0], tAn[1], tAn[2]);
  printf("> anchor2: %f %f %f\n", tAn2[0], tAn2[1], tAn2[2]);
  printf("Transmission axis\n");
  printf("> axis : %f %f %f\n", tAx[0], tAx[1], tAx[2]);
  printf("> axis2: %f %f %f\n", tAx2[0], tAx2[1], tAx2[2]);

  if (dJointGetTransmissionMode(mTransmission) == 2) {
    // drive chain
    dReal r1 = dJointGetTransmissionRadius1(mTransmission);
    dReal r2 = dJointGetTransmissionRadius2(mTransmission);
    printf("r1 = %lf, r2 = %lf\n", r1, r2);
  }

  if (dJointGetTransmissionMode(mTransmission) == 0) {
    // classic gear
    double ratio = dJointGetTransmissionRatio(mTransmission);
    printf("ratio = %lf\n", ratio);
  }

  printf("-- END TRANSMISSION CONFIG ------------------------------------------------------------------------------\n");
  printf("USING CFM = %.15f\n", dWorldGetCFM(WbOdeContext::instance()->world()));
}

void WbTransmissionJoint::updateJointAxisRepresentation() {
  if (!areWrenObjectsInitialized())
    return;

  wr_static_mesh_delete(mMesh);

  float vertices[12];
  const double scaling = 0.5f * wr_config_get_line_scale();
  const WbVector3 &anchorVector = anchor();
  const WbVector3 &axisVector = scaling * axis();
  const WbVector3 &anchorVector2 = anchor2();
  const WbVector3 &axisVector2 = scaling * axis2();

  // joint on endPoint side
  WbVector3 vertex(anchorVector - axisVector);
  vertex.toFloatArray(vertices);

  vertex = anchorVector + axisVector;
  vertex.toFloatArray(vertices + 3);

  // joint on startPoint side
  vertex = anchorVector2 - axisVector2;
  vertex.toFloatArray(vertices + 6);

  vertex = anchorVector2 + axisVector2;
  vertex.toFloatArray(vertices + 9);

  mMesh = wr_static_mesh_line_set_new(4, vertices, NULL);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
}

void WbTransmissionJoint::applyToOdeMinAndMaxStop() {
}

void WbTransmissionJoint::applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) {
}
