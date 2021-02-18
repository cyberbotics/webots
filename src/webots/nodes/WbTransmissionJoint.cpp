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
//#include "WbJointParameters.hpp"
#include "WbBoundingSphere.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeUtilities.hpp"
#include "WbPositionSensor.hpp"
#include "WbRobot.hpp"
#include "WbRotationalMotor.hpp"
#include "WbSlot.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <ode/ode.h>
#include <wren/config.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

// Constructors

void WbTransmissionJoint::init() {
  mJoint2 = NULL;
  mParameters2 = findSFNode("jointParameters2");
  mStartPoint = findSFNode("startPoint");
  mBacklash = findSFDouble("backlash");
  mMultiplier = findSFDouble("multiplier");
  mStartPoint = findSFNode("startPoint");
  mGearType = UNDEFINED;

  mIsStartPointPositionChangedByJoint = false;

  // spring and dampingConstant
  mSpringAndDampingConstantsAxis1On = false;
  mSpringAndDampingConstantsAxis2On = false;

  // hidden field
  mPosition2 = findSFDouble("position2")->value();
  mOdePositionOffset2 = mPosition2;
  mInitialPosition2 = mPosition2;
}

WbTransmissionJoint::WbTransmissionJoint(const QString &modelName, WbTokenizer *tokenizer) :
  WbHingeJoint(modelName, tokenizer) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(WbTokenizer *tokenizer) : WbHingeJoint("TransmissionJoint", tokenizer) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(const WbTransmissionJoint &other) : WbHingeJoint(other) {
  init();
}

WbTransmissionJoint::WbTransmissionJoint(const WbNode &other) : WbHingeJoint(other) {
  init();
}

WbTransmissionJoint::~WbTransmissionJoint() {
  WbSolid *const s = solidStartPoint();
  if (s && !s->isBeingDeleted())
    s->removeJointParent(this);

  if (mJoint2)
    dJointDestroy(mJoint2);
  mJoint2 = NULL;

  /* !!!!!!!!!!!!!!!!!          ???????????????????????????????              !!!!!!!!!!!!!!!!!!!!!
  if (areWrenObjectsInitialized()) {
    wr_static_mesh_delete(mMesh2);
    wr_material_delete(mMaterial2);
    wr_node_delete(WR_NODE(mRenderable2));
    wr_node_delete(WR_NODE(mTransform2));
  }
  */
}

void WbTransmissionJoint::preFinalize() {
  // NOTE: to have a better programming interface for the user, strict inheritance isn't desirable in this class.
  // HingeJoint supports a device and refers to the endPoint but for TransmissionJoints we want this on startPoint
  // side of things (i.e axis(), anchor() of hingeJointParameters should refer to startPoint)

  // startPoint (interface-wise) and endPoint (code-wise)
  WbHingeJoint::preFinalize();
  // endPoint (interface-wise) and startPoint (code-wise)
  WbBaseNode::preFinalize();  // ???????????????????
  updateParameters2();
  updateStartPointZeroTranslationAndRotation();

  WbBaseNode *const p2 = dynamic_cast<WbBaseNode *>(mParameters2->value());
  WbBaseNode *const e2 = dynamic_cast<WbBaseNode *>(mStartPoint->value());
  if (p2 && !p2->isPreFinalizedCalled())
    p2->preFinalize();
  if (e2 && !e2->isPreFinalizedCalled())
    e2->preFinalize();

  mInitialPosition2 = mPosition2;
}

void WbTransmissionJoint::postFinalize() {
  // startPoint (interface-wise) and endPoint (code-wise)
  WbHingeJoint::postFinalize();
  // endPoint (interface-wise) and startPoint (code-wise)
  WbBaseNode::postFinalize();  // ???????????????????
  WbBaseNode *const p2 = dynamic_cast<WbBaseNode *>(mParameters2->value());
  WbBaseNode *const e2 = dynamic_cast<WbBaseNode *>(mStartPoint->value());
  if (p2 && !p2->isPostFinalizedCalled())
    p2->postFinalize();
  if (e2 && !e2->isPostFinalizedCalled())
    e2->postFinalize();

  connect(mParameters2, &WbSFNode::changed, this, &WbTransmissionJoint::updateParameters2);
  connect(mStartPoint, &WbSFNode::changed, this, &WbTransmissionJoint::updateStartPoint);

  const WbGroup *pg = dynamic_cast<WbGroup *>(parentNode());
  if (pg)
    connect(this, &WbTransmissionJoint::startPointChanged, pg, &WbGroup::insertChildFromSlotOrJoint);
  else {
    const WbSlot *slot = dynamic_cast<WbSlot *>(parentNode());
    if (slot)
      connect(this, &WbTransmissionJoint::startPointChanged, slot, &WbSlot::endPointInserted);
  }
  connect(mParameters2, &WbSFNode::changed, this, &WbTransmissionJoint::updateParameters2);

  WbSolid *const s = solidStartPoint();
  if (s) {
    connect(s, &WbSolid::positionChangedArtificially, this, &WbTransmissionJoint::updateStartPointPosition);
    updateStartPointPosition();
  }

  if (protoParameterNode()) {
    const QVector<WbNode *> nodes = protoParameterNode()->protoParameterNodeInstances();
    if (nodes.size() > 1 && nodes.at(0) == this)
      parsingWarn(tr("Joint node defined in PROTO field is used multiple times. "
                     "Webots doesn't fully support this because the multiple node instances cannot be identical."));
  }

  connect(mBacklash, &WbSFDouble::changed, this, &WbTransmissionJoint::updateBacklash);
  connect(mMultiplier, &WbSFDouble::changed, this, &WbTransmissionJoint::updateMultiplier);
}

WbHingeJointParameters *WbTransmissionJoint::hingeJointParameters2() const {
  return dynamic_cast<WbHingeJointParameters *>(mParameters2->value());
}

void WbTransmissionJoint::updateStartPointZeroTranslationAndRotation() {
  if (solidStartPoint() == NULL)
    return;

  WbRotation ir;
  WbVector3 it;
  retrieveStartPointSolidTranslationAndRotation(it, ir);

  WbQuaternion qMinus;
  const double angle = mPosition2;
  if (WbMathsUtilities::isZeroAngle(angle)) {
    // In case of a zero angle, the quaternion axis is undefined, so we keep track of the original one
    mStartPointZeroRotation = ir;
  } else {
    const WbVector3 &ax2 = axis2().normalized();
    qMinus = WbQuaternion(ax2, -angle);
    const WbQuaternion &q = ir.toQuaternion();
    WbQuaternion qNormalized = qMinus * q;
    if (qNormalized.w() != 1.0)
      qNormalized.normalize();
    mStartPointZeroRotation = WbRotation(qNormalized);
    if (mStartPointZeroRotation.angle() == 0.0)
      mStartPointZeroRotation = WbRotation(ax2.x(), ax2.y(), ax2.z(), 0.0);
  }
  const WbVector3 &an2 = anchor2();
  mStartPointZeroTranslation = qMinus * (it - an2) + an2;
}

void WbTransmissionJoint::retrieveStartPointSolidTranslationAndRotation(WbVector3 &it, WbRotation &ir) const {
  const WbSolid *const s = solidStartPoint();
  assert(s);

  if (solidReferenceStartPoint()) {
    const WbTransform *const ut = upperTransform();
    WbMatrix4 m = ut->matrix().pseudoInversed() * s->matrix();
    double scale = 1.0 / ut->absoluteScale().x();
    scale *= scale;
    m *= scale;
    ir = WbRotation(m.extracted3x3Matrix());
    it = m.translation();
  } else {
    ir = s->rotation();
    it = s->translation();
  }
}

WbVector3 WbTransmissionJoint::anchor() const {
  static const WbVector3 DEFAULT_ANCHOR(-1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p = hingeJointParameters();
  return p ? p->anchor() : DEFAULT_ANCHOR;
}

WbVector3 WbTransmissionJoint::anchor2() const {
  static const WbVector3 DEFAULT_ANCHOR(1.0, 0.0, 0.0);
  const WbHingeJointParameters *const p2 = hingeJointParameters();
  return p2 ? p2->anchor() : DEFAULT_ANCHOR;
}

WbVector3 WbTransmissionJoint::axis() const {
  static const WbVector3 DEFAULT_AXIS(0.0, 1.0, 0.0);
  const WbHingeJointParameters *const p = hingeJointParameters();
  return p ? p->axis() : DEFAULT_AXIS;
}

WbVector3 WbTransmissionJoint::axis2() const {
  static const WbVector3 DEFAULT_AXIS(0.0, 1.0, 0.0);
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  return p2 ? p2->axis() : DEFAULT_AXIS;
}

bool WbTransmissionJoint::setJoint() {
  if (!WbBasicJoint::setJoint())
    return false;

  if (mJoint == NULL)
    mJoint = dJointCreateHinge2(WbOdeContext::instance()->world(), 0);

  const WbSolid *const s = solidEndPoint();
  dBodyID body = s ? s->body() : NULL;
  dBodyID parentBody = upperSolid()->bodyMerger();
  if (body && parentBody)
    setOdeJoint(body, parentBody);
  else {
    parsingWarn(tr("TransmissionJoint nodes can only connect Solid nodes that have a Physics node."));
    return false;
  }

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  return true;
}

bool WbTransmissionJoint::setJoint2() {
  WbSolidReference *const sr = solidReferenceStartPoint();
  if (sr)
    sr->updateName();
  const WbSolid *const s = solidStartPoint();
  const bool invalidStartPoint = s == NULL && (sr == NULL || !sr->pointsToStaticEnvironment());
  if (invalidStartPoint || upperSolid() == NULL || (s && s->physics() == NULL) || (s && s->solidMerger().isNull())) {
    if (mJoint2) {
      dJointAttach(mJoint2, NULL, NULL);
      dJointDisable(mJoint2);
    }
    return false;
  }

  if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    updateJointAxisRepresentation();

  return true;
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

void WbTransmissionJoint::applyToOdeAnchor() {
  assert(mJoint);
  printf("applyToOdeAnchor from transmissionJoint called correctly\n");
  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &t = m4 * anchor();
  dJointSetHingeAnchor(mJoint, t.x(), t.y(), t.z());
}

void WbTransmissionJoint::applyToOdeAnchor2() {
  assert(mJoint2);

  updateOdePositionOffset2();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  const WbVector3 &t = m4 * anchor2();
  dJointSetHingeAnchor(mJoint2, t.x(), t.y(), t.z());
}

void WbTransmissionJoint::applyToOdeAxis() {
  updateOdePositionOffset();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  WbVector3 a = m4.sub3x3MatrixDot(axis());
  dJointSetHingeAxis(mJoint, a.x(), a.y(), a.z());
}

void WbTransmissionJoint::applyToOdeAxis2() {
  updateOdePositionOffset2();

  const WbMatrix4 &m4 = upperTransform()->matrix();
  WbVector3 a = m4.sub3x3MatrixDot(axis2());
  dJointSetHingeAxis(mJoint2, a.x(), a.y(), a.z());
}

void WbTransmissionJoint::computeStartPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const {
  const WbVector3 &ax = axis().normalized();
  const WbQuaternion q(ax, mPosition);
  const WbQuaternion iq(mStartPointZeroRotation.toQuaternion());
  WbQuaternion qp(q * iq);
  if (qp.w() != 1.0)
    qp.normalize();
  rotation.fromQuaternion(qp);
  if (rotation.angle() == 0.0)
    rotation = WbRotation(ax.x(), ax.y(), ax.z(), 0.0);
  const WbVector3 &a = anchor();
  translation = q * (mStartPointZeroTranslation - a) + a;
}

void WbTransmissionJoint::computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const {
  const WbVector3 &ax2 = axis2().normalized();
  const WbQuaternion q(ax2, mPosition2);
  const WbQuaternion iq(mEndPointZeroRotation.toQuaternion());
  WbQuaternion qp(q * iq);
  if (qp.w() != 1.0)
    qp.normalize();
  rotation.fromQuaternion(qp);
  if (rotation.angle() == 0.0)
    rotation = WbRotation(ax2.x(), ax2.y(), ax2.z(), 0.0);
  const WbVector3 &a = anchor2();
  translation = q * (mEndPointZeroTranslation - a) + a;
}

// Updates

void WbTransmissionJoint::updateParameters() {
  const WbHingeJointParameters *const p = hingeJointParameters();
  if (p) {
    mOdePositionOffset = p->position();
    mPosition = mOdePositionOffset;
    connect(p, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis, Qt::UniqueConnection);
    connect(p, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor, Qt::UniqueConnection);
  }
}

void WbTransmissionJoint::updateParameters2() {
  const WbHingeJointParameters *const p2 = hingeJointParameters2();
  if (p2) {
    mOdePositionOffset = p2->position();
    mPosition2 = mOdePositionOffset2;
    connect(p2, SIGNAL(positionChanged()), this, SLOT(updatePosition2()), Qt::UniqueConnection);
    connect(p2, &WbHingeJointParameters::axisChanged, this, &WbTransmissionJoint::updateAxis, Qt::UniqueConnection);
    connect(p2, &WbHingeJointParameters::anchorChanged, this, &WbTransmissionJoint::updateAnchor, Qt::UniqueConnection);
  }
}

void WbTransmissionJoint::updatePosition() {
  const WbHingeJointParameters *const p = hingeJointParameters();

  if (solidReferenceStartPoint() == NULL && solidStartPoint())
    updatePositionOf(1, p ? p->position() : mPosition);

  emit updateMuscleStretch(0.0, true, 1);
}

void WbTransmissionJoint::updatePosition2() {
  const WbHingeJointParameters *const p2 = hingeJointParameters2();

  if (solidReference() == NULL && solidEndPoint())
    updatePositionOf(2, p2 ? p2->position() : mPosition2);

  emit updateMuscleStretch(0.0, true, 1);
}

void WbTransmissionJoint::updatePositionOf(int index, double position) {
  if (index == 1) {
    WbSolid *const s = solidStartPoint();
    assert(s);
    mPosition = position;
    WbMotor *m1 = motor();
    if (m1 && !m1->isConfigureDone())
      m1->setTargetPosition(position);
    WbVector3 translation;
    WbRotation rotation;
    computeStartPointSolidPositionFromParameters(translation, rotation);
    if (!translation.almostEquals(s->translation()) || !rotation.almostEquals(s->rotation())) {
      mIsStartPointPositionChangedByJoint = true;
      s->setTranslationAndRotation(translation, rotation);
      s->resetPhysics();
      mIsStartPointPositionChangedByJoint = false;
    }
  }
  if (index == 2) {
    WbSolid *const s = solidEndPoint();
    assert(s);
    mPosition2 = position;
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
}

void WbTransmissionJoint::updateStartPoint() {
  WbSolidReference *const r = solidReferenceStartPoint();
  if (r)
    r->updateName();

  WbSolid *const s = solidStartPoint();
  if (s) {
    connect(s, &WbSolid::positionChangedArtificially, this, &WbTransmissionJoint::updateStartPointPosition,
            Qt::UniqueConnection);
    s->appendJointParent(this);
  }

  updateStartPointPosition();

  if (r)
    connect(r, &WbSolidReference::changed, this, &WbTransmissionJoint::setJoint2, Qt::UniqueConnection);

  if (s == NULL || s->isPostFinalizedCalled()) {
    emit startPointChanged(s);
    if (s != NULL && isPostFinalizedCalled())
      WbBoundingSphere::addSubBoundingSphereToParentNode(this);
  } else {
    connect(s, &WbBaseNode::finalizationCompleted, this, &WbTransmissionJoint::startPointChanged);
    connect(s, &WbBaseNode::finalizationCompleted, this, &WbTransmissionJoint::updateBoundingSphere);
  }
}

void WbTransmissionJoint::updateBoundingSphere(WbBaseNode *subNode) {
  disconnect(subNode, &WbBaseNode::finalizationCompleted, this, &WbTransmissionJoint::updateBoundingSphere);
  WbBoundingSphere::addSubBoundingSphereToParentNode(this);
}

void WbTransmissionJoint::updateStartPointPosition() {
  if (mIsStartPointPositionChangedByJoint)
    return;

  WbSolid *const s = solidStartPoint();
  if (s)
    updateStartPointZeroTranslationAndRotation();

  if (areOdeObjectsCreated())
    setJoint2();
}

void WbTransmissionJoint::updateJointAxisRepresentation() {
  // update joint on endPoint (user perspective, startPoint code-wise)
  /*
  if (!areWrenObjectsInitialized())
    return;

  wr_static_mesh_delete(mMesh2);

  const double scaling = 0.5f * wr_config_get_line_scale();

  const WbVector3 &anchorVector = anchor2();
  const WbVector3 &axisVector = scaling * axis2();

  WbVector3 vertex(anchorVector - axisVector);
  float vertices[6];
  vertex.toFloatArray(vertices);

  vertex = anchorVector + axisVector;
  vertex.toFloatArray(vertices + 3);

  mMesh = wr_static_mesh_line_set_new(2, vertices, NULL);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh2));
  */
  // update joint on startPoint (user perspective, endPoint code-wise)
  WbHingeJoint::updateJointAxisRepresentation();
}

void WbTransmissionJoint::updateBacklash() {
  WbFieldChecker::resetDoubleIfNegative(this, mBacklash, 0.0);
  printf("new backlash %f\n", mBacklash->value());
}

void WbTransmissionJoint::updateMultiplier() {
  if (mMultiplier->isZero()) {
    mMultiplier->setValue(1);
    parsingWarn(tr("'multiplier' must be different from zero, setting it back to 1."));
  }

  printf("new multiplier = %f\n", mMultiplier->value());

  inferGearType();
}

void WbTransmissionJoint::inferGearType() {
  mGearType = UNDEFINED;
  const bool isCodirectional = axis().normalized().almostEquals(axis2().normalized());
  if (mMultiplier->value() < 0.0 && isCodirectional)
    mGearType = CLASSIC_GEAR;
  else if (mMultiplier->value() > 0.0 && isCodirectional)
    mGearType = CHAIN_DRIVE;
  else {
    // determine if they intersect
    const bool isCoplanar = fabs(axis().cross(axis2()).dot(anchor() - anchor2())) < 1e-10;
    const bool isParallel = fabs(axis().cross(axis2()).length2()) < 1e-10;
    if (isCoplanar && !isParallel)
      mGearType = BEVEL_GEAR;
  }

  switch (mGearType) {
    case CLASSIC_GEAR:
      printf("geartype = CLASSIC GEAR\n");
      break;
    case CHAIN_DRIVE:
      printf("geartype = CHAIN DRIVE\n");
      break;
    case BEVEL_GEAR:
      printf("geartype = BEVEL GEAR\n");
      break;
    default:
      printf("geartype = UNDEFINED\n");
  }
}

// utility
void WbTransmissionJoint::setSolidStartPoint(WbSolid *solid) {
  mStartPoint->removeValue();
  mStartPoint->setValue(solid);
  updateStartPoint();
}

void WbTransmissionJoint::setSolidStartPoint(WbSolidReference *solid) {
  mStartPoint->removeValue();
  mStartPoint->setValue(solid);
  updateStartPoint();
}

void WbTransmissionJoint::setSolidStartPoint(WbSlot *slot) {
  mStartPoint->removeValue();
  mStartPoint->setValue(slot);
  updateStartPoint();
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

void WbTransmissionJoint::updateOdePositionOffset() {
  double newValue = position();
  if (mOdePositionOffset == newValue)
    return;

  mOdePositionOffset = newValue;
}

void WbTransmissionJoint::updateOdePositionOffset2() {
  double newValue = position(2);
  if (mOdePositionOffset == newValue)
    return;

  mOdePositionOffset = newValue;
}
