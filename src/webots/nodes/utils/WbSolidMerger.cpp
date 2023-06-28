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

#include "WbSolidMerger.hpp"

#include "WbDamping.hpp"
#include "WbJoint.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbPhysics.hpp"
#include "WbSolid.hpp"
#include "WbSolidUtilities.hpp"
#include "WbWorld.hpp"

#include <ode/ode.h>
#include <cassert>

WbSolidMerger::WbSolidMerger(WbSolid *solid) :
  mSolid(solid),
  mSpace(NULL),
  mCenterOfMass(0.0, 0.0, 0.0),
  mBodyArtificiallyDisabled(false) {
  assert(mSolid);
  mBody = dBodyCreate(WbOdeContext::instance()->world());
  mOdeMass = new dMass;  // stores inertia relative the merger's CoM and its solid frame
  dMassSetZero(mOdeMass);

  connect(WbOdeContext::instance(), &WbOdeContext::worldDefaultDampingChanged, this, &WbSolidMerger::setOdeDamping);
}

WbSolidMerger::~WbSolidMerger() {
  WbOdeContext::instance()->removeBodyContactJointGroup(mBody);
  dBodyDestroy(mBody);
  mBody = NULL;

  delete mOdeMass;

  if (mSpace)
    dSpaceDestroy(mSpace);
  mSpace = NULL;

  typedef QMap<WbSolid *, dMass *>::const_iterator MCI;
  MCI end = mMergedSolids.constEnd();

  for (MCI it = mMergedSolids.constBegin(); it != end; ++it)
    delete it.value();
}

const QMap<WbSolid *, dMass *> &WbSolidMerger::mergedSolids() const {
  return mMergedSolids;
}

// Collects the mass of rigidly linked solids: solid's inertia matrix is assumed to be computed in relative coordinates, around
// solid center
void WbSolidMerger::appendSolid(WbSolid *solid) {
  dMass *dmass = new dMass;
  memcpy(dmass, solid->odeMass(), sizeof(dMass));
  mMergedSolids.insert(solid, dmass);
}

void WbSolidMerger::removeSolid(WbSolid *solid) {
  subtractSolidMass(solid);
  delete mMergedSolids.value(solid);
  mMergedSolids.remove(solid);
  removeExtraSpace();
  setGeomAndBodyPositions();
  addMassToBody();
}

dSpaceID WbSolidMerger::reservedSpace() {
  reserveSpace();
  return mSpace;
}

void WbSolidMerger::reserveSpace() {
  if (mSpace == NULL) {
    dSpaceID mainSpace = WbOdeContext::instance()->space();
    mSpace = dSimpleSpaceCreate(mainSpace);
    dSpaceSetCleanup(mSpace, 0);
    dGeomID g = mSolid->odeGeom();
    if (g && dGeomGetSpace(g) == mainSpace) {
      dSpaceRemove(mainSpace, g);
      dSpaceAdd(mSpace, g);
    }
  }
}

void WbSolidMerger::addGeomToSpace(dGeomID g) {
  assert(g);
  dSpaceID spaceId = dGeomGetSpace(g);

  if (spaceId)
    dSpaceRemove(spaceId, g);

  reserveSpace();

  dSpaceAdd(mSpace, g);
}

void WbSolidMerger::removeExtraSpace() {
  if (mSpace) {
    const bool singleGeometry = dSpaceGetNumGeoms(mSpace) == 1;
    if (singleGeometry) {
      dGeomID g = dSpaceGetGeom(mSpace, 0);
      dSpaceRemove(mSpace, g);  // Moves the geometry up
      dSpaceAdd(WbOdeContext::instance()->space(), g);
    }

    if (dSpaceGetNumGeoms(mSpace) == 0 || singleGeometry) {
      dSpaceDestroy(mSpace);  // Deletes useless subspace
      mSpace = NULL;
    }
  }
}

void WbSolidMerger::updateCenterOfMass() {
  // Handles the trivial case separately so as to avoid rounding errors
  if (mMergedSolids.size() == 1) {
    mCenterOfMass = mSolid->centerOfMass();
    mAbsoluteCenterOfMass = mSolid->matrix() * mCenterOfMass;
    return;
  }

  // Updates CoM's absolute coordinates
  double mass = 0.0;
  mAbsoluteCenterOfMass.setXyz(0.0, 0.0, 0.0);
  typedef QMap<WbSolid *, dMass *>::const_iterator MCI;
  MCI end = mMergedSolids.constEnd();

  for (MCI it = mMergedSolids.constBegin(); it != end; ++it) {
    const WbSolid *s = it.key();
    const WbVector3 &com = s->matrix() * s->centerOfMass();
    const double m = s->mass();
    mAbsoluteCenterOfMass += m * com;
    mass += m;
  }

  if (mass > 0.0)
    mAbsoluteCenterOfMass /= mass;

  // Computes relative coordinates
  mCenterOfMass = mSolid->matrix().pseudoInversed(mAbsoluteCenterOfMass);
}

// Sets the offset position with respect to solid collector's body for all placeable ODE dGeoms
void WbSolidMerger::setGeomOffsetPositions() {
  typedef QMap<WbSolid *, dMass *>::const_iterator MCI;
  MCI end = mMergedSolids.constEnd();
  for (MCI it = mMergedSolids.constBegin(); it != end; ++it) {
    WbSolid *s = it.key();
    s->updateOdeGeomPosition();
  }
}

// Computes the inverse matrix of the solid collector
WbMatrix4 WbSolidMerger::inverseMatrix() const {
  return mSolid->matrix().pseudoInversed();
}

// Transforms and registers the mass of a collected solid: solid's inertia matrix is assumed to be computed in relative
// coordinates, around solid's frame origin
void WbSolidMerger::transformMass(WbSolid *const solid, const WbMatrix4 &m4) const {
  if (solid->isSolidMerger())
    return;

  dMass *const mass = mMergedSolids.value(solid);
  if (mass->mass <= 0.0)
    return;

  // Computes solid's coordinates with respect to solid collector's frame
  const WbMatrix4 &m = solid->matrix();
  const WbMatrix4 &d = m4 * m;
  const WbVector3 &t = d.translation();  // translation
  dMatrix3 r;                            // rotation
  d.extract3x4Matrix(r);
  // qDebug() << "translate" << t.x() << t.y() << t.z();
  // qDebug() << "rotate" << r[0] << r[1] << r[2] << r[3] << r[4] << r[5] << r[6] << r[7] << r[8] << r[9] << r[10] << r[11];
  // Rotates and translates inertia & CoM

  // Mass transformations
  dMassRotate(mass, r);
  dMassTranslate(mass, t.x(), t.y(), t.z());
}

void WbSolidMerger::transformMass(WbSolid *const solid) const {
  transformMass(solid, inverseMatrix());
}

void WbSolidMerger::transformMasses() const {
  const WbMatrix4 &m4 = inverseMatrix();
  typedef QMap<WbSolid *, dMass *>::const_iterator MI;
  MI end = mMergedSolids.end();
  for (MI it = mMergedSolids.begin(); it != end; ++it)
    transformMass(it.key(), m4);
}

// Substracts a previously transformed mass
void WbSolidMerger::subtractSolidMass(WbSolid *solid) {
  dMass *const m = mMergedSolids.value(solid);
  assert(m);
  const double M = mOdeMass->mass;
  mOdeMass->mass -= m->mass;
  if (mOdeMass->mass <= WbSolid::MASS_ZERO_THRESHOLD)
    dMassSetZero(mOdeMass);
  else {
    const double r = 1.0 / mOdeMass->mass;
    dAddScaledVectors3(mOdeMass->c, mOdeMass->c, m->c, r * M, -r * m->mass);
    WbSolidUtilities::subtractInertiaMatrix(mOdeMass->I, m->I);
  }
}

void WbSolidMerger::mergeMass(WbSolid *const solid, bool subtract) {
  removeExtraSpace();
  if (subtract)
    subtractSolidMass(solid);  // removes previous mass

  dMass *const m = mMergedSolids.value(solid);
  memcpy(m, solid->odeMass(), sizeof(dMass));  // stores the new mass

  transformMass(solid);  // transforms the new solid mass

  if (m->mass > 0.0)
    dMassAdd(mOdeMass, m);

  addMassToBody();

  setGeomAndBodyPositions(false, true);  // reset also joints passing through this solid merger
  setOdeDamping();
}

void WbSolidMerger::updateMasses() {
  typedef QMap<WbSolid *, dMass *>::const_iterator MCI;
  MCI end = mMergedSolids.constEnd();

  for (MCI it = mMergedSolids.constBegin(); it != end; ++it) {
    dMass *const m = it.value();
    const dMass *const sourceMass = it.key()->odeMass();
    assert(m && sourceMass);
    memcpy(m, sourceMass, sizeof(dMass));
  }

  dMassSetZero(mOdeMass);
  mergeMasses();
}

void WbSolidMerger::mergeMasses() {
  // if (mSpace != WbOdeContext::instance()->space() && mSpace != NULL)
  // qDebug() << "Collector" << mSolid->usefulName() << "has" << dSpaceGetNumGeoms(mSpace) << "dGeoms in its own space";

  if (mMergedSolids.size() > 1)
    transformMasses();  // transforms all stored masses

  typedef QMap<WbSolid *, dMass *>::const_iterator MCI;
  MCI end = mMergedSolids.constEnd();

  for (MCI it = mMergedSolids.constBegin(); it != end; ++it) {
    const dMass *const m = it.value();
    assert(m->mass > 0.0);
    dMassAdd(mOdeMass, m);
  }

  addMassToBody();
}

void WbSolidMerger::addMassToBody() {
  updateCenterOfMass();  // TODO: check if it is necessary
  dMass dmass;
  memcpy(&dmass, mOdeMass, sizeof(dMass));

  assert(dmass.mass > 0.0);

  dMassTranslate(&dmass, -mCenterOfMass.x(), -mCenterOfMass.y(), -mCenterOfMass.z());
  // assert(fabs(dmass.c[0]) + fabs(dmass.c[1]) + fabs(dmass.c[2]) < 1e-6); // false when deleting multiple geometries at the
  // same time
  dSetZero(dmass.c, 3);
  dBodySetMass(mBody, &dmass);
}

void WbSolidMerger::setOdeDamping() {
  // fixes stability issues when adding a large force to a body whose inertia matrix is not a multiple if the identity matrix.
  static const double MAX_ANGULAR_SPEED = 10000.0;
  dBodySetMaxAngularSpeed(mBody, MAX_ANGULAR_SPEED);

  double ld = 0.0, ad = 0.0, volume = 0.0;
  typedef QMap<WbSolid *, dMass *>::const_iterator MCI;
  MCI end = mMergedSolids.constEnd();

  // We average collected solids'damping weighted by the volume of their bounding objects
  for (MCI it = mMergedSolids.constBegin(); it != end; ++it) {
    const WbSolid *const s = it.key();
    const WbDamping *damping = s->physics()->damping();
    const double v = s->referenceMass()->mass;
    if (damping) {
      ld += v * damping->linear();
      ad += v * damping->angular();
    }
    volume += v;
  }

  const bool linear = ld > 0.0;
  const bool angular = ad > 0.0;
  const bool damping = linear || angular;
  double l = 0.0, a = 0.0;

  if (linear) {
    ld /= volume;
    // convert damping per second (specified in the Scene Tree) in damping per step (for ODE)
    const double ts = WbWorld::instance()->basicTimeStep() * 0.001;
    l = 1.0 - pow(1.0 - ld, ts);
  }

  if (angular) {
    ad /= volume;
    // convert damping per second (specified in the Scene Tree) in damping per step (for ODE)
    const double ts = WbWorld::instance()->basicTimeStep() * 0.001;
    a = 1.0 - pow(1.0 - ad, ts);
  }

  if (damping) {
    dBodySetDamping(mBody, l, a);
    return;
  }

  // global (world) damping default
  dBodySetDampingDefaults(mBody);
}

void WbSolidMerger::setGeomAndBodyPositions(bool zeroVelocities, bool resetJoints) {
  updateCenterOfMass();
  // Moves ODE body to CoM location
  dBodySetPosition(mBody, mAbsoluteCenterOfMass.x(), mAbsoluteCenterOfMass.y(), mAbsoluteCenterOfMass.z());
  // Rotates ODE body
  WbMatrix4 m44 = mSolid->matrix();
  dMatrix3 m;
  m44.extract3x4Matrix(m);
  dBodySetRotation(mBody, m);
  // Sets the offset position (with respect to the ODE dBody) of every ODE dGeom in the boundingObject
  setGeomOffsetPositions();

  // Wake up
  if (!mBodyArtificiallyDisabled)
    dBodyEnable(mBody);

  // Resets velocities as if the object was moved gently
  if (zeroVelocities) {
    dBodySetLinearVel(mBody, 0.0, 0.0, 0.0);
    dBodySetAngularVel(mBody, 0.0, 0.0, 0.0);
  }

  if (resetJoints) {
    typedef QMap<WbSolid *, dMass *>::const_iterator MCI;
    MCI end = mMergedSolids.constEnd();
    for (MCI it = mMergedSolids.constBegin(); it != end; ++it) {
      WbSolid *const s2 = it.key();
      if (s2->mergerIsSet())
        s2->resetJointsToLinkedSolids();
    }
  }
}

void WbSolidMerger::setupOdeBody() {
  setOdeAutoDisable();
  removeExtraSpace();
  mergeMasses();    // merges and attaches ODE dMasses collected from boundingObjects (and / or inertiaMatrices) to collector's
                    // dBody
  setOdeDamping();  // averages the damping on collected solids
  // Sets the positions and the orientations of the body and its ODE dGeoms (must be done before createJoint())
  setGeomAndBodyPositions();
  const WbVector3 &l = mSolid->linearVelocity();
  dBodySetLinearVel(mBody, l.x(), l.y(), l.z());
  const WbVector3 &a = mSolid->angularVelocity();
  dBodySetAngularVel(mBody, a.x(), a.y(), a.z());
}

void WbSolidMerger::setOdeAutoDisable() {
  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  connect(wi, &WbWorldInfo::physicsDisableChanged, this, &WbSolidMerger::setOdeAutoDisable, Qt::UniqueConnection);

  if (!mBodyArtificiallyDisabled)
    dBodyEnable(mBody);
  const double t = wi->physicsDisableTime();
  if (t > 0.0) {
    dBodySetAutoDisableFlag(mBody, true);
    dBodySetAutoDisableLinearThreshold(mBody, wi->physicsDisableLinearThreshold());
    dBodySetAutoDisableAngularThreshold(mBody, wi->physicsDisableAngularThreshold());
    dBodySetAutoDisableTime(mBody, t);
  } else
    dBodySetAutoDisableFlag(mBody, false);
}

// Sets the merger body into placeable ODE dGeoms
void WbSolidMerger::attachGeomsToBody(dGeomID g) {
  dSpaceID spaceId = WbSolidUtilities::dynamicCastInSpaceID(g);
  if (spaceId) {
    const int n = dSpaceGetNumGeoms(spaceId);
    // we need to store the geoms since dGeomSetBody() changes the way they are sorted in their common space
    dGeomID geoms[n];
    for (int i = 0; i < n; ++i)
      geoms[i] = dSpaceGetGeom(spaceId, i);

    for (int i = 0; i < n; ++i)
      attachGeomsToBody(geoms[i]);

    return;
  }

  if (dGeomGetClass(g) != dPlaneClass)
    dGeomSetBody(g, mBody);
  else
    mSolid->parsingWarn(tr("A Plane defined in 'boundingObject' cannot be used with a Physics node."));
}

bool WbSolidMerger::isSet() const {
  return mSolid->mergerIsSet();
}

void WbSolidMerger::setBodyArtificiallyDisabled(bool disabled) {
  mBodyArtificiallyDisabled = disabled;
  if (disabled) {
    dBodyDisable(mBody);
    dGeomID g = mSolid->odeGeom();
    if (g) {
      WbOdeGeomData *const odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(g));
      if (odeGeomData)
        odeGeomData->setEnableForContactPoint(true);
    }
  } else
    dBodyEnable(mBody);
}
