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

#include "WbSimulationCluster.hpp"

#include "WbCamera.hpp"
#include "WbContactProperties.hpp"
#include "WbDistanceSensor.hpp"
#include "WbFluid.hpp"
#include "WbGeometry.hpp"
#include "WbHingeJoint.hpp"
#include "WbImmersionProperties.hpp"
#include "WbKinematicDifferentialWheels.hpp"
#include "WbLightSensor.hpp"
#include "WbLog.hpp"
#include "WbOdeContact.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbPhysicsPlugin.hpp"
#include "WbRadar.hpp"
#include "WbReceiver.hpp"
#include "WbRobot.hpp"
#include "WbSimulationState.hpp"
#include "WbSolid.hpp"
#include "WbTouchSensor.hpp"
#include "WbTrack.hpp"
#include "WbVacuumGripper.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"

#include <QtCore/QMutex>
#include <QtCore/QMutexLocker>

#include <ode/fluid_dynamics/ode_fluid_dynamics.h>
#include <ode/ode_MT.h>

#include <cassert>
#include <limits>

// The maximum number of contact joints to create. Note that the time to compute a physics timestep with the ODE
// physics engine scales with the cube of the number of joints.
#define MAX_CONTACT_JOINTS 10

// "webots" where each character is replaced by its ascii hexadecimal number.
const long long int WbSimulationCluster::WEBOTS_MAGIC_NUMBER = 0x7765626F7473LL;

QMutex *WbSimulationCluster::cJointCreationMutex = NULL;

WbSimulationCluster::WbSimulationCluster(WbOdeContext *context) : mContext(context), mSwapJointContactBuffer(false) {
  cJointCreationMutex = context->jointGroupCreationMutex();
}

WbSimulationCluster::~WbSimulationCluster() {
  // Is this necessary?
  // Safety precaution: If application closes while dSpaceCollideAndWorldStep is being called
  // the contactJointGroup might be destroyed while being written to
  // This ensures all threads finish writing to it before it is destroyed
  cJointCreationMutex->lock();
  mContext->emptyEnabledBodyContactJointGroups1();
  mContext->emptyEnabledBodyContactJointGroups2();
  dImmersionLinkGroupEmpty(mContext->immersionLinkGroup1());
  dImmersionLinkGroupEmpty(mContext->immersionLinkGroup2());
  cJointCreationMutex->unlock();
}

// do not remove this function it is useful to debug the physics engine
/*
void debugStep(dWorldID world) {
  static int stepNumber = 1;

  char bufferFile[50];
  sprintf(bufferFile, "ode_dump_at_step_%d.txt", stepNumber);
  FILE *file = fopen(bufferFile, "w");
  dWorldExportDIF(world, file, "webots_world ");
  fclose(file);

  stepNumber++;
}
*/

void WbSimulationCluster::handleInitialCollisions() {
  WbPhysicsPlugin *const physicsPlugin = WbPhysicsPlugin::instance();
  if (physicsPlugin)
    physicsPlugin->setCurrentContactJointGroup(physicsPluginContactJointGroup());

  dSpaceCollide(mContext->space(), this, odeNearCallback);
  swapBuffer();
}

void WbSimulationCluster::swapBuffer() {
  mSwapJointContactBuffer = !mSwapJointContactBuffer;

  WbPhysicsPlugin *const physicsPlugin = WbPhysicsPlugin::instance();
  if (physicsPlugin)
    physicsPlugin->setCurrentContactJointGroup(physicsPluginContactJointGroup());
}

void WbSimulationCluster::step() {
  dSpaceUpdateFunction *spaceUpdateFunc = NULL;
  if (WbSolidDevice::hasDirtySensors())
    // update rays position after world step and before space collision detection
    spaceUpdateFunc = &odeSensorRaysUpdate;
  dWorldStepAndSpaceCollide(mContext->space(), this, odeNearCallback, mContext->world(),
                            WbWorld::instance()->basicTimeStep() * 0.001, &dWorldStep, spaceUpdateFunc);

  // every step, we need to save contact points in 'back buffer'
  // here we swap the front buffer with the back buffer.
  swapBuffer();

  handleKinematicsCollisions();
  // debugStep(mContext->world());
  // empty all the joints created by Webots in the previous frame
  dJointGroupEmpty(physicsPluginContactJointGroup());
  if (mSwapJointContactBuffer)
    mContext->emptyEnabledBodyContactJointGroups1();
  else
    mContext->emptyEnabledBodyContactJointGroups2();
  dImmersionLinkGroupEmpty(immersionLinkGroup());

  WbSolidDevice::clearDirtySensorsList();

  // qDebug() << "number of physics plugin contacts 1:" << dJointGroupGetCount(mContext->physicsPluginContactJointGroup1());
  // qDebug() << "number of physics plugin contacts 2:" << dJointGroupGetCount(mContext->physicsPluginContactJointGroup2());
  // qDebug() << "number of contacts 1:" << dJointGroupGetCount(mContext->contactJointGroup1());
  // qDebug() << "number of contacts 2:" << dJointGroupGetCount(mContext->contactJointGroup2());
}

dSpaceID WbSimulationCluster::space() const {
  return mContext->space();
}

dWorldID WbSimulationCluster::world() const {
  return mContext->world();
}

dImmersionLinkGroupID WbSimulationCluster::immersionLinkGroup() const {
  return mSwapJointContactBuffer ? mContext->immersionLinkGroup1() : mContext->immersionLinkGroup2();
}

dJointGroupID WbSimulationCluster::physicsPluginContactJointGroup() const {
  return mSwapJointContactBuffer ? mContext->physicsPluginContactJointGroup1() : mContext->physicsPluginContactJointGroup2();
}

dJointGroupID WbSimulationCluster::bodyContactJointGroup(dBodyID b) {
  return mSwapJointContactBuffer ? mContext->bodyContactJointGroup1(b) : mContext->bodyContactJointGroup2(b);
}

void WbSimulationCluster::handleKinematicsCollisions() {
  // handle differential robots without physics
  foreach (WbKinematicDifferentialWheels *robot, mCollisionedRobots)
    robot->applyKinematicDisplacement();
  mCollisionedRobots.clear();
}

void WbSimulationCluster::collideKinematicRobots(WbKinematicDifferentialWheels *robot, bool collideWithOtherRobot,
                                                 dContact *contact, bool body1) {
  // handle DifferentialWheels robots without physics
  WbVector2 normal2D(contact[0].geom.normal[0], contact[0].geom.normal[1]);
  WbVector3 normal3D(contact[0].geom.normal[0], contact[0].geom.normal[2], contact[0].geom.normal[1]);
  double depth = fabs(contact[0].geom.depth * (normal2D.length() / normal3D.length()));

  // record contact points for graphical and sound rendering
  WbWorld *const w = WbWorld::instance();
  const WbOdeContact odeContact(contact[0].geom, NULL);
  w->appendOdeContact(odeContact);

  // move robots to prevent robots beeing one inside the other
  WbVector2 displacement = WbVector2(normal2D[0] * depth, normal2D[1] * depth);
  if (collideWithOtherRobot)
    displacement /= 2;
  if (body1)
    robot->addKinematicDisplacement(-displacement);
  else
    robot->addKinematicDisplacement(displacement);
}

// don't not remove, this is useful to debug surface paramaters
/*
static void debugSurfaceParameters(const dSurfaceParameters *surf) {
  printf("mu         = %g\n", surf->mu);
  printf("bounce     = %g\n", surf->bounce);
  printf("bounce_vel = %g\n", surf->bounce_vel);
  printf("soft_cfm   = %g\n", surf->soft_cfm);
  printf("soft_erp   = %g\n", surf->soft_erp);
  printf("slip1      = %g\n", surf->slip1);
  printf("slip2      = %g\n\n", surf->slip2);
}
*/

// find the ContactProperties object to use for collisions between two solids
const WbContactProperties *WbSimulationCluster::findContactProperties(const WbSolid *s1, const WbSolid *s2) {
  const WbWorldInfo *const info = WbWorld::instance()->worldInfo();

  const WbContactProperties *contactProperties = NULL;
  // search in list
  const int size = info->contactPropertiesCount();
  for (int i = 0; i < size; ++i) {
    const WbContactProperties *const cp = info->contactProperties(i);
    if ((cp->material1() == s1->contactMaterial() && cp->material2() == s2->contactMaterial()) ||
        (cp->material1() == s2->contactMaterial() && cp->material2() == s1->contactMaterial())) {
      contactProperties = cp;
      break;
    }
  }
  return contactProperties;
}

// fill surface parameters using values in ContactProperties and Track nodes
void WbSimulationCluster::fillSurfaceParameters(const WbContactProperties *cp, const WbSolid *s1, const WbSolid *s2,
                                                const WbGeometry *wg1, const WbGeometry *wg2, dContact *contact) {
  // default values
  int frictionSize = 1;
  int fdsSize = 1;
  double mu[4] = {1.0, 0.0, 0.0, 0.0};
  double rho[3] = {0.0, 0.0, 0.0};
  double bounce = 0.5;
  double bounce_vel = 0.01;
  double fds[4] = {0.0, 0.0, 0.0, 0.0};
  double soft_cfm = 0.001;
  double soft_erp = 0.2;
  bool inversed = false;
  WbVector2 frictionRotation(0, 0);
  if (cp) {
    if (cp->material2() == s1->contactMaterial())
      inversed = true;
    frictionSize = cp->coulombFrictionSize();
    bounce = cp->bounce();
    bounce_vel = cp->bounceVelocity();
    fdsSize = cp->forceDependentSlipSize();
    soft_cfm = cp->softCFM();
    soft_erp = cp->softERP();
    for (int j = 0; (j < frictionSize) && (j < 4); ++j) {
      mu[j] = cp->coulombFriction(j);
      if (mu[j] == -1.0)
        mu[j] = dInfinity;
    }
    const WbVector3 rf = cp->rollingFriction();
    for (int j = 0; j < 3; ++j)
      rho[j] = rf[j] == -1.0 ? dInfinity : rf[j];
    for (int j = 0; (j < fdsSize) && (j < 4); ++j)
      fds[j] = cp->forceDependentSlip(j);
    // get friction direction only if needed
    if ((frictionSize > 1) || (fdsSize > 1))  // asymetric contact
      frictionRotation = cp->frictionRotation();
  }

  WbVector3 globalFdirS1, globalFdirS2;
  if ((frictionSize > 1) || (fdsSize > 1)) {  // asymetric contact
    // Compute first friction direction
    WbVector3 contactNormal(contact->geom.normal[0], contact->geom.normal[1], contact->geom.normal[2]);
    if (inversed) {  // respect the order: material1 -> material2
      globalFdirS1 = wg2->matrix().extracted3x3Matrix() * wg2->computeFrictionDirection(contactNormal);
      globalFdirS2 = wg1->matrix().extracted3x3Matrix() * wg1->computeFrictionDirection(contactNormal);
    } else {
      globalFdirS1 = wg1->matrix().extracted3x3Matrix() * wg1->computeFrictionDirection(contactNormal);
      globalFdirS2 = wg2->matrix().extracted3x3Matrix() * wg2->computeFrictionDirection(contactNormal);
    }

    // check if both geometries support asymmetric friction
    if (globalFdirS1.isNull() || globalFdirS2.isNull()) {
      contact->surface.mode = (bounce != 0.0 ? dContactBounce : 0) | dContactApprox1 | dContactSoftCFM | dContactSoftERP;
      frictionSize = 1;
      fdsSize = 1;
    } else {
      // Apply friction direction rotation if set
      if (!frictionRotation.isNull()) {
        globalFdirS1 =
          WbRotation(contactNormal[0], contactNormal[1], contactNormal[2], -frictionRotation[0]).toMatrix3() * globalFdirS1;
        globalFdirS2 =
          WbRotation(contactNormal[0], contactNormal[1], contactNormal[2], -frictionRotation[1]).toMatrix3() * globalFdirS2;
      }

      // Apply friction direction to the contact joint parameters
      contact->surface.mode =
        dContactFDir1 | (bounce != 0.0 ? dContactBounce : 0) | dContactApprox1 | dContactSoftCFM | dContactSoftERP;
      contact->fdir1[0] = globalFdirS1[0];
      contact->fdir1[1] = globalFdirS1[1];
      contact->fdir1[2] = globalFdirS1[2];
    }
  } else  // fully symmetric contact (slip and friction)
    contact->surface.mode = (bounce != 0.0 ? dContactBounce : 0) | dContactApprox1 | dContactSoftCFM | dContactSoftERP;

  // handle asymmetric friction
  if (frictionSize <= 1)  // symmetric friction
    contact->surface.mu = mu[0];
  else {  // asymmetric friction
    // compute mu1 and mu2 (length of globalFdirS1 and globalFdirS2 is assumed to be 1)
    double vectorAngle = globalFdirS1.angle(globalFdirS2);
    double ratio1 = fabs(cos(vectorAngle));
    double ratio2 = fabs(sin(vectorAngle));
    double mu1, mu2;
    if (frictionSize == 3) {  // second solid has symmetric friction
      mu1 = mu[0] + mu[2];
      mu2 = mu[1] + mu[2];
    } else if (frictionSize == 2) {  // both solids have asymetric friction (with same coefficients)
      mu1 = mu[0] + mu[0] * ratio1 + mu[1] * ratio2;
      mu2 = mu[1] + mu[0] * ratio2 + mu[1] * ratio1;
    } else {  // both solids have asymetric friction (with independent coefficient)
      mu1 = mu[0] + mu[2] * ratio1 + mu[3] * ratio2;
      mu2 = mu[1] + mu[2] * ratio2 + mu[3] * ratio1;
    }

    // apply friction contact joint parameters
    contact->surface.mode = contact->surface.mode | dContactMu2;
    contact->surface.mu = mu1;
    contact->surface.mu2 = mu2;
  }

  // handle rolling friction
  if (rho[0] > 0 || rho[1] > 0 || rho[2] > 0) {
    contact->surface.mode = contact->surface.mode | dContactRolling;
    if (rho[1] > 0 || rho[2] > 0)
      contact->surface.mode = contact->surface.mode | dContactAxisDep;

    contact->surface.rho = rho[0];
    contact->surface.rho2 = rho[1];
    contact->surface.rhoN = rho[2];
  }

  // handle asymetric slip
  if (fdsSize <= 1) {  // symmetric slip
    contact->surface.slip1 = fds[0];
    contact->surface.slip2 = fds[0];
    contact->surface.mode = contact->surface.mode | (fds[0] != 0.0 ? dContactSlip1 | dContactSlip2 : 0);
  } else {  // asymetric slip
    // compute slip1 and slip2 (length of globalFdirS1 and globalFdirS2 is assumed to be 1)
    double vectorAngle = globalFdirS1.angle(globalFdirS2);
    double ratio1 = fabs(cos(vectorAngle));
    double ratio2 = fabs(sin(vectorAngle));
    double slip1 = 0, slip2 = 0;
    if (fdsSize == 3) {  // second solid has symmetric slip
      slip1 = fds[0] + fds[2];
      slip2 = fds[1] + fds[2];
      contact->surface.mode = contact->surface.mode | (slip1 != 0.0 ? dContactSlip1 : 0) | (slip2 != 0.0 ? dContactSlip2 : 0);
    } else if (fdsSize == 2) {  // both solids have asymetric slip (with same coefficients)
      if (fds[1] == 0.0) {      // only Slip1 is activated
        slip1 = fds[0];
        contact->surface.mode = contact->surface.mode | (slip1 != 0.0 ? dContactSlip1 : 0);
      } else if (fds[0] == 0.0) {  // only Slip2 is activated
        slip2 = fds[1];
        contact->surface.mode = contact->surface.mode | (slip2 != 0.0 ? dContactSlip2 : 0);
      } else {
        slip1 = fds[0] + fds[0] * ratio1 + fds[1] * ratio2;
        slip2 = fds[1] + fds[0] * ratio2 + fds[1] * ratio1;
        contact->surface.mode = contact->surface.mode | (slip1 != 0.0 ? dContactSlip1 : 0) | (slip2 != 0.0 ? dContactSlip2 : 0);
      }
    } else {  // both solids have asymetric slip (with independent coefficient)
      slip1 = fds[0] + fds[2] * ratio1 + fds[3] * ratio2;
      slip2 = fds[1] + fds[2] * ratio2 + fds[3] * ratio1;
      contact->surface.mode = contact->surface.mode | (slip1 != 0.0 ? dContactSlip1 : 0) | (slip2 != 0.0 ? dContactSlip2 : 0);
    }

    // apply slip contact joint parameters
    contact->surface.slip1 = slip1;
    contact->surface.slip2 = slip2;
  }

  // define the contact joint parameters
  contact->surface.bounce = bounce;
  contact->surface.bounce_vel = bounce_vel;
  contact->surface.soft_cfm = soft_cfm;
  contact->surface.soft_erp = soft_erp;

  // add tracks surface velocity
  const WbTrack *t = dynamic_cast<const WbTrack *>(s1);
  WbVector3 contactNormal(contact->geom.normal);
  dBodyID trackBody;
  if (t)
    trackBody = dGeomGetBody(wg1->odeGeom());
  else {
    t = dynamic_cast<const WbTrack *>(s2);
    if (t) {
      trackBody = dGeomGetBody(wg2->odeGeom());
      contactNormal *= -1;
    }
  }
  if (t) {
    contactNormal.normalize();
    const WbMatrix4 m(t->matrix());
    WbVector3 bodyZAxis(m(0, 2), m(1, 2), m(2, 2));
    bodyZAxis.normalize();
    double dotProduct = contactNormal.dot(bodyZAxis);
    if (dotProduct < -1)
      dotProduct = -1;
    else if (dotProduct > 1)
      dotProduct = 1;
    const bool invertedSign = dotProduct < 0;
    const WbVector3 &gravityUnit = WbWorld::instance()->worldInfo()->gravityUnitVector();
    double sign;
    if (fabs(gravityUnit[0]) >= fabs(gravityUnit[1]) && fabs(gravityUnit[0]) >= fabs(gravityUnit[2])) {
      // gravity along the X axis
      sign = ((contact->geom.normal[0] < 0) == invertedSign) ? -gravityUnit[0] : gravityUnit[0];
    } else if (fabs(gravityUnit[1]) >= fabs(gravityUnit[2]))  // gravity along the Y axis
      sign = ((contact->geom.normal[1] < 0) == invertedSign) ? -gravityUnit[1] : gravityUnit[1];
    else  // gravity along the Z axis
      sign = ((contact->geom.normal[2] < 0) == invertedSign) ? -gravityUnit[2] : gravityUnit[2];
    // scale based on orientation w.r.t y-axis and set velocity to 0 if track lies on the side
    sign *= (fabs(acos(dotProduct) - M_PI_2) / M_PI_2);

    contact->surface.mode = contact->surface.mode | dContactMotion1;
    contact->surface.motion1 = t->contactSurfaceVelocity() * sign;
    if (trackBody != NULL) {
      contact->surface.mode = contact->surface.mode | dContactFDir1;
      WbVector3 forceDir(m(0, 0), m(1, 0), m(2, 0));
      forceDir.normalize();
      contact->fdir1[0] = forceDir.x() * (invertedSign ? -1 : 1);
      contact->fdir1[1] = forceDir.y() * (invertedSign ? -1 : 1);
      contact->fdir1[2] = forceDir.z() * (invertedSign ? -1 : 1);
    }
  }
}

// fill surface parameters using values in ImmersionProperties nodes
void WbSimulationCluster::fillImmersionSurfaceParameters(const WbSolid *s, const WbImmersionProperties *ip,
                                                         dImmersionSurfaceParameters *surf) {
  surf->mode = ip->immersionSurfaceMode();
  const WbVector3 &dfc = ip->dragForceCoefficients();
  surf->dragForceCoefficients[0] = dfc.x();
  surf->dragForceCoefficients[1] = dfc.y();
  surf->dragForceCoefficients[2] = dfc.z();
  const WbVector3 &dtc = ip->dragTorqueCoefficients();
  surf->dragTorqueCoefficients[0] = dtc.x();
  surf->dragTorqueCoefficients[1] = dtc.y();
  surf->dragTorqueCoefficients[2] = dtc.z();
  surf->viscousResistanceForceCoefficient = ip->viscousResistanceForceCoefficient();
  surf->viscousResistanceTorqueCoefficient = ip->viscousResistanceTorqueCoefficient();
  surf->maxLinearVel = std::numeric_limits<double>::max();
  surf->maxAngularVel = 1000.0;
}

static bool needCollisionDetection(WbSolid *solid, bool isOtherRayGeom) {
  switch (solid->nodeType()) {
    case WB_NODE_TOUCH_SENSOR:
      if (isOtherRayGeom)
        return false;
    case WB_NODE_CAMERA:
    case WB_NODE_DISTANCE_SENSOR:
    case WB_NODE_LIGHT_SENSOR:
    case WB_NODE_RADAR:
    case WB_NODE_RECEIVER:
    case WB_NODE_ROBOT:
      return true;
  }

  return false;
}

void WbSimulationCluster::appendCollisionedRobot(WbKinematicDifferentialWheels *robot) {
  if (WbOdeContext::instance()->numberOfThreads() > 1) {
    QMutexLocker<QMutex> lock(&mCollisionedRobotsMutex);
    mCollisionedRobots.append(robot);
  } else
    mCollisionedRobots.append(robot);
}

void WbSimulationCluster::handleCollisionIfSpace(void *data, dGeomID o1, dGeomID o2) {
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
    // colliding a mContext->space() with something
    dSpaceCollide2(o1, o2, data, odeNearCallback);
    // collide all geoms internal to the mContext->space()(s)
    if (dGeomIsSpace(o1))
      dSpaceCollide(reinterpret_cast<dSpaceID>(o1), data, odeNearCallback);
    if (dGeomIsSpace(o2))
      dSpaceCollide(reinterpret_cast<dSpaceID>(o2), data, odeNearCallback);
  }
}

void WbSimulationCluster::warnMoreContactPointsThanContactJoints(const QString &material1, const QString &material2, int max,
                                                                 int n) {
  static QMutex mutex;
  QMutexLocker<QMutex> lock(&mutex);
  static double lastWarningTime = -INFINITY;
  const double currentSimulationTime = WbSimulationState::instance()->time();
  if (currentSimulationTime > lastWarningTime + 1000.0) {
    WbLog::warning(QObject::tr("Contact joints between materials '%1' and '%2' will only be created for the %3 deepest contact "
                               "points instead of all the %4 contact points.")
                     .arg(material1)
                     .arg(material2)
                     .arg(max)
                     .arg(n),
                   false, WbLog::ODE);
    lastWarningTime = currentSimulationTime;
  }
}

void WbSimulationCluster::odeNearCallback(void *data, dGeomID o1, dGeomID o2) {
  // retrieve data
  WbOdeGeomData *const odeGeomData1 = static_cast<WbOdeGeomData *>(dGeomGetData(o1));
  WbOdeGeomData *const odeGeomData2 = static_cast<WbOdeGeomData *>(dGeomGetData(o2));
  WbSolid *s1 = NULL, *s2 = NULL;
  WbGeometry *wg1 = NULL, *wg2 = NULL;

  // colliding two non-space geoms, so generate contact points between o1 and o2
  WbPhysicsPlugin *const physicsPlugin = WbPhysicsPlugin::instance();
  if (physicsPlugin) {
    // Were these dGeoms created by Webots?
    const bool webotsGeom1 = odeGeomData1 && odeGeomData1->magicNumber() == WEBOTS_MAGIC_NUMBER;
    const bool webotsGeom2 = odeGeomData2 && odeGeomData2->magicNumber() == WEBOTS_MAGIC_NUMBER;

    const int pluginContacts = physicsPlugin->collide(o1, o2);

    if (pluginContacts == 0)
      handleCollisionIfSpace(data, o1, o2);
    if (pluginContacts == 2) {  // Webots will change the boundingObject color to notify collision
      if (webotsGeom1) {
        s1 = odeGeomData1->solid();
        wg1 = odeGeomData1->geometry();
      }
      if (webotsGeom2) {
        s2 = odeGeomData2->solid();
        wg2 = odeGeomData2->geometry();
      }
      if (s1)
        wg1->setColliding();
      if (s2)
        wg2->setColliding();
      return;
    } else if (pluginContacts == 1 || !webotsGeom1 || !webotsGeom2)  // Webots won't attempt to manipulate user-defined dGeoms
      return;
  } else
    handleCollisionIfSpace(data, o1, o2);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
    return;

  // yvan: we are never interested in the collision between 2 ray geoms
  // (rays geoms can be either distance sensor, emitter-receiver or light sensor rays)
  const bool isRayGeom1 = dGeomGetClass(o1) == dRayClass;
  const bool isRayGeom2 = dGeomGetClass(o2) == dRayClass;
  if (isRayGeom1 && isRayGeom2)
    return;

  // don't reach this point if handled by physics plugin
  assert(odeGeomData1 && odeGeomData2 && odeGeomData1->magicNumber() == WEBOTS_MAGIC_NUMBER &&
         odeGeomData2->magicNumber() == WEBOTS_MAGIC_NUMBER);
  s1 = odeGeomData1->solid();
  wg1 = odeGeomData1->geometry();
  s2 = odeGeomData2->solid();
  wg2 = odeGeomData2->geometry();

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  WbSimulationCluster *const cl = static_cast<WbSimulationCluster *>(data);

  if (!s1 || !s2) {
    if (isRayGeom1 || isRayGeom2)
      return;

    if ((s1 && !b1) || (s2 && !b2))
      return;

    WbFluid *const f1 = odeGeomData1->fluid();
    WbFluid *const f2 = odeGeomData2->fluid();

    if ((s1 && f2) || (s2 && f1)) {
      WbSolid *const solid = s1 ? s1 : s2;
      WbFluid *const fluid = f1 ? f1 : f2;

      const WbImmersionProperties *immersionProperties = NULL;
      const WbMFNode &imp = solid->immersionProperties();
      const int size = imp.size();
      for (int i = 0; i < size; ++i) {
        const WbImmersionProperties *const ip = static_cast<WbImmersionProperties *>(imp.item(i));
        if (ip->fluidName() == fluid->name())
          immersionProperties = ip;
      }

      if (immersionProperties == NULL)
        return;

      dImmersion immersion;
      dImmersionOutlineID io = dImmersionOutlineCreate();
      immersion.geom.outline = io;
      const int n = s1 ? dImmerse(o1, o2, 1, &immersion.geom) : dImmerse(o2, o1, 1, &immersion.geom);
      if (n == 0) {
        dImmersionOutlineDestroy(io);
        return;
      }

      assert((s1 && (o1 == immersion.geom.g1 && o2 == immersion.geom.g2)) ||
             (s2 && (o2 == immersion.geom.g1 && o1 == immersion.geom.g2)));

      dBodyID b = b1 ? b1 : b2;
      dFluidID f = fluid->odeFluid();

      wg1->setColliding();
      wg2->setColliding();

      // using immersion properties specified in Solid
      cl->fillImmersionSurfaceParameters(solid, immersionProperties, &immersion.surface);

      // add these links to the simulation
      cJointCreationMutex->lock();  // TODO: check if this mutex is useful and possibly rename it
      dImmersionLinkID iml = dImmersionLinkCreate(dBodyGetWorld(b), cl->immersionLinkGroup(), &immersion);
      dImmersionLinkAttach(iml, b, f);
      cJointCreationMutex->unlock();

      // record immersion info for immersion outline rendering
      WbWorld *const world = WbWorld::instance();
      world->appendOdeImmersionGeom(immersion.geom);
    }

    return;
  }

  assert(s1 && s2);
  // No collision detection onboard the same robot unless Robot.selfCollision is true
  const WbRobot *const r1 = s1->robot();
  const WbRobot *const r2 = s2->robot();
  if (r1 && r1 == r2 && !r1->selfCollision())
    return;

  // exit if two part of static environment collide
  // but continue for a collision that involves a sensor
  if (b1 == NULL && b2 == NULL && !needCollisionDetection(s1, isRayGeom2) && !needCollisionDetection(s2, isRayGeom1))
    return;

  // Exit without doing anything if the two bodies are connected by a joint
  // or if a body is attached to the static environment with a joint
  // special case: collisions betweeen rays and bodies attached to the
  // static environment doesn't have to be ignored
  if ((b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) ||
      (!isRayGeom1 && !isRayGeom2 &&
       ((b1 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) ||
        (b2 && dAreConnectedExcluding(b2, b1, dJointTypeContact)))))
    return;

  // ignore collision if bodies are connected by a chain of HingeJoint (or derivatives) sharing a same anchor
  if (b1 && b2) {
    WbNode *n;  // climb up the chain from the lowest level
    dBodyID b;  // stop when we reach this body
    if (s1->level() > s2->level()) {
      n = dynamic_cast<WbNode *>(s1);
      b = b2;
    } else {
      n = dynamic_cast<WbNode *>(s2);
      b = b1;
    }

    WbVector3 firstAnchor(NAN, NAN, NAN);
    while (n && n->level() >= 1) {
      const WbSolid *s = dynamic_cast<WbSolid *>(n);
      if (s && s->body() == b && !firstAnchor.isNan())
        return;  // one intermediary joint is mandatory to ignore collisions at this level, otherwise it's taken care elsewhere

      const WbHingeJoint *joint = dynamic_cast<WbHingeJoint *>(n);
      if (joint) {
        if (firstAnchor.isNan())
          firstAnchor = joint->anchor();
        else if (!firstAnchor.almostEquals(joint->anchor()))
          break;
      }
      n = n->parentNode();
    }
  }
  const WbContactProperties *contactProperties = cl->findContactProperties(s1, s2);
  const int maxContactJoints = contactProperties ? contactProperties->maxContactJoints() : 10;

  thread_local QVector<dContact> contactVector;
  int maxContactPoints = std::max<size_t>(100, contactVector.capacity());

  int n;
  dContact *contact;
  while (true) {
    contactVector.reserve(maxContactPoints);
    contact = contactVector.data();
    n = dCollide(o1, o2, maxContactPoints, &contact[0].geom, sizeof(dContact));
    if (n < maxContactPoints)
      break;
    else
      maxContactPoints *= 10;
  }

  if (n == 0)
    return;

  bool areContactPointsSorted = false;
  if (n > maxContactJoints) {
    WbSimulationCluster::warnMoreContactPointsThanContactJoints(s1->contactMaterial(), s2->contactMaterial(), maxContactJoints,
                                                                n);
    std::nth_element(contact, contact + maxContactJoints, contact + n,
                     [](const dContact &c1, const dContact &c2) { return (c1.geom.depth > c2.geom.depth); });
    areContactPointsSorted = true;
    n = maxContactJoints;
  }

  WbTouchSensor *const ts1 = dynamic_cast<WbTouchSensor *>(s1);
  if (ts1 && !isRayGeom2)
    ts1->setTouching(true);

  WbTouchSensor *const ts2 = dynamic_cast<WbTouchSensor *>(s2);
  if (ts2 && !isRayGeom1)
    ts2->setTouching(true);

  WbVacuumGripper *vg = dynamic_cast<WbVacuumGripper *>(s1);
  WbSolid *vgSolid;
  if (vg)
    vgSolid = isRayGeom2 ? NULL : s2;
  else {
    vg = dynamic_cast<WbVacuumGripper *>(s2);
    vgSolid = isRayGeom1 ? NULL : s1;
  }
  if (vg && vgSolid && vg->isWaitingForConnection() && n >= vg->contactPoints()) {
    if (!areContactPointsSorted)
      std::nth_element(contact, contact, contact + n,
                       [](const dContact &c1, const dContact &c2) { return (c1.geom.depth > c2.geom.depth); });
    vg->addCollidedSolid(vgSolid, contact[0].geom.depth);
  }

  const WbPhysics *const p1 = s1->physics();
  const WbPhysics *const p2 = s2->physics();

  // kinematic mode
  if (!p1 || !p2) {
    WbRobot *const robot1 = dynamic_cast<WbRobot *>(s1);
    WbRobot *const robot2 = dynamic_cast<WbRobot *>(s2);
    // Ensure that the deepest contact point is first for calls to collideKinematicRobots() below.
    if (!areContactPointsSorted)
      std::nth_element(contact, contact, contact + n,
                       [](const dContact &c1, const dContact &c2) { return (c1.geom.depth > c2.geom.depth); });
    if (robot1 && !p1 && !isRayGeom2 && robot1->kinematicDifferentialWheels()) {
      wg1->setColliding();
      cl->appendCollisionedRobot(robot1->kinematicDifferentialWheels());
      if (robot2 && !p2 && robot2->kinematicDifferentialWheels()) {
        wg2->setColliding();
        cl->appendCollisionedRobot(robot2->kinematicDifferentialWheels());
        collideKinematicRobots(robot1->kinematicDifferentialWheels(), true, contact, true);
        collideKinematicRobots(robot2->kinematicDifferentialWheels(), true, contact, false);
      } else
        collideKinematicRobots(robot1->kinematicDifferentialWheels(), false, contact, true);
      return;
    }
    if (robot2 && !p2 && !isRayGeom1 && robot2->kinematicDifferentialWheels()) {
      wg2->setColliding();
      cl->appendCollisionedRobot(robot2->kinematicDifferentialWheels());
      collideKinematicRobots(robot2->kinematicDifferentialWheels(), false, contact, false);
      return;
    }
  }

  // Handles the case of ray geometry against a non-ray geometry, i.e. at least one body is null
  if (isRayGeom1) {
    WbDistanceSensor *const ds = dynamic_cast<WbDistanceSensor *>(s1);
    if (ds) {
      int ix = 0;  // index of the closest contact
      for (int i = 1; i < n; ++i)
        if (contact[i].geom.depth < contact[ix].geom.depth)
          ix = i;
      // Luc : contact[0].geom.g1 and contact[0].geom.g2 may not coincide with o1 and o2 in an oddly defined dCollide
      // call-back function of ODE. Should we be worried?
      assert(o1 == contact[ix].geom.g1 && o2 == contact[ix].geom.g2);
      ds->rayCollisionCallback(odeGeomData2->geometry(), o1, &contact[ix].geom);
      return;
    }

    WbCamera *const camera = dynamic_cast<WbCamera *>(s1);
    if (camera) {
      camera->rayCollisionCallback(o1, s2, contact[0].geom.depth);
      return;
    }

    WbRadar *const radar = dynamic_cast<WbRadar *>(s1);
    if (radar) {
      radar->rayCollisionCallback(o1, s2, contact[0].geom.depth);
      return;
    }

    WbLightSensor *const ls = dynamic_cast<WbLightSensor *>(s1);
    if (ls) {
      ls->rayCollisionCallback(o1);
      return;
    }

    WbReceiver *const r = dynamic_cast<WbReceiver *>(s1);
    if (r) {
      r->rayCollisionCallback(o1, s2);
      return;
    }
  } else if (isRayGeom2) {
    WbDistanceSensor *const ds = dynamic_cast<WbDistanceSensor *>(s2);
    if (ds) {
      assert(o1 == contact[0].geom.g1 && o2 == contact[0].geom.g2);
      ds->rayCollisionCallback(odeGeomData1->geometry(), o2, &contact[0].geom);
      return;
    }

    WbCamera *const camera = dynamic_cast<WbCamera *>(s2);
    if (camera) {
      camera->rayCollisionCallback(o2, s1, contact[0].geom.depth);
      return;
    }

    WbRadar *const radar = dynamic_cast<WbRadar *>(s2);
    if (radar) {
      radar->rayCollisionCallback(o2, s1, contact[0].geom.depth);
      return;
    }

    WbLightSensor *const ls = dynamic_cast<WbLightSensor *>(s2);
    if (ls) {
      ls->rayCollisionCallback(o2);
      return;
    }

    WbReceiver *const r = dynamic_cast<WbReceiver *>(s2);
    if (r) {
      r->rayCollisionCallback(o2, s1);
      return;
    }
  }

  if (b1 == NULL && b2 == NULL)
    return;

  // A physical collision occurs between bodies from different robots

  wg1->setColliding();
  wg2->setColliding();

  // if the geometry of one colliding object has changed this step, we need to enable the other body
  // (otherwise if the changed geometry is kinematic the other body will not move)
  if (odeGeomData1->lastChangeTime() > 0.0) {
    if (odeGeomData1->lastChangeTime() >= WbSimulationState::instance()->time()) {
      if (b2)
        dBodyEnable(b2);
    } else
      odeGeomData1->setLastChangeTime(0.0);
  }

  if (odeGeomData2->lastChangeTime() > 0.0) {
    if (odeGeomData2->lastChangeTime() >= WbSimulationState::instance()->time()) {
      if (b1)
        dBodyEnable(b1);
    } else
      odeGeomData2->setLastChangeTime(0.0);
  }

  if (b1 && odeGeomData1->enableForContactPoint())
    dBodyEnable(b1);

  if (b2 && odeGeomData2->enableForContactPoint())
    dBodyEnable(b2);
  // no need to create contact joints for contacts between disable bodies and kinematic solids
  const bool b1Disabled = b1 && !dBodyIsEnabled(b1);
  const bool b2Disabled = b2 && !dBodyIsEnabled(b2);
  if ((s1->isKinematic() && b2Disabled) || (s2->isKinematic() && b1Disabled) || (b1Disabled && b2Disabled))
    return;

  // From now on, solids have a non-null body
  // So, we can fill the contact surface parameters.
  //------------------------------------------------

  for (int i = 0; i < n; ++i) {
    assert(o1 == contact[i].geom.g1 && o2 == contact[i].geom.g2);

    // using contact properties for the materials associated with two solids
    cl->fillSurfaceParameters(contactProperties, s1, s2, wg1, wg2, &contact[i]);

    // add these joints to the simulation
    dWorldID jointWorld = b1 ? dBodyGetWorld(b1) : dBodyGetWorld(b2);

    cJointCreationMutex->lock();
    dJointGroupID group = 0;
    if (b1 == NULL || (s1->isKinematic() && !s2->isKinematic()))
      group = cl->bodyContactJointGroup(b2);
    else
      group = cl->bodyContactJointGroup(b1);
    dJointID contactJoint = dJointCreateContact(jointWorld, group, &contact[i]);
    dJointAttach(contactJoint, b1, b2);
    cJointCreationMutex->unlock();

    // record contact points for graphical and sound rendering
    WbWorld *const world = WbWorld::instance();
    const WbOdeContact odeContact(contact[i].geom, contactProperties);
    world->appendOdeContact(odeContact);
  }
}

void WbSimulationCluster::odeSensorRaysUpdate(int threadID) {
  QList<dSpaceID> spaceIDList;
  int count = dThreadGetSpacesCount(threadID);
  for (int i = 0; i < count; ++i)
    spaceIDList.append(dThreadGetSpaceID(threadID, i));
  WbSolidDevice::updateDirtySensors(spaceIDList);
}
