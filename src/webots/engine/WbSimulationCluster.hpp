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

#ifndef WB_SIMULATION_CLUSTER_HPP
#define WB_SIMULATION_CLUSTER_HPP

//
// Description: single-threaded simulation
//

#include <ode/fluid_dynamics/ode_fluid_dynamics.h>
#include <ode/ode.h>
#include <QtCore/QList>
#include <QtCore/QMutex>

class WbContactProperties;
class WbImmersionProperties;
class WbOdeContext;
class WbKinematicDifferentialWheels;
class WbSolid;
class WbGeometry;
class QMutex;

class WbSimulationCluster {
public:
  // create/destroy cluster thread and ODE objects
  explicit WbSimulationCluster(WbOdeContext *context);
  virtual ~WbSimulationCluster();

  // collision detection and simulation steps
  // for this cluster's world and space
  void step();

  // ODE objects
  dWorldID world() const;
  dSpaceID space() const;
  dJointGroupID bodyContactJointGroup(dBodyID b);
  dImmersionLinkGroupID immersionLinkGroup() const;
  dJointGroupID physicsPluginContactJointGroup() const;

  void handleInitialCollisions();  // used to synchronize contact point representations and current positions

private:
  WbOdeContext *mContext;
  static QMutex *cJointCreationMutex;

  QMutex mCollisionedRobotsMutex;
  QList<WbKinematicDifferentialWheels *> mCollisionedRobots;
  void appendCollisionedRobot(WbKinematicDifferentialWheels *robot);
  void handleKinematicsCollisions();
  void swapBuffer();
  static void handleCollisionIfSpace(void *data, dGeomID o1, dGeomID o2);
  static const WbContactProperties *findContactProperties(const WbSolid *s1, const WbSolid *s2);
  static void fillSurfaceParameters(const WbContactProperties *cp, const WbSolid *s1, const WbSolid *s2, const WbGeometry *wg1,
                                    const WbGeometry *wg2, dContact *contact);
  static void fillImmersionSurfaceParameters(const WbSolid *s, const WbImmersionProperties *ip,
                                             dImmersionSurfaceParameters *surf);
  static void odeNearCallback(void *data, dGeomID o1, dGeomID o2);
  static void collideKinematicRobots(WbKinematicDifferentialWheels *robot, bool collideWithOtherRobot, dContact *contact,
                                     bool body1);
  static void odeSensorRaysUpdate(int threadID);
  static const long long int WEBOTS_MAGIC_NUMBER;
  bool mSwapJointContactBuffer;
  static void warnMoreContactPointsThanContactJoints(const QString &material1, const QString &material2, int max, int n);
};

#endif
