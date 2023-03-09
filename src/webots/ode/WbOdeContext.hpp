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

#ifndef WB_ODE_CONTEXT_HPP
#define WB_ODE_CONTEXT_HPP

#include <ode/fluid_dynamics/common_fluid_dynamics.h>
#include <ode/ode.h>

#include <QtCore/QHash>
#include <QtCore/QMutex>
#include <QtCore/QObject>

class WbOdeContext : public QObject {
  Q_OBJECT

public:
  // create ODE objects
  WbOdeContext();

  // destroy ODE objects
  ~WbOdeContext();

  static WbOdeContext *instance() { return cOdeContext; }

  // apply to the world
  void setGravity(double x, double y, double z);
  void setCfm(double cfm);
  void setErp(double erp);
  void setDamping(double linear, double angular);
  void setPhysicsDisableTime(double time);
  void setNumberOfThreads(int n);

  // getters
  dWorldID world() const { return mWorld; }
  dSpaceID space() const { return mSpace; }
  int numberOfThreads() const { return mNumberOfThreads; }

  QMutex *jointGroupCreationMutex() { return mJointGroupCreationMutex; }
  dImmersionLinkGroupID immersionLinkGroup1() const { return mImmersionLinkGroup1; }
  dImmersionLinkGroupID immersionLinkGroup2() const { return mImmersionLinkGroup2; }
  dJointGroupID physicsPluginContactJointGroup1() const { return mPhysicsPluginContactJointGroup1; }
  dJointGroupID physicsPluginContactJointGroup2() const { return mPhysicsPluginContactJointGroup2; }

  void emptyEnabledBodyContactJointGroups1();
  void emptyEnabledBodyContactJointGroups2();
  dJointGroupID bodyContactJointGroup1(dBodyID b);
  dJointGroupID bodyContactJointGroup2(dBodyID b);
  void removeBodyContactJointGroup(dBodyID b);

signals:
  void worldDefaultDampingChanged();

private:
  dWorldID mWorld;
  dSpaceID mSpace;

  QMutex *mJointGroupCreationMutex;
  QHash<dBodyID, dJointGroupID> mBodyContactJointGroupList1, mBodyContactJointGroupList2;
  dImmersionLinkGroupID mImmersionLinkGroup1, mImmersionLinkGroup2;
  dJointGroupID mPhysicsPluginContactJointGroup1, mPhysicsPluginContactJointGroup2;

  int mNumberOfThreads;
  static WbOdeContext *cOdeContext;
};

#endif
