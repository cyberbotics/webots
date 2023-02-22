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

#include "WbOdeContext.hpp"

#include <ode/fluid_dynamics/objects_fluid_dynamics.h>
#include <cassert>

WbOdeContext *WbOdeContext::cOdeContext = NULL;

WbOdeContext::WbOdeContext() : QObject() {
  dInitODE();
  mWorld = dWorldCreate();
  dWorldSetLinearDampingThreshold(mWorld, 0.0);
  dWorldSetAngularDampingThreshold(mWorld, 0.0);

  mSpace = dSimpleSpaceCreate(NULL);

  mJointGroupCreationMutex = new QMutex();
  mImmersionLinkGroup1 = dImmersionLinkGroupCreate();
  mImmersionLinkGroup2 = dImmersionLinkGroupCreate();
  mPhysicsPluginContactJointGroup1 = dJointGroupCreate(0);
  mPhysicsPluginContactJointGroup2 = dJointGroupCreate(0);
  mBodyContactJointGroupList1.clear();
  mBodyContactJointGroupList2.clear();

  mNumberOfThreads = -1;
  cOdeContext = this;
}

WbOdeContext::~WbOdeContext() {
  QList<dJointGroupID> groups(mBodyContactJointGroupList1.values());
  for (int i = 0; i < groups.size(); ++i) {
    dJointGroupDestroy(groups[i]);
  }
  groups = mBodyContactJointGroupList2.values();
  for (int i = 0; i < groups.size(); ++i) {
    dJointGroupDestroy(groups[i]);
  }
  mBodyContactJointGroupList1.clear();
  mBodyContactJointGroupList2.clear();

  dJointGroupDestroy(mPhysicsPluginContactJointGroup1);
  dJointGroupDestroy(mPhysicsPluginContactJointGroup2);
  dSpaceDestroy(mSpace);
  dWorldDestroy(mWorld);

  delete mJointGroupCreationMutex;
  mJointGroupCreationMutex = NULL;

  dCloseODE();
  cOdeContext = NULL;
}

void WbOdeContext::setNumberOfThreads(int n) {
  if (mNumberOfThreads == n)
    return;
  mNumberOfThreads = n;
  // dToggleODE_MT(0) will cause ODE to work non-threaded
  // dToggleODE_MT(1) will cause ODE to work on multi-thread with a single thread, which is inefficient
  // This is why when we set the number of threads to 1, we want to revert to the non-threaded mode
  if (mNumberOfThreads == 1)
    dToggleODE_MT(0);
  else
    dToggleODE_MT(n);
}

void WbOdeContext::setGravity(double x, double y, double z) {
  dWorldSetGravity(mWorld, x, y, z);
}

void WbOdeContext::setErp(double erp) {
  dWorldSetERP(mWorld, erp);
}

void WbOdeContext::setCfm(double cfm) {
  dWorldSetCFM(mWorld, cfm);
}

void WbOdeContext::setDamping(double linear, double angular) {
  dWorldSetDamping(mWorld, linear, angular);

  emit worldDefaultDampingChanged();
}

void WbOdeContext::setPhysicsDisableTime(double time) {
  dWorldSetAutoDisableTime(mWorld, time);
  dWorldSetAutoDisableSteps(mWorld, 0);  // focus is on time left (ms) before sleep
}

void WbOdeContext::emptyEnabledBodyContactJointGroups1() {
  // mutex is locked in WbSimulationCluster before calling this function
  QHashIterator<dBodyID, dJointGroupID> it(mBodyContactJointGroupList1);
  while (it.hasNext()) {
    it.next();
    if (dBodyIsEnabled(it.key()))
      dJointGroupEmpty(it.value());
  }
}

void WbOdeContext::emptyEnabledBodyContactJointGroups2() {
  // mutex is locked in WbSimulationCluster before calling this function
  QHashIterator<dBodyID, dJointGroupID> it(mBodyContactJointGroupList2);
  while (it.hasNext()) {
    it.next();
    if (dBodyIsEnabled(it.key()))
      dJointGroupEmpty(it.value());
  }
}

dJointGroupID WbOdeContext::bodyContactJointGroup1(dBodyID b) {
  // mutex is locked in WbSimulationCluster before calling this function
  QHash<dBodyID, dJointGroupID>::const_iterator it = mBodyContactJointGroupList1.find(b);
  if (it == mBodyContactJointGroupList1.end()) {
    // body not found
    dJointGroupID group = dJointGroupCreate(0);
    mBodyContactJointGroupList1.insert(b, group);
    return group;
  }

  return it.value();
}

dJointGroupID WbOdeContext::bodyContactJointGroup2(dBodyID b) {
  // mutex is locked in WbSimulationCluster before calling this function
  QHash<dBodyID, dJointGroupID>::const_iterator it = mBodyContactJointGroupList2.find(b);
  if (it == mBodyContactJointGroupList2.end()) {
    // body not found
    dJointGroupID group = dJointGroupCreate(0);
    mBodyContactJointGroupList2.insert(b, group);
    return group;
  }

  return it.value();
}

void WbOdeContext::removeBodyContactJointGroup(dBodyID b) {
  mJointGroupCreationMutex->lock();
  if (mBodyContactJointGroupList1.contains(b)) {
    dJointGroupDestroy(mBodyContactJointGroupList1.value(b));
    mBodyContactJointGroupList1.remove(b);
  }
  if (mBodyContactJointGroupList2.contains(b)) {
    dJointGroupDestroy(mBodyContactJointGroupList2.value(b));
    mBodyContactJointGroupList2.remove(b);
  }
  mJointGroupCreationMutex->unlock();
}
