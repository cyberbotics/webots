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

#include "WbPhysics.hpp"

#include "WbDamping.hpp"
#include "WbFieldChecker.hpp"
#include "WbMFVector3.hpp"
#include "WbSFNode.hpp"

#include <ode/ode.h>

#include <cassert>

void WbPhysics::init() {
  mDensity = findSFDouble("density");
  mMass = findSFDouble("mass");
  mCenterOfMass = findMFVector3("centerOfMass");
  mInertiaMatrix = findMFVector3("inertiaMatrix");
  mDamping = findSFNode("damping");

  mSkipUpdate = false;
  mHasAvalidInertiaMatrix = true;
  mMode = INVALID;
}

WbPhysics::WbPhysics(WbTokenizer *tokenizer) : WbBaseNode("Physics", tokenizer) {
  init();
}

WbPhysics::WbPhysics(const WbPhysics &other) : WbBaseNode(other) {
  init();
}

WbPhysics::WbPhysics(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbPhysics::~WbPhysics() {
}

void WbPhysics::preFinalize() {
  WbBaseNode::preFinalize();

  if (damping())
    damping()->preFinalize();

  checkDensity();
  checkMass();
  checkMassAndDensity();

  int size = mInertiaMatrix->size();
  if (size == 1) {
    mInertiaMatrix->insertDefaultItem(1);
    parsingWarn(tr("'inertiaMatrix' must have exactly 0 or 2 lines. Second line inserted."));
  } else if (size > 2) {
    do {
      mInertiaMatrix->removeItem(2);
    } while (mInertiaMatrix->size() > 2);
    parsingWarn(tr("'inertiaMatrix' must have exactly 0 or 2 lines. Extra lines skipped."));
  }

  size = mCenterOfMass->size();
  if (size > 1) {
    do {
      mCenterOfMass->removeItem(1);
    } while (mCenterOfMass->size() > 1);
    parsingWarn(tr("'centerOfMass' must have exactly 0 or 1 line. Extra lines skipped."));
  }

  checkInertiaMatrix(true);
  updateMode();
  updateDamping();
}

void WbPhysics::postFinalize() {
  WbBaseNode::postFinalize();

  if (damping())
    damping()->postFinalize();

  connect(mDensity, &WbSFDouble::changed, this, &WbPhysics::updateDensity);
  connect(mMass, &WbSFDouble::changed, this, &WbPhysics::updateMass);
  connect(mCenterOfMass, &WbMFVector3::changed, this, &WbPhysics::updateCenterOfMass);
  connect(mInertiaMatrix, &WbMFVector3::changed, this, &WbPhysics::updateInertiaMatrix);
  connect(mDamping, &WbSFNode::changed, this, &WbPhysics::updateDamping);
}

void WbPhysics::reset(const QString &id) {
  WbBaseNode::reset(id);

  WbNode *const d = mDamping->value();
  if (d)
    d->reset(id);
}

void WbPhysics::checkMass() {
  if (mDensity->value() <= 0.0)
    WbFieldChecker::resetDoubleIfNonPositive(this, mMass, 1);
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mMass, -1, -1);
}

void WbPhysics::checkDensity() {
  if (mMass->value() <= 0.0)
    WbFieldChecker::resetDoubleIfNonPositive(this, mDensity, 1000);
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mDensity, 1000, -1);
}

void WbPhysics::checkMassAndDensity() const {
  const double m = mMass->value();
  const double d = mDensity->value();
  if (m == -1.0 && d == -1.0) {
    parsingWarn(tr("Either the 'mass' or the 'density' must be specified."));
    return;
  }

  if (m > 0.0 && d > 0.0) {
    parsingWarn(tr("Both 'density' and 'mass' specified: the 'density' will be ignored."));
    return;
  }
}

void WbPhysics::checkCenterOfMass() const {
  const bool validCom = mCenterOfMass->size() == 1;

  const int size = mInertiaMatrix->size();
  if (!validCom && size == 2)
    parsingInfo(tr("The center of mass must also be specified when using a custom inertia matrix."));
  // else if (validCom && size == 0)
  //  parsingInfo(tr("The inertia matrix must also be specified when using a center of mass."));
}

void WbPhysics::checkInertiaMatrix(bool showInfo) {
  mHasAvalidInertiaMatrix = true;
  const int size = mInertiaMatrix->size();

  if (size != 2) {
    mHasAvalidInertiaMatrix = false;
    // if (showInfo && size == 0 && mCenterOfMass->size() == 1)
    //  parsingWarn(tr("Remove 'centerOfMass' to use bounding object based inertial properties, or insert a new
    //  'inertiaMatrix'."));
    return;
  }

  if (showInfo && mCenterOfMass->size() == 0)
    parsingInfo(tr("You must also specify the 'centerOfMass' when specifying the 'inertiaMatrix'"));

  if (showInfo && mMass->value() <= 0.0)
    parsingInfo(tr("You must also set the 'mass' to a positive value when specifying the 'inertiaMatrix'."));

  // The principal moment of inertia must be positive
  // to avoid assertion failure with debug version of ODE
  const WbVector3 &v0 = mInertiaMatrix->item(0);
  if (v0[0] <= 0.0 || v0[1] <= 0.0 || v0[2] <= 0.0) {
    mHasAvalidInertiaMatrix = false;
    if (showInfo)
      parsingWarn(tr("The first line of 'inertiaMatrix' (principal moments of inertia) must have only positive values."));
  }
  const WbVector3 &v1 = mInertiaMatrix->item(1);
  const dReal I[12] = {v0[0], v1[0], v1[1], 0.0, v1[0], v0[1], v1[2], 0.0, v1[1], v1[2], v0[2], 0.0};
  if (!dIsPositiveDefinite(I, 3)) {
    mHasAvalidInertiaMatrix = false;
    if (showInfo)
      parsingWarn(tr("'inertiaMatrix' must be positive definite."));
  } else if (mCenterOfMass->size() == 0 && showInfo)
    parsingWarn(tr("'centerOfmass' must also be specified when using an inertia matrix."));
  else if (mMass->value() < 0.0 && showInfo)
    parsingWarn(tr("'mass' must be positive when using an inertia matrix."));
}

WbDamping *WbPhysics::damping() const {
  return static_cast<WbDamping *>(mDamping->value());
}

void WbPhysics::updateMode() {
  const bool massIsPositive = mMass->value() > 0.0;
  const int comSize = mCenterOfMass->size();

  checkInertiaMatrix(false);

  if (massIsPositive && comSize == 1 && mHasAvalidInertiaMatrix)
    mMode = CUSTOM_INERTIA_MATRIX;
  else if ((massIsPositive || mDensity->value() > 0.0) && mInertiaMatrix->size() == 0)
    mMode = BOUNDING_OBJECT_BASED;
  else
    mMode = INVALID;
}

void WbPhysics::updateDensity() {
  if (mSkipUpdate)
    return;

  checkDensity();
  updateMode();

  if (mMode == BOUNDING_OBJECT_BASED)
    emit massOrDensityChanged();
}

void WbPhysics::updateMass() {
  if (mSkipUpdate)
    return;

  checkMass();
  updateMode();

  if (mMode == CUSTOM_INERTIA_MATRIX)
    emit inertialPropertiesChanged();
  else if (mMode == BOUNDING_OBJECT_BASED)
    emit massOrDensityChanged();
}

void WbPhysics::updateCenterOfMass() {
  assert(mCenterOfMass->size() <= 1);

  if (mSkipUpdate)
    return;

  checkCenterOfMass();

  updateMode();

  if (mMode == CUSTOM_INERTIA_MATRIX)
    emit inertialPropertiesChanged();
  else if (mMode == BOUNDING_OBJECT_BASED)
    emit centerOfMassChanged();
}

void WbPhysics::updateInertiaMatrix() {
  if (mSkipUpdate)
    return;

  checkInertiaMatrix(true);

  updateMode();

  if (mMode == CUSTOM_INERTIA_MATRIX)
    emit inertialPropertiesChanged();
  else if (mMode == BOUNDING_OBJECT_BASED)
    emit modeSwitched();
}

void WbPhysics::updateDamping() {
  if (mSkipUpdate)
    return;

  if (damping())
    connect(damping(), &WbDamping::changed, this, &WbPhysics::dampingChanged, Qt::UniqueConnection);

  emit dampingChanged();
}

void WbPhysics::setMass(double mass, bool skipUpdate) {
  mSkipUpdate = skipUpdate;
  mMass->setValue(mass);
  mSkipUpdate = false;
}

void WbPhysics::setDensity(double density, bool skipUpdate) {
  mSkipUpdate = skipUpdate;
  mDensity->setValue(density);
  mSkipUpdate = false;
}

void WbPhysics::setCenterOfMass(double x, double y, double z, bool skipUpdate) {
  assert(mCenterOfMass->size() <= 1);
  mSkipUpdate = skipUpdate;
  const WbVector3 v(x, y, z);
  if (mCenterOfMass->size() == 0)
    mCenterOfMass->insertItem(0, v);
  else
    mCenterOfMass->setItem(0, v);
  mSkipUpdate = false;
}

void WbPhysics::setInertiaMatrix(double v0x, double v0y, double v0z, double v1x, double v1y, double v1z, bool skipUpdate) {
  assert(mInertiaMatrix->size() == 0);
  mSkipUpdate = skipUpdate;
  mInertiaMatrix->insertItem(0, WbVector3(v0x, v0y, v0z));
  mInertiaMatrix->insertItem(1, WbVector3(v1x, v1y, v1z));
  mSkipUpdate = false;
}

bool WbPhysics::exportNodeHeader(WbWriter &writer) const {
  if (writer.isUrdf())
    return true;
  return WbNode::exportNodeHeader(writer);
}
