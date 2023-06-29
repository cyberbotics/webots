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

#include "WbFluid.hpp"

#include "WbField.hpp"
#include "WbGeometry.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbMFNode.hpp"
#include "WbMFVector3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMatrix3.hpp"
#include "WbMatrix4.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbPlane.hpp"
#include "WbRotation.hpp"
#include "WbSimulationState.hpp"
#include "WbSolidUtilities.hpp"
#include "WbVector4.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"
#include "WbWrenRenderingContext.hpp"

#include <ode/fluid_dynamics/ode_fluid_dynamics.h>
#include <ode/ode.h>

#include <QtCore/QStringList>

void WbFluid::init() {
  mOdeFluid = NULL;
  mDensity = findSFDouble("density");
  mViscosity = findSFDouble("viscosity");
  mStreamVelocity = findSFVector3("streamVelocity");
}

WbFluid::WbFluid(WbTokenizer *tokenizer) : WbMatter("Fluid", tokenizer) {
  init();
}

WbFluid::WbFluid(const WbFluid &other) : WbMatter(other) {
  init();
}

WbFluid::WbFluid(const WbNode &other) : WbMatter(other) {
  init();
}

WbFluid::WbFluid(const QString &modelName, WbTokenizer *tokenizer) : WbMatter(modelName, tokenizer) {
  init();
}
void WbFluid::preFinalize() {
  WbMatter::preFinalize();

  if (mBoundingObject->value())
    boundingObject()->preFinalize();

  setMatrixNeedUpdate();  // force the matrix update after the first ode update
}

void WbFluid::postFinalize() {
  WbMatter::postFinalize();

  connect(mDensity, &WbSFDouble::changed, this, &WbFluid::updateDensity);
  connect(mViscosity, &WbSFDouble::changed, this, &WbFluid::updateViscosity);
  connect(mStreamVelocity, &WbSFVector3::changed, this, &WbFluid::updateStreamVelocity);
}

void WbFluid::createWrenObjects() {
  WbMatter::createWrenObjects();
  // Connects signals for further updates
  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this, &WbFluid::updateLineScale);

  applyChangesToWren();
}

////////////////////////////
//   Create ODE Objects   //
////////////////////////////

void WbFluid::createOdeObjects() {
  if (mBoundingObject->value())
    boundingObject()->createOdeObjects();

  mOdeFluid = dFluidCreate(WbOdeContext::instance()->world());
  updateDensity();
  updateViscosity();
  updateStreamVelocity();

  createOdeGeoms();
  dGeomID g = odeGeom();
  if (g)
    attachGeomsToFluid(g);

  // Recurses through solid descendants
  WbMatter::createOdeObjects();

  updateOdeGeomPosition();
}

void WbFluid::createOdeGeoms() {
  assert(odeGeom() == NULL);
  dSpaceID space = WbOdeContext::instance()->space();
  createOdeGeomFromBoundingObject(space);
}

void WbFluid::setGeomMatter(dGeomID g, WbBaseNode *node) {
  assert(mOdeFluid);
  dGeomSetFluid(g, mOdeFluid);
}

/////////////////////
// Update Methods  //
/////////////////////

// Resets recursively ODE dGeoms positions
void WbFluid::handleJerk() {
  jerk(false);
  WbWorld::instance()->awake();
}

void WbFluid::updateBoundingObject() {
  if (mBoundingObject->value() != NULL) {
    WbBaseNode *node = dynamic_cast<WbBaseNode *>(mBoundingObject->value());
    assert(node);
    if (!isBoundingObjectFinalizationCompleted(node))
      // postpone bounding object update after finalization
      return;

    createOdeGeoms();
    dGeomID g = odeGeom();
    if (g)
      attachGeomsToFluid(g);
  }

  mBoundingObjectHasChanged = true;
}

// Sets the fluid into placeable ODE dGeoms
void WbFluid::attachGeomsToFluid(dGeomID g) {
  dSpaceID space = WbSolidUtilities::dynamicCastInSpaceID(g);
  if (space) {
    const int n = dSpaceGetNumGeoms(space);
    for (int i = 0; i < n; ++i)
      attachGeomsToFluid(dSpaceGetGeom(space, i));
    return;
  }

  dGeomSetFluid(g, mOdeFluid);
}

void WbFluid::createOdeGeomFromInsertedGroupItem(WbBaseNode *node) {
  assert(node);

  dSpaceID space = upperSpace();
  assert(space);

  dGeomID insertedGeom = createOdeGeomFromNode(space, node);
  if (!insertedGeom)  // if the inserted node has no Geometry child or it has an indexed face set which is invalid
    return;

  // Attaches the dGeom to fluid body and adjusts the mass
  updateOdeGeomPosition(insertedGeom);
  dGeomSetFluid(insertedGeom, mOdeFluid);
}

void WbFluid::updateDensity() {
  const double d = mDensity->value();
  if (d < 0.0) {
    parsingWarn(tr("'density' must be greater than or equal to zero. Reset to default value 1000 kg/m^3"));
    mDensity->setValue(1000.0);
    return;
  }

  if (mOdeFluid)
    dFluidSetDensity(mOdeFluid, mDensity->value());
}

void WbFluid::updateViscosity() {
  const double v = mViscosity->value();
  if (v < 0.0) {
    parsingWarn(tr("'viscosity' must be greater than or equal to zero. Reset to default value 0.001 kg/(ms)"));
    mViscosity->setValue(1000.0);
    return;
  }

  if (mOdeFluid)
    dFluidSetViscosity(mOdeFluid, mViscosity->value());
}

void WbFluid::updateStreamVelocity() {
  if (mOdeFluid)
    dFluidSetStreamVel(mOdeFluid, mStreamVelocity->value().ptr());
}

double WbFluid::density() const {
  return mDensity->value();
}

double WbFluid::viscosity() const {
  return mViscosity->value();
}

void WbFluid::propagateSelection(bool selected) {
  select(selected);

  WbBaseNode *const bo = boundingObject();
  if (bo)
    bo->propagateSelection(selected);
}

void WbFluid::jerk(bool resetVelocities, bool rootJerk) {
  updateOdeGeomPosition();
}

// Collision and sleep flags management

void WbFluid::propagateBoundingObjectMaterialUpdate(bool onSelection) {
  WbBaseNode *const bo = boundingObject();
  if (!bo)
    return;

  const bool triggerChange = mBoundingObjectHasChanged || onSelection;

  // Update with current collision and sleep flags
  if (triggerChange) {
    bo->updateCollisionMaterial(true);

    updateSleepFlag();
    return;
  }

  bo->updateCollisionMaterial();
  updateSleepFlag();
}
