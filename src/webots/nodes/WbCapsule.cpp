// Copyright 1996-2024 Cyberbotics Ltd.
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

#include "WbCapsule.hpp"

#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbMatter.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeGeomData.hpp"
#include "WbPose.hpp"
#include "WbRay.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSFBool.hpp"
#include "WbSFInt.hpp"
#include "WbSimulationState.hpp"
#include "WbTransform.hpp"
#include "WbVrmlNodeUtilities.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <ode/ode.h>
#include <cmath>

void WbCapsule::init() {
  mBottom = findSFBool("bottom");
  mRadius = findSFDouble("radius");
  mHeight = findSFDouble("height");
  mSide = findSFBool("side");
  mTop = findSFBool("top");
  mSubdivision = findSFInt("subdivision");

  mResizeConstraint = WbWrenAbstractResizeManipulator::X_EQUAL_Y;
}

WbCapsule::WbCapsule(WbTokenizer *tokenizer) : WbGeometry("Capsule", tokenizer) {
  init();
  if (tokenizer == NULL) {
    mRadius->setValueNoSignal(0.05);
    mHeight->setValueNoSignal(0.1);
  }
}

WbCapsule::WbCapsule(const WbCapsule &other) : WbGeometry(other) {
  init();
}

WbCapsule::WbCapsule(const WbNode &other) : WbGeometry(other) {
  init();
}

WbCapsule::~WbCapsule() {
  wr_static_mesh_delete(mWrenMesh);
}

void WbCapsule::postFinalize() {
  WbGeometry::postFinalize();

  connect(mBottom, &WbSFBool::changed, this, &WbCapsule::updateBottom);
  connect(mRadius, &WbSFDouble::changed, this, &WbCapsule::updateRadius);
  connect(mHeight, &WbSFDouble::changed, this, &WbCapsule::updateHeight);
  connect(mSide, &WbSFBool::changed, this, &WbCapsule::updateSide);
  connect(mTop, &WbSFBool::changed, this, &WbCapsule::updateTop);
  connect(mSubdivision, &WbSFInt::changed, this, &WbCapsule::updateSubdivision);
}

void WbCapsule::createWrenObjects() {
  WbGeometry::createWrenObjects();

  if (isInBoundingObject()) {
    connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this, &WbCapsule::updateLineScale);

    if (mSubdivision->value() < MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION && !WbVrmlNodeUtilities::hasAUseNodeAncestor(this))
      // silently reset the subdivision on node initialization
      mSubdivision->setValue(MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION);
  }

  sanitizeFields();
  buildWrenMesh();

  emit wrenObjectsCreated();
}

void WbCapsule::setResizeManipulatorDimensions() {
  WbVector3 scale(1.0f, 1.0f, 1.0f);

  const WbTransform *const up = upperTransform();
  if (up)
    scale *= up->absoluteScale();

  if (isAValidBoundingObject())
    scale *= 1.0f + (wr_config_get_line_scale() / LINE_SCALE_FACTOR);

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

void WbCapsule::createResizeManipulator() {
  mResizeManipulator =
    new WbRegularResizeManipulator(uniqueId(), (WbWrenAbstractResizeManipulator::ResizeConstraint)mResizeConstraint);
}

bool WbCapsule::areSizeFieldsVisibleAndNotRegenerator() const {
  const WbField *const heightField = findField("height", true);
  const WbField *const radiusField = findField("radius", true);
  return WbVrmlNodeUtilities::isVisible(heightField) && WbVrmlNodeUtilities::isVisible(radiusField) &&
         !WbNodeUtilities::isTemplateRegeneratorField(heightField) && !WbNodeUtilities::isTemplateRegeneratorField(radiusField);
}

bool WbCapsule::sanitizeFields() {
  if (WbFieldChecker::resetIntIfNotInRangeWithIncludedBounds(this, mSubdivision, 4, 1000, 4))
    return false;
  if (mSubdivision->value() < MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION && isInBoundingObject() &&
      !WbVrmlNodeUtilities::hasAUseNodeAncestor(this)) {
    parsingWarn(tr("'subdivision' value has no effect to physical 'boundingObject' geometry. "
                   "A minimum value of %2 is used for the representation.")
                  .arg(MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION));
    mSubdivision->setValue(MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION);
    return false;
  }

  if (WbFieldChecker::resetDoubleIfNonPositive(this, mRadius, 1.0))
    return false;

  if (WbFieldChecker::resetDoubleIfNonPositive(this, mHeight, 1.0))
    return false;

  return true;
}

void WbCapsule::buildWrenMesh() {
  const bool resizeManipulator = mResizeManipulator && mResizeManipulator->isAttached();
  WbGeometry::deleteWrenRenderable();

  wr_static_mesh_delete(mWrenMesh);
  mWrenMesh = NULL;

  if (mBottom->isFalse() && mSide->isFalse() && mTop->isFalse())
    return;

  WbGeometry::computeWrenRenderable();

  // This must be done after WbGeometry::computeWrenRenderable() otherwise
  // the outline scaling is applied to the wrong WREN transform
  if (isInBoundingObject())
    updateLineScale();

  // Reattach resize manipulator
  if (mResizeManipulator) {
    mResizeManipulator->attachTo(mWrenScaleTransform);
    if (resizeManipulator)
      mResizeManipulator->show();
  }

  // Restore pickable state
  setPickable(isPickable());

  const bool createOutlineMesh = isInBoundingObject();
  const int subdivision = (createOutlineMesh && mSubdivision->value() < MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION) ?
                            MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION :
                            mSubdivision->value();

  mWrenMesh = wr_static_mesh_capsule_new(subdivision, mRadius->value(), mHeight->value(), mSide->isTrue(), mTop->isTrue(),
                                         mBottom->isTrue(), createOutlineMesh);

  wr_renderable_set_mesh(mWrenRenderable, WR_MESH(mWrenMesh));
}

void WbCapsule::rescale(const WbVector3 &scale) {
  if (scale.x() != 1.0)
    setRadius(radius() * scale.x());
  else if (scale.z() != 1.0)
    setRadius(radius() * scale.z());

  if (scale.y() != 1.0)
    setHeight(height() * scale.y());
}

void WbCapsule::updateBottom() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCapsule::updateRadius() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (isInBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCapsule::updateHeight() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (isInBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCapsule::updateSide() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCapsule::updateTop() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCapsule::updateSubdivision() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  emit changed();
}

void WbCapsule::updateLineScale() {
  if (!isAValidBoundingObject())
    return;

  float offset = wr_config_get_line_scale() / LINE_SCALE_FACTOR;

  float scale[] = {static_cast<float>(1.0f + offset), static_cast<float>(1.0f + offset), static_cast<float>(1.0f + offset)};
  wr_transform_set_scale(wrenNode(), scale);
}

QStringList WbCapsule::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "radius"
         << "height"
         << "subdivision"
         << "bottom"
         << "side"
         << "top";
  return fields;
}

/////////////////
// ODE objects //
/////////////////

dGeomID WbCapsule::createOdeGeom(dSpaceID space) {
  if (mRadius->value() <= 0.0) {
    parsingWarn(tr("'radius' must be positive when used in a 'boundingObject'."));
    return NULL;
  }

  if (mHeight->value() <= 0.0) {
    parsingWarn(tr("'height' must be positive when used in a 'boundingObject'."));
    return NULL;
  }

  if (WbNodeUtilities::findUpperMatter(this)->nodeType() == WB_NODE_FLUID)
    checkFluidBoundingObjectOrientation();

  return dCreateCapsule(space, scaledRadius(), scaledHeight());
}

void WbCapsule::applyToOdeData(bool correctSolidMass) {
  if (mOdeGeom == NULL)
    return;

  assert(dGeomGetClass(mOdeGeom) == dCapsuleClass);
  dGeomCapsuleSetParams(mOdeGeom, scaledRadius(), scaledHeight());

  WbOdeGeomData *const odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(mOdeGeom));
  assert(odeGeomData);
  odeGeomData->setLastChangeTime(WbSimulationState::instance()->time());

  if (correctSolidMass)
    applyToOdeMass();
}

double WbCapsule::scaledRadius() const {
  const WbVector3 &scale = absoluteScale();
  return fabs(mRadius->value() * std::max(scale.x(), scale.y()));
}

double WbCapsule::scaledHeight() const {
  return fabs(mHeight->value() * absoluteScale().z());
}

bool WbCapsule::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidRadius = mRadius->value() <= 0.0;
  const bool invalidHeight = mHeight->value() <= 0.0;
  if (warning) {
    if (invalidRadius)
      parsingWarn(tr("'radius' must be positive when used in a 'boundingObject'."));

    if (invalidHeight)
      parsingWarn(tr("'height' must be positive when used in a 'boundingObject'."));
  }

  return (!invalidRadius && !invalidHeight);
}

bool WbCapsule::isAValidBoundingObject(bool checkOde, bool warning) const {
  const bool admissible = WbGeometry::isAValidBoundingObject(checkOde, warning);
  return admissible && isSuitableForInsertionInBoundingObject(admissible && warning);
}
/////////////////
// Ray Tracing //
/////////////////

bool WbCapsule::pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet) const {
  WbVector3 localCollisionPoint;
  double collisionDistance = computeLocalCollisionPoint(localCollisionPoint, ray);
  if (collisionDistance < 0)
    return false;

  const double theta = atan2(localCollisionPoint.x(), -localCollisionPoint.y()) + M_PI;
  const double u = 0.5 * theta / M_PI;

  // default: offset of the top half sphere
  double v = 0.0;
  const double h = scaledHeight();
  const double h2 = 0.5 * h;
  const double r = scaledRadius();
  const double absZ = fabs(localCollisionPoint.z());
  if (absZ <= h2) {
    // body
    v = (2.0 - (localCollisionPoint.z() + h2) / h) / 3.0;

  } else {
    // top and bottom half sphere

    double zIi;
    const double zI = absZ - h2;
    const int sub4 = 0.25 * mSubdivision->value();
    const int sub5 = sub4 + 1;
    double prevD = 0;
    const int p = 3 * sub4;
    const double factor4 = M_PI_2 / sub4;
    for (int i = 1; i < sub5; i++) {
      double alpha = factor4 * i;
      double d = r * sin(alpha);
      if (zI < d) {
        zIi = zI - prevD;
        v = (double)(i - 1 + 2 * sub4) / p + zIi / (p * (d - prevD));
        break;
      }

      prevD = d;
    }

    // if top half sphere
    if (localCollisionPoint.z() > 0)
      v = 1.0 - v;
  }

  // result
  uv.setXy(u, v);
  return true;
}

double WbCapsule::computeDistance(const WbRay &ray) const {
  WbVector3 collisionPoint;
  return computeLocalCollisionPoint(collisionPoint, ray);
}

double WbCapsule::computeLocalCollisionPoint(WbVector3 &point, const WbRay &ray) const {
  WbVector3 direction(ray.direction());
  WbVector3 origin(ray.origin());
  const WbPose *const up = upperPose();
  if (up) {
    direction = ray.direction() * up->matrix();
    direction.normalize();
    origin = up->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
  }

  const double r = scaledRadius();
  const double halfH = scaledHeight() * 0.5;
  double d = std::numeric_limits<double>::infinity();

  // distance from cylinder body
  if (mSide->value()) {
    const double a = direction.x() * direction.x() + direction.y() * direction.y();
    const double b = 2.0 * (origin.x() * direction.x() + origin.y() * direction.y());
    const double c = origin.x() * origin.x() + origin.y() * origin.y() - r * r;
    double discriminant = b * b - 4.0 * a * c;

    // if c < 0: ray origin is inside the cylinder body
    if (c >= 0 && discriminant > 0.0) {
      discriminant = sqrt(discriminant);
      const double t1 = (-b - discriminant) / (2 * a);
      const double t2 = (-b + discriminant) / (2 * a);
      const double z1 = origin.z() + t1 * direction.z();
      const double z2 = origin.z() + t2 * direction.z();
      if (t1 > 0.0 && z1 >= -halfH && z1 <= halfH)
        d = t1;
      else if (t2 > 0.0 && z2 >= -halfH && z2 <= halfH)
        d = t2;
    }
  }

  // distance with top half sphere
  if (mTop->value()) {
    std::pair<bool, double> intersection = WbRay(origin, direction).intersects(WbVector3(0, 0, halfH), r, true);
    if (intersection.first && intersection.second > 0 && intersection.second < d) {
      double z = origin.z() + intersection.second * direction.z();
      if (z >= halfH)
        d = intersection.second;
    }
  }

  // distance with bottom half spheres
  if (mBottom->value()) {
    std::pair<bool, double> intersection = WbRay(origin, direction).intersects(WbVector3(0, 0, -halfH), r, true);
    if (intersection.first && intersection.second > 0 && intersection.second < d) {
      double z = origin.z() + intersection.second * direction.z();
      if (z <= -halfH)
        d = intersection.second;
    }
  }

  if (d == std::numeric_limits<double>::infinity())
    return -1;

  point = origin + d * direction;
  return d;
}

void WbCapsule::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  const bool top = mTop->value();
  const bool side = mSide->value();
  const bool bottom = mBottom->value();
  const double halfHeight = mHeight->value() / 2.0;
  const double r = mRadius->value();

  if (!top && !side && !bottom) {  // it is empty
    mBoundingSphere->empty();
    return;
  }

  if (top + side + bottom == 1) {
    if (top || bottom)
      mBoundingSphere->set(WbVector3(0, 0, top ? halfHeight : -halfHeight), r);
    else  // side
      mBoundingSphere->set(WbVector3(), WbVector3(r, 0, halfHeight).length());
  } else if (top != bottom) {  // we have 'top and side' or 'side and bottom'
    const double maxZ = top ? halfHeight + r : halfHeight;
    const double minZ = bottom ? -halfHeight - r : -halfHeight;
    const double totalHeight = (maxZ - minZ);
    const double newRadius = totalHeight / 2.0 + r * r / (2 * totalHeight);
    const double offsetZ = top ? (maxZ - newRadius) : (minZ + newRadius);
    mBoundingSphere->set(WbVector3(0, 0, offsetZ), newRadius);
  } else  // complete capsule
    mBoundingSphere->set(WbVector3(), halfHeight + r);
}

////////////////////////
// Friction Direction //
////////////////////////

WbVector3 WbCapsule::computeFrictionDirection(const WbVector3 &normal) const {
  parsingWarn(
    tr("A Capsule is used in a Bounding object using an asymmetric friction. Capsule does not support asymmetric friction"));
  return WbVector3(0, 0, 0);
}
