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

#include "WbCone.hpp"

#include "WbAffinePlane.hpp"
#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbNodeUtilities.hpp"
#include "WbRay.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSFBool.hpp"
#include "WbSFInt.hpp"
#include "WbSimulationState.hpp"
#include "WbTransform.hpp"
#include "WbVector2.hpp"

#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <cmath>

void WbCone::init() {
  mBottomRadius = findSFDouble("bottomRadius");
  mHeight = findSFDouble("height");
  mSide = findSFBool("side");
  mBottom = findSFBool("bottom");
  mSubdivision = findSFInt("subdivision");

  mResizeConstraint = WbWrenAbstractResizeManipulator::X_EQUAL_Z;
}

WbCone::WbCone(WbTokenizer *tokenizer) : WbGeometry("Cone", tokenizer) {
  init();
  if (tokenizer == NULL) {
    mBottomRadius->setValueNoSignal(0.05);
    mHeight->setValueNoSignal(0.1);
  }
}

WbCone::WbCone(const WbCone &other) : WbGeometry(other) {
  init();
}

WbCone::WbCone(const WbNode &other) : WbGeometry(other) {
  init();
}

WbCone::~WbCone() {
  wr_static_mesh_delete(mWrenMesh);
}

void WbCone::postFinalize() {
  WbGeometry::postFinalize();

  connect(mBottomRadius, &WbSFDouble::changed, this, &WbCone::updateBottomRadius);
  connect(mHeight, &WbSFDouble::changed, this, &WbCone::updateHeight);
  connect(mSide, &WbSFBool::changed, this, &WbCone::updateSide);
  connect(mBottom, &WbSFBool::changed, this, &WbCone::updateBottom);
  connect(mSubdivision, &WbSFInt::changed, this, &WbCone::updateSubdivision);
}

void WbCone::createWrenObjects() {
  WbGeometry::createWrenObjects();

  sanitizeFields();
  buildWrenMesh();

  emit wrenObjectsCreated();
}

void WbCone::setResizeManipulatorDimensions() {
  WbVector3 scale(mBottomRadius->value(), mHeight->value(), mBottomRadius->value());

  WbTransform *transform = upperTransform();
  if (transform)
    scale *= transform->matrix().scale();

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

void WbCone::createResizeManipulator() {
  mResizeManipulator = new WbRegularResizeManipulator(uniqueId(), WbWrenAbstractResizeManipulator::ResizeConstraint::X_EQUAL_Z);
}

bool WbCone::areSizeFieldsVisibleAndNotRegenerator() const {
  const WbField *const height = findField("height", true);
  const WbField *const radius = findField("bottomRadius", true);
  return WbNodeUtilities::isVisible(height) && WbNodeUtilities::isVisible(radius) &&
         !WbNodeUtilities::isTemplateRegeneratorField(height) && !WbNodeUtilities::isTemplateRegeneratorField(radius);
}

void WbCone::exportNodeFields(WbVrmlWriter &writer) const {
  WbGeometry::exportNodeFields(writer);
  if (writer.isX3d())
    writer << " subdivision=\'" << mSubdivision->value() << "\'";
}

bool WbCone::sanitizeFields() {
  if (isInBoundingObject())
    return false;

  if (WbFieldChecker::resetIntIfNotInRangeWithIncludedBounds(this, mSubdivision, 3, 1000, 3))
    return false;

  if (WbFieldChecker::resetDoubleIfNonPositive(this, mBottomRadius, 1.0))
    return false;

  if (WbFieldChecker::resetDoubleIfNonPositive(this, mHeight, 1.0))
    return false;

  return true;
}

void WbCone::buildWrenMesh() {
  WbGeometry::deleteWrenRenderable();

  wr_static_mesh_delete(mWrenMesh);
  mWrenMesh = NULL;

  if (mBottom->isFalse() && mSide->isFalse())
    return;

  WbGeometry::computeWrenRenderable();

  mWrenMesh = wr_static_mesh_unit_cone_new(mSubdivision->value(), mSide->isTrue(), mBottom->isTrue());

  wr_renderable_set_mesh(mWrenRenderable, WR_MESH(mWrenMesh));

  updateScale();
}

void WbCone::rescale(const WbVector3 &scale) {
  if (scale.x() != 1.0)
    setBottomRadius(bottomRadius() * scale.x());
  else if (scale.z() != 1.0)
    setBottomRadius(bottomRadius() * scale.z());

  if (scale.y() != 1.0)
    setHeight(height() * scale.y());
}

double WbCone::height() const {
  return mHeight->value();
}

double WbCone::bottomRadius() const {
  return mBottomRadius->value();
}

void WbCone::setHeight(double h) {
  mHeight->setValue(h);
}

void WbCone::setBottomRadius(double r) {
  mBottomRadius->setValue(r);
}

void WbCone::updateBottomRadius() {
  if (!sanitizeFields())
    return;

  updateScale();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCone::updateHeight() {
  if (!sanitizeFields())
    return;

  updateScale();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCone::updateSide() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCone::updateBottom() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbCone::updateSubdivision() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  emit changed();
}

void WbCone::updateScale() {
  float scale[] = {static_cast<float>(mBottomRadius->value()), static_cast<float>(mHeight->value()),
                   static_cast<float>(mBottomRadius->value())};
  wr_transform_set_scale(wrenNode(), scale);
}

double WbCone::scaledHeight() const {
  return fabs(mHeight->value() * absoluteScale().y());
}

double WbCone::scaledBottomRadius() const {
  const WbVector3 &scale = absoluteScale();
  return fabs(mBottomRadius->value() * std::max(scale.x(), scale.z()));
}

/////////////////
// Ray Tracing //
/////////////////

bool WbCone::pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet) const {
  WbVector3 localCollisionPoint;
  const double collisionDistance = computeLocalCollisionPoint(localCollisionPoint, ray);
  if (collisionDistance < 0.0)
    return false;

  const double h = scaledHeight();
  const double r = scaledBottomRadius() * (0.5 - localCollisionPoint.y() / h);

  double u, v;
  if (localCollisionPoint.y() == -h / 2.0) {
    // bottom face
    if (localCollisionPoint.x() * localCollisionPoint.x() + localCollisionPoint.z() * localCollisionPoint.z() > r * r)
      return false;

    u = (localCollisionPoint.x() + r) / (2.0 * r);
    v = 1.0 - (localCollisionPoint.z() + r) / (2.0 * r);

    if (textureCoordSet == 1) {
      u = u * 0.5 + 0.5;
    }

  } else {
    // body
    double theta = atan(localCollisionPoint.x() / localCollisionPoint.z());
    theta -= floor(theta / (2.0 * M_PI)) * 2.0 * M_PI;
    if (theta < M_PI && localCollisionPoint.x() > 0 && localCollisionPoint.z() > 0.0) {
      theta += M_PI;
    } else if (theta < M_PI && localCollisionPoint.x() > 0.0 && localCollisionPoint.z() < 0.0) {
      theta += M_PI;
    } else if (theta > M_PI && localCollisionPoint.x() < 0.0 && localCollisionPoint.z() > 0.0) {
      theta -= M_PI;
    } else if (theta > M_PI && localCollisionPoint.x() < 0.0 && localCollisionPoint.z() < 0.0) {
      theta -= M_PI;
    }
    u = theta / (2.0 * M_PI);
    v = 1.0 - (localCollisionPoint.y() + h / 2.0) / h;

    if (textureCoordSet == 1)
      u *= 0.5;
  }

  uv.setXy(u, v);
  return true;
}

double WbCone::computeDistance(const WbRay &ray) const {
  WbVector3 collisionPoint;
  return computeLocalCollisionPoint(collisionPoint, ray);
}

double WbCone::computeLocalCollisionPoint(WbVector3 &point, const WbRay &ray) const {
  WbVector3 direction(ray.direction());
  WbVector3 origin(ray.origin());

  const WbTransform *const transform = upperTransform();
  if (transform) {
    direction = ray.direction() * transform->matrix();
    direction.normalize();
    origin = transform->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
  }
  const double radius = scaledBottomRadius();
  const double radius2 = radius * radius;
  const double h = scaledHeight();
  double d = std::numeric_limits<double>::infinity();

  // distance from body
  if (mSide->value()) {
    const double k = radius2 / (h * h);
    const double halfH = h / 2.0;
    const double o = origin.y() - halfH;
    const double a = direction.x() * direction.x() + direction.z() * direction.z() - k * direction.y() * direction.y();
    const double b = 2.0 * (origin.x() * direction.x() + origin.z() * direction.z() - k * o * direction.y());
    const double c = origin.x() * origin.x() + origin.z() * origin.z() - k * o * o;
    double discriminant = b * b - 4.0 * a * c;

    // if c < 0: ray origin is inside the cone body
    if (c >= 0 && discriminant > 0) {
      // ray intersects the sphere in two points
      discriminant = sqrt(discriminant);
      const double t1 = (-b - discriminant) / (2 * a);
      const double t2 = (-b + discriminant) / (2 * a);
      const double y1 = origin.y() + t1 * direction.y();
      const double y2 = origin.y() + t2 * direction.y();
      if (mSide->value() && t1 > 0.0 && y1 >= -halfH && y1 <= halfH)
        d = t1;
      else if (mSide->value() && t2 > 0.0 && y2 >= -halfH && y2 <= halfH)
        d = t2;
    }
  }

  // distance from bottom face
  if (mBottom->value()) {
    std::pair<bool, double> intersection =
      WbRay(origin, direction).intersects(WbAffinePlane(WbVector3(0.0, -1.0, 0.0), WbVector3(0.0, -h / 2.0, 0.0)), true);
    if (mBottom->value() && intersection.first && intersection.second > 0.0 && intersection.second < d) {
      const WbVector3 &p = origin + intersection.second * direction;
      if (p.x() * p.x() + p.z() * p.z() <= radius2) {
        d = intersection.second;
      }
    }
  }

  if (d == std::numeric_limits<double>::infinity())
    return -1;

  point = origin + d * direction;
  return d;
}

void WbCone::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  const bool side = mSide->value();
  const double radius = scaledBottomRadius();
  const double height = scaledHeight();
  const double halfHeight = height / 2.0;

  if (!side || height <= radius)  // consider it as disk
    mBoundingSphere->set(WbVector3(0, -halfHeight, 0), radius);
  else {
    const double newRadius = halfHeight + radius * radius / (2 * height);
    mBoundingSphere->set(WbVector3(0, halfHeight - newRadius, 0), newRadius);
  }
}

////////////////////////
// Friction Direction //
////////////////////////

WbVector3 WbCone::computeFrictionDirection(const WbVector3 &normal) const {
  parsingWarn(
    tr("A Cone is used in a Bounding object using an asymmetric friction. Cone does not support asymmetric friction"));
  return WbVector3(0, 0, 0);
}
