// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbPlane.hpp"

#include "WbAffinePlane.hpp"
#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbNodeUtilities.hpp"
#include "WbRay.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSFVector2.hpp"
#include "WbSimulationState.hpp"
#include "WbTransform.hpp"
#include "WbWrenAbstractResizeManipulator.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWriter.hpp"

#include <wren/config.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <ode/ode.h>

#include <algorithm>

void WbPlane::init() {
  mSize = findSFVector2("size");
}

WbPlane::WbPlane(WbTokenizer *tokenizer) : WbGeometry("Plane", tokenizer) {
  init();
}

WbPlane::WbPlane(const WbPlane &other) : WbGeometry(other) {
  init();
}

WbPlane::WbPlane(const WbNode &other) : WbGeometry(other) {
  init();
}

WbPlane::~WbPlane() {
  wr_static_mesh_delete(mWrenMesh);
}

void WbPlane::postFinalize() {
  WbGeometry::postFinalize();

  connect(mSize, &WbSFVector2::changed, this, &WbPlane::updateSize);
}

const WbVector2 &WbPlane::size() const {
  return mSize->value();
}

void WbPlane::setSize(const WbVector2 &size) {
  mSize->setValue(size);
}

void WbPlane::setSize(double x, double y) {
  mSize->setValue(x, y);
}

void WbPlane::setX(double x) {
  mSize->setX(x);
}

void WbPlane::setY(double y) {
  mSize->setY(y);
}

const WbVector2 WbPlane::scaledSize() const {
  const WbVector2 &s1 = mSize->value();
  const WbVector3 &s2 = absoluteScale();
  return WbVector2(fabs(s2.x() * s1.x()), fabs(s2.y() * s1.y()));
}

void WbPlane::write(WbWriter &writer) const {
  if (writer.isWebots())
    WbGeometry::write(writer);
  else
    writeExport(writer);
}

void WbPlane::exportNodeFields(WbWriter &writer) const {
  if (writer.isWebots())
    WbGeometry::exportNodeFields(writer);
  else if (writer.isX3d()) {
    writer << " size=\'";
    mSize->write(writer);
    writer << "\'";
  }
}

void WbPlane::createWrenObjects() {
  WbGeometry::createWrenObjects();
  WbGeometry::computeWrenRenderable();

  sanitizeFields();

  const bool createOutlineMesh = isInBoundingObject();

  mWrenMesh = wr_static_mesh_unit_rectangle_new(createOutlineMesh);

  wr_renderable_set_mesh(mWrenRenderable, WR_MESH(mWrenMesh));

  updateSize();

  if (createOutlineMesh)
    connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this, &WbPlane::updateLineScale);

  emit wrenObjectsCreated();
}

void WbPlane::createResizeManipulator() {
  mResizeManipulator = new WbPlaneResizeManipulator(uniqueId());
}

void WbPlane::setResizeManipulatorDimensions() {
  WbVector3 scale(size().x(), size().y(), 0.1f * std::min(mSize->value().x(), mSize->value().y()));
  WbTransform *transform = upperTransform();
  if (transform)
    scale *= transform->absoluteScale();

  if (isAValidBoundingObject()) {
    float offset = 1.0f + (wr_config_get_line_scale() / LINE_SCALE_FACTOR);
    scale *= WbVector3(offset, offset, 1.0f);
  }

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

bool WbPlane::areSizeFieldsVisibleAndNotRegenerator() const {
  const WbField *const sizeField = findField("size", true);
  return WbNodeUtilities::isVisible(sizeField) && !WbNodeUtilities::isTemplateRegeneratorField(sizeField);
}

bool WbPlane::sanitizeFields() {
  if (WbFieldChecker::resetVector2IfNonPositive(this, mSize, WbVector2(1.0, 1.0)))
    return false;

  return true;
}

void WbPlane::rescale(const WbVector3 &scale) {
  WbVector2 resizedSize = size();
  if (scale.x() != 1.0)
    resizedSize[0] *= scale.x();
  if (scale.y() != 1.0)
    resizedSize[1] *= scale.y();
  setSize(resizedSize);
}

void WbPlane::updateSize() {
  if (!sanitizeFields())
    return;

  if (isInBoundingObject())
    updateLineScale();
  else
    updateScale();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbPlane::updateLineScale() {
  if (!isAValidBoundingObject())
    return;

  float offset = wr_config_get_line_scale() / LINE_SCALE_FACTOR;

  // allow the bounding sphere to scale down
  float scaleZ = 0.1f * std::min(mSize->value().x(), mSize->value().y());

  float scale[] = {static_cast<float>(mSize->value().x() * (1.0f + offset)),
                   static_cast<float>(mSize->value().y() * (1.0f + offset)), scaleZ};
  wr_transform_set_scale(wrenNode(), scale);
}

void WbPlane::updateScale() {
  // allow the bounding sphere to scale down
  float scaleZ = 0.1f * std::min(mSize->value().x(), mSize->value().y());

  float scale[] = {static_cast<float>(mSize->value().x()), static_cast<float>(mSize->value().y()), scaleZ};
  wr_transform_set_scale(wrenNode(), scale);
}

bool WbPlane::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidDimensions = (mSize->x() <= 0.0 || mSize->y() <= 0.0);
  if (warning && invalidDimensions)
    parsingWarn(tr("All 'size' components must be positive for a Plane used in a 'boundingObject'."));

  return !invalidDimensions;
}

/////////////////
// ODE objects //
/////////////////

dGeomID WbPlane::createOdeGeom(dSpaceID space) {
  double d;
  WbVector3 n;
  computePlaneParams(n, d);
  return dCreatePlane(space, n.x(), n.y(), n.z(), d);
}

bool WbPlane::isAValidBoundingObject(bool checkOde, bool warning) const {
  const bool admissible = WbGeometry::isAValidBoundingObject(checkOde, warning);
  return admissible && isSuitableForInsertionInBoundingObject(admissible && warning);
}

void WbPlane::setOdePosition(const WbVector3 &translation) {
  updateOdePlanePosition();
}

void WbPlane::setOdeRotation(const WbMatrix3 &matrix) {
  updateOdePlanePosition();
}

void WbPlane::updateOdePlanePosition() {
  WbVector3 n;
  double d;
  computePlaneParams(n, d);
  dGeomPlaneSetParams(mOdeGeom, n.x(), n.y(), n.z(), d);
}

void WbPlane::computePlaneParams(WbVector3 &n, double &d) {
  WbTransform *transform = upperTransform();

  // initial values with identity matrices
  n.setXyz(0.0, 0.0, 1.0);  // plane normal

  if (transform) {
    const WbMatrix3 &m3 = transform->rotationMatrix();
    // Applies this transform's rotation to plane normal
    n = m3 * n;

    // Computes the d parameter in the plane equation
    d = transform->position().dot(n);
  } else
    d = 0.0;
}

/////////////////
// Ray tracing //
/////////////////

bool WbPlane::pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet) const {
  WbVector3 collisionPoint;
  const bool intersectionExists = computeCollisionPoint(collisionPoint, ray);
  if (!intersectionExists)
    // no valid collision
    return false;

  // transform intersection point to plane coordinates
  WbVector3 pointOnTexture(collisionPoint);
  const WbTransform *const transform = upperTransform();
  if (transform) {
    pointOnTexture = transform->matrix().pseudoInversed(collisionPoint);
    pointOnTexture /= absoluteScale();
  }

  // transform point into texture coordinates in range [0..1]
  const double sx = scaledSize().x();
  const double sy = scaledSize().y();

  const double u = pointOnTexture.x() / sx + 0.5;
  const double v = -pointOnTexture.y() / sy + 0.5;

  // result
  uv.setXy(u, v);
  return true;
}

double WbPlane::computeDistance(const WbRay &ray) const {
  WbVector3 collisionPoint;
  const bool collisionExists = computeCollisionPoint(collisionPoint, ray);
  if (!collisionExists)
    // no valid collision
    return -1;

  // distance
  const WbVector3 &d = ray.origin() - collisionPoint;
  return d.length();
}

bool WbPlane::computeCollisionPoint(WbVector3 &point, const WbRay &ray) const {
  // 1. Compute the 4 plane vertices in world coordinates.
  const double planeWidth = size().x();
  const double planeHeight = size().y();
  const WbMatrix4 &upperMatrix = upperTransform()->matrix();
  const WbVector3 p1 = upperMatrix * WbVector3(0.5 * planeWidth, -0.5 * planeHeight, 0.0);
  const WbVector3 p2 = upperMatrix * WbVector3(0.5 * planeWidth, 0.5 * planeHeight, 0.0);
  const WbVector3 p3 = upperMatrix * WbVector3(-0.5 * planeWidth, 0.5 * planeHeight, 0.0);
  const WbVector3 p4 = upperMatrix * WbVector3(-0.5 * planeWidth, -0.5 * planeHeight, 0.0);

  // 2. Check if the ray intersects one of the two oriented triangle.
  // Compute the intersection point in such case.
  double u, v;
  const std::pair<bool, double> intersection1 = ray.intersects(p1, p2, p3, true, u, v);
  if (intersection1.first && intersection1.second > 0.0) {
    point = ray.origin() + intersection1.second * ray.direction();
    return true;
  }

  const std::pair<bool, double> intersection2 = ray.intersects(p1, p3, p4, true, u, v);
  if (intersection2.first && intersection2.second > 0.0) {
    point = ray.origin() + intersection2.second * ray.direction();
    return true;
  }

  // 3. The ray does not intersect the plane.
  return false;
}

void WbPlane::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->set(WbVector3(), scaledSize().length() / 2.0);
}

////////////////////////
// Friction Direction //
////////////////////////

WbVector3 WbPlane::computeFrictionDirection(const WbVector3 &normal) const {
  return WbVector3(1, 0, 0);
}
