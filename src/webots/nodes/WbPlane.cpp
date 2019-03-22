// Copyright 1996-2019 Cyberbotics Ltd.
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
#include "WbVrmlWriter.hpp"
#include "WbWrenAbstractResizeManipulator.hpp"
#include "WbWrenRenderingContext.hpp"

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
  const WbVector2 &size = mSize->value();
  const WbVector3 &scale = absoluteScale();
  return WbVector2(fabs(scale.x() * size.x()), fabs(scale.z() * size.y()));
}

void WbPlane::write(WbVrmlWriter &writer) const {
  if (writer.isWebots())
    WbGeometry::write(writer);
  else
    writeExport(writer);
}

void WbPlane::exportNodeFields(WbVrmlWriter &writer) const {
  if (writer.isWebots())
    WbGeometry::exportNodeFields(writer);
  else if (writer.isX3d())
    writer << " coordIndex=\'0 1 2 3 -1\' texCoordIndex=\'0 1 2 3 -1\'";
  else {  // VRML
    writer.indent();
    writer << "coordIndex [ 0 1 2 3 -1 ]\n";
    writer.indent();
    writer << "texCoordIndex [ 0 1 2 3 -1 ]\n";
  }
}

void WbPlane::exportNodeSubNodes(WbVrmlWriter &writer) const {
  double sx = mSize->value().x() / 2.0;
  double sz = mSize->value().y() / 2.0;
  if (writer.isWebots())
    WbGeometry::exportNodeSubNodes(writer);
  else if (writer.isX3d()) {
    writer << "<Coordinate point=\'";
    writer << -sx << " 0 " << -sz << ", ";
    writer << -sx << " 0 " << sz << ", ";
    writer << sx << " 0 " << sz << ", ";
    writer << sx << " 0 " << -sz << "\'></Coordinate>";
    writer << "<TextureCoordinate point=\'0 1, 0 0, 1 0, 1 1\'></TextureCoordinate>";
  } else {  // VRML
    writer.indent();
    writer << "coord Coordinate {\n";
    writer.increaseIndent();
    writer.indent();
    writer << "point [ ";
    writer << -sx << " 0 " << -sz << ", ";
    writer << -sx << " 0 " << sz << ", ";
    writer << sx << " 0 " << sz << ", ";
    writer << sx << " 0 " << -sz << " ]\n";
    writer.decreaseIndent();
    writer.indent();
    writer << "}\n";
    writer.indent();
    writer << "texCoord TextureCoordinate {\n";
    writer.increaseIndent();
    writer.indent();
    writer << "point [ 0 1, 0 0, 1 0, 1 1]\n";
    writer.decreaseIndent();
    writer.indent();
    writer << "}\n";
  }
}

void WbPlane::createWrenObjects() {
  WbGeometry::createWrenObjects();
  WbGeometry::computeWrenRenderable();

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
  WbVector3 scale(size().x(), 0.1f * std::min(mSize->value().x(), mSize->value().y()), size().y());
  WbTransform *transform = upperTransform();
  if (transform)
    scale *= transform->matrix().scale();

  if (isAValidBoundingObject()) {
    float offset = 1.0f + (wr_config_get_line_scale() / LINE_SCALE_FACTOR);
    scale *= WbVector3(offset, 1.0f, offset);
  }

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

bool WbPlane::areSizeFieldsVisibleAndNotRegenerator() const {
  const WbField *const size = findField("size", true);
  return WbNodeUtilities::isVisible(size) && !WbNodeUtilities::isTemplateRegeneratorField(size);
}

bool WbPlane::sanitizeFields() {
  if (WbFieldChecker::checkVector2IsPositive(this, mSize, WbVector2(1.0, 1.0)))
    return false;

  return true;
}

void WbPlane::rescale(const WbVector3 &scale) {
  WbVector2 resizedSize = size();
  if (scale.x() != 1.0)
    resizedSize[0] *= scale.x();
  if (scale.z() != 1.0)
    resizedSize[1] *= scale.z();
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
  if (!sanitizeFields() || !isAValidBoundingObject())
    return;

  float offset = wr_config_get_line_scale() / LINE_SCALE_FACTOR;

  // allow the bounding sphere to scale down
  float scaleY = 0.1f * std::min(mSize->value().x(), mSize->value().y());

  float scale[] = {static_cast<float>(mSize->value().x() * (1.0f + offset)), scaleY,
                   static_cast<float>(mSize->value().y() * (1.0f + offset))};
  wr_transform_set_scale(wrenNode(), scale);
}

void WbPlane::updateScale() {
  if (!sanitizeFields())
    return;

  // allow the bounding sphere to scale down
  float scaleY = 0.1f * std::min(mSize->value().x(), mSize->value().y());

  float scale[] = {static_cast<float>(mSize->value().x()), scaleY, static_cast<float>(mSize->value().y())};
  wr_transform_set_scale(wrenNode(), scale);
}

bool WbPlane::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidDimensions = (mSize->x() <= 0.0 || mSize->y() <= 0.0);
  if (warning && invalidDimensions)
    warn(tr("All 'size' components must be positive for a Plane used in a 'boundingObject'."));

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

void WbPlane::setOdeRotation(const WbRotation &rotation) {
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
  n.setXyz(0.0, 1.0, 0.0);     // plane normal
  WbVector3 p(0.0, 0.0, 0.0);  // a point in the plane

  if (transform) {
    const WbMatrix3 &m3 = transform->rotationMatrix();
    // Applies this transform's rotation to plane normal
    n = m3 * n;

    // Translates p
    p = transform->position();
    // Computes the d parameter in the plane equation
    d = p.dot(n);
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
  const double sz = scaledSize().y();

  const double u = pointOnTexture.x() / sx + 0.5;
  const double v = pointOnTexture.z() / sz + 0.5;

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
  // compute intersection point between ray and plane
  WbVector3 planeNormal(0.0, 1.0, 0.0);
  WbVector3 translation(0.0, 0.0, 0.0);
  const WbTransform *const transform = upperTransform();
  if (transform) {
    planeNormal = transform->matrix().sub3x3MatrixDot(WbVector3(0.0, 1.0, 0.0));
    planeNormal.normalize();
    translation = transform->matrix().translation();
  }
  const WbAffinePlane plane(planeNormal, translation);
  const std::pair<bool, double> intersection = ray.intersects(plane, true);

  if (!intersection.first || intersection.second < 0.0)
    // collision not in the direction of the ray or no intersection
    return false;

  // intersection point
  point = ray.origin() + intersection.second * ray.direction();
  return true;
}

void WbPlane::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->set(WbVector3(), scaledSize().length() / 2.0);
}

////////////////////////
// Friction Direction //
////////////////////////

WbVector3 WbPlane::computeFrictionDirection(const WbVector3 &normal) const {
  return WbVector3(0, 0, 1);
}
