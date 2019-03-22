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

#include "WbCapsule.hpp"

#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbMatter.hpp"
#include "WbNodeUtilities.hpp"
#include "WbRay.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSFBool.hpp"
#include "WbSFInt.hpp"
#include "WbSimulationState.hpp"
#include "WbTransform.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <ode/ode.h>
#include <cmath>

void WbCapsule::init() {
  // rotate capsule by 90 degrees around the x-axis because ODE capsules
  // are z-aligned but Webots needs the Caspules to be y-aligned
  mIs90DegreesRotated = true;

  mBottom = findSFBool("bottom");
  mRadius = findSFDouble("radius");
  mHeight = findSFDouble("height");
  mSide = findSFBool("side");
  mTop = findSFBool("top");
  mSubdivision = findSFInt("subdivision");

  mResizeConstraint = WbWrenAbstractResizeManipulator::X_EQUAL_Z;
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

  if (isInBoundingObject())
    connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this, &WbCapsule::updateLineScale);

  buildWrenMesh();

  emit wrenObjectsCreated();
}

void WbCapsule::setResizeManipulatorDimensions() {
  WbVector3 scale(1.0f, 1.0f, 1.0f);

  WbTransform *transform = upperTransform();
  if (transform)
    scale *= transform->matrix().scale();

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
  const WbField *const height = findField("height", true);
  const WbField *const radius = findField("radius", true);
  return WbNodeUtilities::isVisible(height) && WbNodeUtilities::isVisible(radius) &&
         !WbNodeUtilities::isTemplateRegeneratorField(height) && !WbNodeUtilities::isTemplateRegeneratorField(radius);
}

bool WbCapsule::sanitizeFields() {
  if (WbFieldChecker::checkIntInRangeWithIncludedBounds(this, mSubdivision, 4, 1000, 4))
    return false;

  if (WbFieldChecker::checkDoubleIsPositive(this, mRadius, 1.0))
    return false;

  if (WbFieldChecker::checkDoubleIsPositive(this, mHeight, 1.0))
    return false;

  return true;
}

void WbCapsule::buildWrenMesh() {
  const bool resizeManipulator = mResizeManipulator && mResizeManipulator->isAttached();
  WbGeometry::deleteWrenRenderable();

  wr_static_mesh_delete(mWrenMesh);
  mWrenMesh = NULL;

  if (!sanitizeFields())
    return;

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

  mWrenMesh = wr_static_mesh_capsule_new(mSubdivision->value(), mRadius->value(), mHeight->value(), mSide->isTrue(),
                                         mTop->isTrue(), mBottom->isTrue(), createOutlineMesh);

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
  if (!sanitizeFields() || !isAValidBoundingObject())
    return;

  float offset = wr_config_get_line_scale() / LINE_SCALE_FACTOR;

  float scale[] = {static_cast<float>(1.0f + offset), static_cast<float>(1.0f + offset), static_cast<float>(1.0f + offset)};
  wr_transform_set_scale(wrenNode(), scale);
}

/////////////////
// ODE objects //
/////////////////

dGeomID WbCapsule::createOdeGeom(dSpaceID space) {
  if (mRadius->value() <= 0.0) {
    warn(tr("'radius' must be positive when used in a 'boundingObject'."));
    return NULL;
  }

  if (mHeight->value() <= 0.0) {
    warn(tr("'height' must be positive when used in a 'boundingObject'."));
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

  if (correctSolidMass)
    applyToOdeMass();
}

double WbCapsule::scaledRadius() const {
  const WbVector3 &scale = absoluteScale();
  return fabs(mRadius->value() * std::max(scale.x(), scale.z()));
}

double WbCapsule::scaledHeight() const {
  return fabs(mHeight->value() * absoluteScale().y());
}

bool WbCapsule::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidRadius = mRadius->value() <= 0.0;
  const bool invalidHeight = mHeight->value() <= 0.0;
  if (warning) {
    if (invalidRadius)
      warn(tr("'radius' must be positive when used in a 'boundingObject'."));

    if (invalidHeight)
      warn(tr("'height' must be positive when used in a 'boundingObject'."));
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

  const double theta = atan2(localCollisionPoint.x(), localCollisionPoint.z()) + M_PI;
  const double u = 0.5 * theta / M_PI;

  // default: offset of the top half sphere
  double v = 0.0;
  const double h = scaledHeight();
  const double h2 = 0.5 * h;
  const double r = scaledRadius();
  const double absY = fabs(localCollisionPoint.y());
  if (absY <= h2) {
    // body
    v = (2.0 - (localCollisionPoint.y() + h2) / h) / 3.0;

  } else {
    // top and bottom half sphere

    double yIi;
    const double yI = absY - h2;
    const int sub4 = 0.25 * mSubdivision->value();
    const int sub5 = sub4 + 1;
    double prevD = 0;
    const int p = 3 * sub4;
    const double factor4 = M_PI_2 / sub4;
    for (int i = 1; i < sub5; i++) {
      double alpha = factor4 * i;
      double d = r * sin(alpha);
      if (yI < d) {
        yIi = yI - prevD;
        v = (double)(i - 1 + 2 * sub4) / p + yIi / (p * (d - prevD));
        break;
      }

      prevD = d;
    }

    // if top half sphere
    if (localCollisionPoint.y() > 0)
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
  WbTransform *transform = upperTransform();
  if (transform) {
    direction = ray.direction() * transform->matrix();
    direction.normalize();
    origin = transform->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
  }

  const double r = scaledRadius();
  const double halfH = scaledHeight() * 0.5;
  double d = std::numeric_limits<double>::infinity();

  // distance from cylinder body
  if (mSide->value()) {
    const double a = direction.x() * direction.x() + direction.z() * direction.z();
    const double b = 2.0 * (origin.x() * direction.x() + origin.z() * direction.z());
    const double c = origin.x() * origin.x() + origin.z() * origin.z() - r * r;
    double discriminant = b * b - 4.0 * a * c;

    // if c < 0: ray origin is inside the cylinder body
    if (c >= 0 && discriminant > 0.0) {
      discriminant = sqrt(discriminant);
      const double t1 = (-b - discriminant) / (2 * a);
      const double t2 = (-b + discriminant) / (2 * a);
      const double y1 = origin.y() + t1 * direction.y();
      const double y2 = origin.y() + t2 * direction.y();
      if (t1 > 0.0 && y1 >= -halfH && y1 <= halfH)
        d = t1;
      else if (t2 > 0.0 && y2 >= -halfH && y2 <= halfH)
        d = t2;
    }
  }

  // distance with top half sphere
  if (mTop->value()) {
    std::pair<bool, double> intersection = WbRay(origin, direction).intersects(WbVector3(0, halfH, 0), r, true);
    if (intersection.first && intersection.second > 0 && intersection.second < d) {
      double y = origin.y() + intersection.second * direction.y();
      if (y >= halfH)
        d = intersection.second;
    }
  }

  // distance with bottom half spheres
  if (mBottom->value()) {
    std::pair<bool, double> intersection = WbRay(origin, direction).intersects(WbVector3(0, -halfH, 0), r, true);
    if (intersection.first && intersection.second > 0 && intersection.second < d) {
      double y = origin.y() + intersection.second * direction.y();
      if (y <= -halfH)
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
  const double halfHeight = scaledHeight() / 2.0;
  const double radius = scaledRadius();

  if (!top && !side && !bottom) {  // it is empty
    mBoundingSphere->empty();
    return;
  }

  if (top + side + bottom == 1) {
    if (top || bottom)
      mBoundingSphere->set(WbVector3(0, top ? halfHeight : -halfHeight, 0), radius);
    else  // side
      mBoundingSphere->set(WbVector3(), WbVector3(radius, halfHeight, 0).length());
  } else if (top != bottom) {  // we have 'top and side' or 'side and bottom'
    const double maxY = top ? halfHeight + radius : halfHeight;
    const double minY = bottom ? -halfHeight - radius : -halfHeight;
    const double totalHeight = (maxY - minY);
    const double newRadius = totalHeight / 2.0 + radius * radius / (2 * totalHeight);
    const double offsetY = top ? (maxY - newRadius) : (minY + newRadius);
    mBoundingSphere->set(WbVector3(0, offsetY, 0), newRadius);
  } else  // complete capsule
    mBoundingSphere->set(WbVector3(), halfHeight + radius);
}

void WbCapsule::write(WbVrmlWriter &writer) const {
  if (writer.isWebots())
    WbGeometry::write(writer);
  else
    writeExport(writer);
}

void WbCapsule::exportNodeFields(WbVrmlWriter &writer) const {
  if (writer.isWebots()) {
    WbGeometry::exportNodeFields(writer);
    return;
  }

  int sub = mSubdivision->value();

  // coordIndex and creaseAngle
  if (writer.isX3d()) {
    writer << " creaseAngle=\'1\'";
    writer << " coordIndex=\'";
  } else {
    writer.indent();
    writer << "creaseAngle 1\n";
    writer.indent();
    writer << "coordIndex [ ";
  }
  int counter = 0;
  int sub4 = sub / 4;
  assert(sub4 >= 1);
  if (mTop->isTrue()) {
    counter++;
    // write top triangles
    for (int i = 0; i < sub; i++) {
      int j = i + 1;
      if (j == sub)
        j = 0;
      writer << "0 " << QString::number(counter + j) << " " << QString::number(counter + i) << " -1 ";
    }
    // write remaining quads
    for (int k = 1; k < sub4; k++) {
      for (int i = 0; i < sub; i++) {
        int j = i + 1;
        if (j == sub)
          j = 0;
        writer << QString::number(counter + i) << " " << QString::number(counter + j) << " "
               << QString::number(counter + sub + j) << " " << QString::number(counter + sub + i) << " -1 ";
      }
      counter += sub;
    }
  }
  if (mSide->isTrue()) {
    for (int i = 0; i < sub; i++) {
      int j = i + 1;
      if (j == sub)
        j = 0;
      writer << QString::number(counter + i) << " " << QString::number(counter + j) << " " << QString::number(counter + sub + j)
             << " " << QString::number(counter + sub + i) << " -1 ";
    }
  }
  if (mTop->isTrue() || mSide->isTrue())
    counter += sub;
  if (mBottom->isTrue()) {
    // write bottom quads
    for (int k = 1; k < sub4; k++) {
      for (int i = 0; i < sub; i++) {
        int j = i + 1;
        if (j == sub)
          j = 0;
        writer << QString::number(counter + i) << " " << QString::number(counter + j) << " "
               << QString::number(counter + sub + j) << " " << QString::number(counter + sub + i) << " -1 ";
      }
      counter += sub;
    }
    // write bottom triangles
    for (int i = 0; i < sub; i++) {
      int j = i + 1;
      if (j == sub)
        j = 0;
      writer << QString::number(counter + i) << " " << QString::number(counter + j) << " " << QString::number(counter + sub)
             << " -1 ";
    }
  }

  // texCoordIndex
  counter = 0;
  if (writer.isX3d()) {
    writer << "\' texCoordIndex=\'";
  } else {
    writer << "]\n";
    writer.indent();
    writer << "texCoordIndex [ ";
  }
  if (mTop->isTrue()) {
    // top triangles
    for (int i = 0; i < sub; i++)
      writer << i << " " << (counter + sub + i + 1) << " " << (counter + sub + i) << " -1 ";
    counter += sub;
    // write remaining quads
    for (int k = 1; k < sub4; k++) {
      for (int i = 0; i < sub; i++) {
        int j = i + 1;
        writer << counter + i << " " << counter + j << " " << counter + sub + 1 + j << " " << counter + sub + 1 + i << " -1 ";
      }
      counter += sub + 1;
    }
  }
  if (mSide->isTrue()) {
    for (int i = 0; i < sub; i++) {
      int j = i + 1;
      writer << counter + i << " " << counter + j << " " << counter + sub + 1 + j << " " << counter + sub + 1 + i << " -1 ";
    }
  }
  if (mTop->isTrue() || mSide->isTrue())
    counter += sub + 1;
  if (mBottom->isTrue()) {
    // bottom quads
    for (int k = 1; k < sub4; k++) {
      for (int i = 0; i < sub; i++) {
        int j = i + 1;
        writer << counter + i << " " << counter + j << " " << counter + sub + 1 + j << " " << counter + sub + 1 + i << " -1 ";
      }
      counter += sub + 1;
    }
    // bottom triangles
    for (int i = 0; i < sub; i++)
      writer << (counter + i) << " " << (counter + i + 1) << " " << counter + sub + 1 + i << " -1 ";
  }
  if (writer.isX3d()) {
    writer << "\'";
  } else {
    writer << "]\n";
  }
}

void WbCapsule::exportNodeSubNodes(WbVrmlWriter &writer) const {
  if (writer.isWebots()) {
    WbGeometry::exportNodeSubNodes(writer);
    return;
  }

  int sub = mSubdivision->value();
  int sub4 = sub / 4;
  double r = mRadius->value();
  double h2 = mHeight->value() / 2.0;
  double inc = 2.0 * M_PI / sub;
  double a;

  // Coordinate
  if (writer.isX3d()) {
    writer << "<Coordinate point=\'";
  } else {  // VRML
    writer.indent();
    writer << "coord Coordinate {\n";
    writer.increaseIndent();
    writer.indent();
    writer << "point [ ";
  }
  bool first = true;
  if (mTop->isTrue()) {
    writer << "0 " << h2 + r << " 0";
    first = false;
    for (int i = 1; i < sub4; i++) {
      double angle = M_PI_2 * (sub4 - i) / sub4;
      double dh = r * sin(angle);
      double dr = r * cos(angle);
      for (a = 0; a < 2.0 * M_PI - 0.0001; a += inc)
        writer << ", " << sin(a) * dr << " " << h2 + dh << " " << -cos(a) * dr;
    }
  }
  if (mSide->isTrue() || mTop->isTrue())
    for (a = 0; a < 2.0 * M_PI - 0.0001; a += inc) {
      if (!first)
        writer << ", ";
      first = false;
      writer << sin(a) * r << " " << h2 << " " << -cos(a) * r;
    }
  if (mSide->isTrue() || mBottom->isTrue())
    for (a = 0; a < 2.0 * M_PI - 0.0001; a += inc) {
      if (!first)
        writer << ", ";
      first = false;
      writer << sin(a) * r << " " << -h2 << " " << -cos(a) * r;
    }
  if (mBottom->isTrue()) {
    for (int i = 1; i < sub4; i++) {
      double angle = M_PI_2 * i / sub4;
      double dh = r * sin(angle);
      double dr = r * cos(angle);
      for (a = 0; a < 2.0 * M_PI - 0.0001; a += inc)
        writer << ", " << sin(a) * dr << " " << -h2 - dh << " " << -cos(a) * dr;
    }
    // last one
    writer << " 0 " << -h2 - r << " 0";
  }
  if (writer.isX3d()) {
    writer << "\'></Coordinate>";
  } else {
    writer << " ]\n";
    writer.decreaseIndent();
    writer.indent();
    writer << "}\n";
  }
  // TextureCoordinate
  if (writer.isX3d()) {
    writer << "<TextureCoordinate point=\'";
  } else {  // VRML
    writer.indent();
    writer << "texCoord TextureCoordinate {\n";
    writer.increaseIndent();
    writer.indent();
    writer << "point [ ";
  }
  first = true;
  if (mTop->isTrue()) {
    for (int i = 0; i < sub; i++) {
      if (!first)
        writer << ", ";
      first = false;
      writer << (-0.5 + sub - i) / sub << " 1";
    }
    for (int i = 1; i < sub4; i++) {
      double dh = (2.0 + ((double)(sub4 - i) / sub4)) / 3.0;
      for (int j = 0; j <= sub; j++)
        writer << ", " << (double)(sub - j) / sub << " " << dh;
    }
  }
  if (mSide->isTrue() || mTop->isTrue())
    for (int j = 0; j <= sub; j++) {
      if (!first)
        writer << ", ";
      first = false;
      writer << (double)(sub - j) / sub << " " << 2.0 / 3.0;
    }
  if (mSide->isTrue() || mBottom->isTrue())
    for (int j = 0; j <= sub; j++) {
      if (!first)
        writer << ", ";
      first = false;
      writer << (double)(sub - j) / sub << " " << 1.0 / 3.0;
    }
  if (mBottom->isTrue()) {
    for (int i = 1; i < sub4; i++) {
      double dh = (1.0 - ((double)i / sub4)) / 3.0;
      for (int j = 0; j <= sub; j++)
        writer << ", " << (double)(sub - j) / sub << " " << dh;
    }
    // bottom line
    for (int i = 0; i < sub; i++) {
      if (!first)
        writer << ", ";
      first = false;
      writer << (-0.5 + sub - i) / sub << " 0";
    }
  }
  if (writer.isX3d()) {
    writer << "\'></TextureCoordinate>";
  } else {
    writer << " ]\n";
    writer.decreaseIndent();
    writer.indent();
    writer << "}\n";
  }
}

////////////////////////
// Friction Direction //
////////////////////////

WbVector3 WbCapsule::computeFrictionDirection(const WbVector3 &normal) const {
  warn(tr("A Capsule is used in a Bounding object using an asymmetric friction. Capsule does not support asymmetric friction"));
  return WbVector3(0, 0, 0);
}
