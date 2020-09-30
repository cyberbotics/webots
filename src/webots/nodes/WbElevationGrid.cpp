// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "WbElevationGrid.hpp"

#include "WbAffinePlane.hpp"
#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbLog.hpp"
#include "WbMFDouble.hpp"
#include "WbMatter.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeGeomData.hpp"
#include "WbRay.hpp"
#include "WbResizeManipulator.hpp"
#include "WbRgb.hpp"
#include "WbSFBool.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSimulationState.hpp"
#include "WbTransform.hpp"
#include "WbWrenMeshBuffers.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <ode/ode.h>

void WbElevationGrid::init() {
  mHeightfieldData = NULL;
  mData = NULL;
  mMinHeight = 0;
  mMaxHeight = 0;

  mHeight = findMFDouble("height");
  mXDimension = findSFInt("xDimension");
  mXSpacing = findSFDouble("xSpacing");
  mZDimension = findSFInt("zDimension");
  mZSpacing = findSFDouble("zSpacing");
  mThickness = findSFDouble("thickness");
}

WbElevationGrid::WbElevationGrid(WbTokenizer *tokenizer) : WbGeometry("ElevationGrid", tokenizer) {
  init();

  if (tokenizer == NULL) {
    mXDimension->setValueNoSignal(2);
    mZDimension->setValueNoSignal(2);
  }
}

WbElevationGrid::WbElevationGrid(const WbElevationGrid &other) : WbGeometry(other) {
  init();
}

WbElevationGrid::WbElevationGrid(const WbNode &other) : WbGeometry(other) {
  init();
}

WbElevationGrid::~WbElevationGrid() {
  if (mHeightfieldData)
    dGeomHeightfieldDataDestroy(mHeightfieldData);

  delete[] mData;

  wr_static_mesh_delete(mWrenMesh);
}

void WbElevationGrid::preFinalize() {
  WbGeometry::preFinalize();

  sanitizeFields();

  if (isInBoundingObject()) {
    if (WbNodeUtilities::findUpperMatter(this)->nodeType() == WB_NODE_FLUID) {
      parsingWarn("The ElevationGrid geometry cannot be used as a Fluid boundingObject. Immersions will have not effect.\n");
      // TODO: enable dHeightField for immersion detection in src/ode/ode/fluid_dynamics
    }
    isSuitableForInsertionInBoundingObject(true);  // boundingObject specific warnings
  }
}

void WbElevationGrid::postFinalize() {
  WbGeometry::postFinalize();
  connect(mHeight, &WbMFDouble::changed, this, &WbElevationGrid::updateHeight);
  connect(mXDimension, &WbSFInt::changed, this, &WbElevationGrid::updateXDimension);
  connect(mXSpacing, &WbSFDouble::changed, this, &WbElevationGrid::updateXSpacing);
  connect(mZDimension, &WbSFInt::changed, this, &WbElevationGrid::updateZDimension);
  connect(mZSpacing, &WbSFDouble::changed, this, &WbElevationGrid::updateZSpacing);
  connect(mThickness, &WbSFDouble::changed, this, &WbElevationGrid::updateThickness);
}

void WbElevationGrid::createWrenObjects() {
  WbGeometry::createWrenObjects();
  buildWrenMesh();

  if (isInBoundingObject())
    connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this,
            &WbElevationGrid::updateLineScale);

  emit wrenObjectsCreated();
}

void WbElevationGrid::buildWrenMesh() {
  const bool resizeManipulator = mResizeManipulator && mResizeManipulator->isAttached();
  WbGeometry::deleteWrenRenderable();

  wr_static_mesh_delete(mWrenMesh);
  mWrenMesh = NULL;

  if (xDimension() < 2 || zDimension() < 2)
    return;

  if (xSpacing() == 0.0 || zSpacing() == 0.0)
    return;

  WbGeometry::computeWrenRenderable();

  // Reattach resize manipulator
  if (mResizeManipulator) {
    mResizeManipulator->attachTo(mWrenScaleTransform);
    if (resizeManipulator)
      mResizeManipulator->show();
  }

  // Restore pickable state
  setPickable(isPickable());

  // convert height values to float, pad with zeroes if necessary
  int numValues = xDimension() * zDimension();
  float *heightData = new float[numValues];

  int availableValues = std::min(numValues, mHeight->size());
  for (int i = 0; i < availableValues; ++i)
    heightData[i] = mHeight->item(i);

  int remainingValues = numValues - availableValues;
  memset(&heightData[availableValues], 0, remainingValues * sizeof(float));

  const bool createOutlineMesh = isInBoundingObject();

  mWrenMesh =
    wr_static_mesh_unit_elevation_grid_new(xDimension(), zDimension(), heightData, mThickness->value(), createOutlineMesh);

  delete[] heightData;

  // This must be done after WbGeometry::computeWrenRenderable() otherwise
  // the outline scaling is applied to the wrong WREN transform
  if (createOutlineMesh)
    updateLineScale();
  else
    updateScale();

  wr_renderable_set_mesh(mWrenRenderable, WR_MESH(mWrenMesh));
}

void WbElevationGrid::rescale(const WbVector3 &scale) {
  if (scale.x() != 1.0) {
    // rescale x spacing
    setXspacing(xSpacing() * scale.x());
  }

  if (scale.y() != 0.0) {
    // rescale height
    setHeightScaleFactor(scale.y());
  }

  if (scale.z() != 0.0) {
    // rescale z spacing
    setZspacing(zSpacing() * scale.z());
  }
}

bool WbElevationGrid::sanitizeFields() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mThickness, 0.0))
    return false;

  if (WbFieldChecker::resetIntIfNegative(this, mXDimension, 0))
    return false;

  if (WbFieldChecker::resetDoubleIfNonPositive(this, mXSpacing, 1.0))
    return false;

  if (WbFieldChecker::resetIntIfNegative(this, mZDimension, 0))
    return false;

  if (WbFieldChecker::resetDoubleIfNonPositive(this, mZSpacing, 1.0))
    return false;

  checkHeight();

  return true;
}

void WbElevationGrid::checkHeight() {
  const int xd = mXDimension->value();
  const int zd = mZDimension->value();
  const int xdzd = xd * zd;

  const int extra = mHeight->size() - xdzd;
  if (extra > 0)
    parsingWarn(tr("'height' contains %1 ignored extra value(s).").arg(extra));

  // find min/max height
  mMinHeight = 0;
  mMaxHeight = 0;
  mHeight->findMinMax(&mMinHeight, &mMaxHeight);

  // adjust min/max height if height field is not complete
  if (mHeight->size() < xdzd) {
    if (mMinHeight > 0.0)
      mMinHeight = 0.0;
    if (mMaxHeight < 0.0)
      mMaxHeight = 0.0;
  }
}

void WbElevationGrid::updateHeight() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbElevationGrid::updateThickness() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbElevationGrid::updateXDimension() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbElevationGrid::updateXSpacing() {
  if (!sanitizeFields())
    return;

  if (isInBoundingObject())
    updateLineScale();
  else
    updateScale();

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbElevationGrid::updateZDimension() {
  if (!sanitizeFields())
    return;

  buildWrenMesh();

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbElevationGrid::updateZSpacing() {
  if (!sanitizeFields())
    return;

  if (isInBoundingObject())
    updateLineScale();
  else
    updateScale();

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void WbElevationGrid::updateLineScale() {
  if (!isAValidBoundingObject())
    return;

  const float offset = wr_config_get_line_scale() / LINE_SCALE_FACTOR;

  float scale[] = {static_cast<float>(xSpacing()), 1.0f + offset, static_cast<float>(zSpacing())};

  wr_transform_set_scale(wrenNode(), scale);
}

void WbElevationGrid::updateScale() {
  float scale[] = {static_cast<float>(xSpacing()), static_cast<float>(1.0f), static_cast<float>(zSpacing())};
  wr_transform_set_scale(wrenNode(), scale);
}

void WbElevationGrid::createResizeManipulator() {
  mResizeManipulator =
    new WbRegularResizeManipulator(uniqueId(), WbWrenAbstractResizeManipulator::ResizeConstraint::NO_CONSTRAINT);
}

void WbElevationGrid::setResizeManipulatorDimensions() {
  WbVector3 scale(xSpacing(), 1.0f, zSpacing());
  WbTransform *transform = upperTransform();
  if (transform)
    scale *= transform->matrix().scale();

  if (isAValidBoundingObject())
    scale *= WbVector3(1.0f, 1.0f + (wr_config_get_line_scale() / LINE_SCALE_FACTOR), 1.0f);

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

bool WbElevationGrid::areSizeFieldsVisibleAndNotRegenerator() const {
  const WbField *const xSpacing = findField("xSpacing", true);
  const WbField *const zSpacing = findField("zSpacing", true);
  return WbNodeUtilities::isVisible(xSpacing) && WbNodeUtilities::isVisible(zSpacing) &&
         !WbNodeUtilities::isTemplateRegeneratorField(xSpacing) && !WbNodeUtilities::isTemplateRegeneratorField(zSpacing);
}

/////////////////
// ODE objects //
/////////////////

dGeomID WbElevationGrid::createOdeGeom(dSpaceID space) {
  if (setOdeHeightfieldData() == false)
    return NULL;

  // Creates a height field with without dSpace.
  // We need to translate the height field because in VRML the coordinate center is located in
  // one corner of the grid, while in ODE, it is located in the middle of the grid.
  mLocalOdeGeomOffsetPosition = WbVector3(scaledWidth() / 2.0, 0.0, scaledDepth() / 2.0);
  return dCreateHeightfield(space, mHeightfieldData, true);
}

bool WbElevationGrid::setOdeHeightfieldData() {
  if (isAValidBoundingObject(false, false) == false)
    return false;

  const int xd = mXDimension->value();
  const int zd = mZDimension->value();
  const int xdzd = xd * zd;
  // Creates height field data
  delete[] mData;
  mData = new double[xdzd];
  memset(mData, 0, xdzd * sizeof(double));
  mHeight->copyItemsTo(mData, xdzd);
  if (mHeightfieldData == NULL)
    mHeightfieldData = dGeomHeightfieldDataCreate();
  dGeomHeightfieldDataBuildDouble(mHeightfieldData, mData, false, scaledWidth(), scaledDepth(), xd, zd, heightScaleFactor(),
                                  0.0, mThickness->value(), false);

  // This should improve performance and allow the heightmap to be rotated
  dGeomHeightfieldDataSetBounds(mHeightfieldData, mMinHeight, mMaxHeight);
  return true;
}

void WbElevationGrid::applyToOdeData(bool correctSolidMass) {
  if (setOdeHeightfieldData() == false)
    return;

  if (mOdeGeom == NULL) {
    if (areOdeObjectsCreated())
      emit validElevationGridInserted();
    return;
  }

  assert(dGeomGetClass(mOdeGeom) == dHeightfieldClass);

  dGeomHeightfieldSetHeightfieldData(mOdeGeom, mHeightfieldData);
  WbOdeGeomData *const odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(mOdeGeom));
  assert(odeGeomData);
  odeGeomData->setLastChangeTime(WbSimulationState::instance()->time());
  mLocalOdeGeomOffsetPosition = WbVector3(scaledWidth() / 2.0, 0.0, scaledDepth() / 2.0);
}

double WbElevationGrid::scaledWidth() const {
  return fabs(absoluteScale().x() * width());
}

double WbElevationGrid::scaledDepth() const {
  return fabs(absoluteScale().z() * depth());
}

bool WbElevationGrid::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidDimensions = mXDimension->value() < 2 || mZDimension->value() < 2;
  const bool invalidSpacings = mXSpacing->value() <= 0.0 || mZSpacing->value() < 0.0;
  const bool invalid = invalidDimensions || invalidSpacings;

  if (warning) {
    if (mXDimension->value() < 2)
      parsingWarn(tr("Invalid 'xDimension' (should be greater than 1) for use in boundingObject."));

    if (mZDimension->value() < 2)
      parsingWarn(tr("Invalid 'zDimension' (should be greater than 1) for use in boundingObject."));

    if (invalidSpacings)
      parsingWarn(tr("'height' must be positive when used in a 'boundingObject'."));

    if (invalid)
      parsingWarn(tr("Cannot create the associated physics object."));
  }

  return !invalid;
}

bool WbElevationGrid::isAValidBoundingObject(bool checkOde, bool warning) const {
  return isSuitableForInsertionInBoundingObject(warning && isInBoundingObject()) &&
         WbGeometry::isAValidBoundingObject(checkOde, warning);
}

/////////////////
// Ray Tracing //
/////////////////

bool WbElevationGrid::pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet) const {
  WbVector3 localCollisionPoint;
  const double collisionDistance = computeLocalCollisionPoint(ray, localCollisionPoint);
  if (collisionDistance < 0)
    return false;

  const double sizeX = scaledWidth();
  const double sizeZ = scaledDepth();

  const double u = (double)localCollisionPoint.x() / sizeX;
  const double v = (double)localCollisionPoint.z() / sizeZ;

  // result
  uv.setXy(u, v);
  return true;
}

double WbElevationGrid::computeDistance(const WbRay &ray) const {
  WbVector3 localCollisionPoint;
  return computeLocalCollisionPoint(ray, localCollisionPoint);
}

double WbElevationGrid::computeLocalCollisionPoint(const WbRay &ray, WbVector3 &localCollisionPoint) const {
  double dx = mXSpacing->value() * absoluteScale().x();
  double dz = mZSpacing->value() * absoluteScale().z();
  int numX = mXDimension->value();
  int numZ = mZDimension->value();
  double minDistance = std::numeric_limits<double>::infinity();

  int size = numX * numZ;
  double *data = new double[size];
  memset(data, 0, size * sizeof(double));
  mHeight->copyItemsTo(data, size);

  WbRay localRay(ray);
  WbTransform *transform = upperTransform();
  if (transform) {
    localRay.setDirection(ray.direction() * transform->matrix());
    WbVector3 origin = transform->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
    localRay.setOrigin(origin);
    localRay.normalize();
  }

  for (int z = 0; z < (numZ - 1); z++) {
    for (int x = 0; x < (numX - 1); x++) {
      WbVector3 vertexA(x * dx, data[z * numX + x] * absoluteScale().y(), z * dz);
      WbVector3 vertexB(x * dx, data[(z + 1) * numX + x] * absoluteScale().y(), (z + 1) * dz);
      WbVector3 vertexC((x + 1) * dx, data[z * numX + x + 1] * absoluteScale().y(), z * dz);
      WbVector3 vertexD((x + 1) * dx, data[(z + 1) * numX + x + 1] * absoluteScale().y(), (z + 1) * dz);

      // first triangle: ABC
      WbAffinePlane plane(vertexA, vertexB, vertexC);
      plane.normalize();
      std::pair<bool, double> result = localRay.intersects(plane, true);
      if (result.first && result.second > 0 && result.second < minDistance) {
        // check finite plane bounds
        WbVector3 p = localRay.origin() + result.second * localRay.direction();
        if (p.x() >= vertexA.x() && p.x() <= vertexC.x() && p.z() >= vertexA.z() && p.z() <= vertexB.z()) {
          minDistance = result.second;
          localCollisionPoint = p;
        }
      }

      // second triangle: BCD
      plane = WbAffinePlane(vertexD, vertexC, vertexB);
      plane.normalize();
      result = localRay.intersects(plane, true);

      if (result.first && result.second > 0 && result.second < minDistance) {
        // check finite plane bounds
        WbVector3 p = localRay.origin() + result.second * localRay.direction();
        if (p.x() >= vertexB.x() && p.x() <= vertexC.x() && p.z() >= vertexC.z() && p.z() <= vertexD.z()) {
          minDistance = result.second;
          localCollisionPoint = p;
        }
      }
    }
  }

  delete[] data;

  if (minDistance == std::numeric_limits<double>::infinity())
    return -1;

  return minDistance;
}

void WbElevationGrid::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();
  if (mHeight->size() == 0)
    return;

  // create list of vertices
  const int xd = mXDimension->value();
  const int zd = mZDimension->value();
  const double xs = mXSpacing->value();
  const double zs = mZSpacing->value();
  const int size = zd * xd;
  double *h = new double[size];
  memset(h, 0, size * sizeof(double));
  mHeight->copyItemsTo(h, size);
  WbVector3 *vertices = new WbVector3[size];
  int index = 0;
  double posZ = 0.0;
  for (int z = 0; z < xd; z++, posZ += zs) {
    double posX = 0.0;
    for (int x = 0; x < zd; x++, posX += xs) {
      vertices[index] = WbVector3(posX, h[index], posZ);
      ++index;
    }
  }
  delete[] h;

  // Ritter's bounding sphere approximation
  // (see description in WbIndexedFaceSet::recomputeBoundingSphere)
  WbVector3 p2(vertices[0]);
  WbVector3 p1;
  double maxDistance;  // squared distance
  for (int i = 0; i < 2; ++i) {
    maxDistance = 0.0;
    p1 = p2;
    for (int j = 0; j < size; ++j) {
      const double d = p1.distance2(vertices[j]);
      if (d > maxDistance) {
        maxDistance = d;
        p2 = vertices[j];
      }
    }
  }
  mBoundingSphere->set((p2 + p1) * 0.5, sqrt(maxDistance) * 0.5);

  for (int j = 0; j < size; ++j)
    mBoundingSphere->enclose(vertices[j]);

  delete[] vertices;
}

void WbElevationGrid::exportNodeFields(WbVrmlWriter &writer) const {
  if (writer.isWebots()) {
    WbGeometry::exportNodeFields(writer);
    return;
  }

  findField("xDimension", true)->write(writer);
  findField("zDimension", true)->write(writer);
  findField("xSpacing", true)->write(writer);
  findField("zSpacing", true)->write(writer);
  if (!mHeight->isEmpty())
    findField("height", true)->write(writer);
  else {
    int total = mXDimension->value() * mZDimension->value();
    if (writer.isX3d())
      writer << " height=\'";
    else {
      writer.indent();
      writer << "height [ ";
    }
    for (int i = 0; i < total; i++) {
      if (i != 0)
        writer << " ";
      writer << "0";
    }
    if (writer.isX3d())
      writer << "\'";
    else
      writer << " ]\n";
  }
}

////////////////////////
// Friction Direction //
////////////////////////

WbVector3 WbElevationGrid::computeFrictionDirection(const WbVector3 &normal) const {
  parsingWarn(tr("A ElevationGrid is used in a Bounding object using an asymmetric friction. ElevationGrid does not support "
                 "asymmetric friction"));
  return WbVector3(0, 0, 0);
}
