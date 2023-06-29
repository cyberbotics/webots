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

#include "WbIndexedLineSet.hpp"

#include "WbBoundingSphere.hpp"
#include "WbCoordinate.hpp"
#include "WbField.hpp"
#include "WbMFInt.hpp"
#include "WbMFVector3.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSFNode.hpp"
#include "WbWrenMeshBuffers.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/config.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

void WbIndexedLineSet::init() {
  mCoord = findSFNode("coord");
  mCoordIndex = findMFInt("coordIndex");
}

WbIndexedLineSet::WbIndexedLineSet(WbTokenizer *tokenizer) : WbGeometry("IndexedLineSet", tokenizer) {
  init();
}

WbIndexedLineSet::WbIndexedLineSet(const WbIndexedLineSet &other) : WbGeometry(other) {
  init();
}

WbIndexedLineSet::WbIndexedLineSet(const WbNode &other) : WbGeometry(other) {
  init();
}

WbIndexedLineSet::~WbIndexedLineSet() {
  wr_static_mesh_delete(mWrenMesh);
}

void WbIndexedLineSet::postFinalize() {
  WbGeometry::postFinalize();

  connect(mCoord, &WbSFNode::changed, this, &WbIndexedLineSet::updateCoord);
  connect(mCoordIndex, &WbMFInt::changed, this, &WbIndexedLineSet::updateCoordIndex);

  if (coord())
    connect(coord(), &WbCoordinate::fieldChanged, this, &WbIndexedLineSet::updateCoord, Qt::UniqueConnection);
}

WbCoordinate *WbIndexedLineSet::coord() const {
  return static_cast<WbCoordinate *>(mCoord->value());
}

void WbIndexedLineSet::createWrenObjects() {
  WbGeometry::createWrenObjects();
  updateCoord();
  if (isInBoundingObject())
    connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this,
            &WbIndexedLineSet::updateLineScale);
  buildWrenMesh();
  emit wrenObjectsCreated();
}

void WbIndexedLineSet::updateLineScale() {
  if (!isAValidBoundingObject())
    return;

  float offset = wr_config_get_line_scale() / LINE_SCALE_FACTOR;

  float scale[] = {static_cast<float>(1.0 + offset), static_cast<float>(1.0 + offset), static_cast<float>(1.0 + offset)};
  wr_transform_set_scale(wrenNode(), scale);
}

void WbIndexedLineSet::setWrenMaterial(WrMaterial *material, bool castShadows) {
  WbGeometry::setWrenMaterial(material, castShadows);

  if (material)
    wr_material_set_default_program(material, WbWrenShaders::lineSetShader());
}

bool WbIndexedLineSet::sanitizeFields() {
  if (!coord() || coord()->point().isEmpty()) {
    parsingWarn(tr("A 'Coordinate' node should be present in the 'coord' field with at least two items."));
    return false;
  }

  if (mCoordIndex->isEmpty() || estimateIndexCount() < 2) {
    parsingWarn(tr("The 'coordIndex' field should have at least two items."));
    return false;
  }

  return true;
}

void WbIndexedLineSet::buildWrenMesh() {
  WbGeometry::deleteWrenRenderable();

  wr_static_mesh_delete(mWrenMesh);
  mWrenMesh = NULL;

  WbGeometry::computeWrenRenderable();

  wr_renderable_set_drawing_mode(mWrenRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);

  // In the worst case we end up with 2 * mCoordIndex->size() - 1 coordinates
  float *coordsData = new float[mCoordIndex->size() * 6];
  int coordsCount = computeCoordsData(coordsData);

  if (coordsCount > 0) {
    mWrenMesh = wr_static_mesh_line_set_new(coordsCount, coordsData, NULL);
    wr_renderable_set_mesh(mWrenRenderable, WR_MESH(mWrenMesh));
  }

  delete[] coordsData;
}

void WbIndexedLineSet::reset(const QString &id) {
  WbGeometry::reset(id);

  WbNode *const c = mCoord->value();
  if (c)
    c->reset(id);
}

int WbIndexedLineSet::computeCoordsData(float *data) {
  WbMFInt::Iterator it(*mCoordIndex);
  if (!it.hasNext())
    return 0;
  int i = it.next();
  int count = 0;
  WbVector3 v;
  QStringList invalidIndices;
  const int size = coord()->point().size();
  while (it.hasNext()) {
    int j = it.next();
    if (i >= 0 && j >= 0 && i < size && j < size) {
      v = coord()->point().item(i);
      data[3 * count] = v.x();
      data[3 * count + 1] = v.y();
      data[3 * count + 2] = v.z();
      ++count;

      v = coord()->point().item(j);
      data[3 * count] = v.x();
      data[3 * count + 1] = v.y();
      data[3 * count + 2] = v.z();
      ++count;
    } else {  // i or j are invalid: log them.
      if (j < -1 || j >= size)
        invalidIndices << QString::number(j);
      if (i < -1 || i >= size)
        invalidIndices << QString::number(i);
    }

    i = j;
  }

  if (invalidIndices.size() > 0) {
    invalidIndices.removeDuplicates();
    parsingWarn(
      tr("The following indices are out of the range of coordinates specified in the 'IndexedLineSet.coord' field: %1")
        .arg(invalidIndices.join(", ")));
  }

  return count;
}

void WbIndexedLineSet::updateCoord() {
  if (!sanitizeFields())
    return;

  if (coord())
    connect(coord(), &WbCoordinate::fieldChanged, this, &WbIndexedLineSet::updateCoord, Qt::UniqueConnection);

  if (areWrenObjectsInitialized())
    buildWrenMesh();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  emit changed();
}

void WbIndexedLineSet::updateCoordIndex() {
  if (!sanitizeFields())
    return;

  if (areWrenObjectsInitialized())
    buildWrenMesh();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  emit changed();
}

int WbIndexedLineSet::estimateIndexCount(bool isOutlineMesh) const {
  int ni = 0;
  int s1 = coord()->point().size();
  WbMFInt::Iterator it(*mCoordIndex);
  int i = it.next();
  while (it.hasNext()) {
    int j = it.next();
    if (i != -1 && j != -1 && i < s1 && j < s1)
      ni += 2;
    i = j;
  }
  return ni;
}

void WbIndexedLineSet::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();

  if (!coord())
    return;

  const WbMFVector3 &points = coord()->point();
  if (points.size() == 0)
    return;

  // Ritter's bounding sphere approximation
  // (see description in WbIndexedFaceSet::recomputeBoundingSphere)
  WbMFInt::Iterator it(*mCoordIndex);
  WbVector3 p2(points.item(it.next()));
  WbVector3 p1;
  double maxDistance;  // squared distance
  for (int i = 0; i < 2; ++i) {
    maxDistance = 0.0;
    p1 = p2;
    while (it.hasNext()) {
      const int index = it.next();
      if (index >= 0 && index < points.size()) {  // skip '-1' or other invalid indices.
        const WbVector3 &point = points.item(index);
        const double d = p1.distance2(point);
        if (d > maxDistance) {
          maxDistance = d;
          p2 = point;
        }
      }
    }
    it.toFront();
  }
  mBoundingSphere->set((p2 + p1) * 0.5, sqrt(maxDistance) * 0.5);

  while (it.hasNext()) {
    const int index = it.next();
    if (index >= 0 && index < points.size())  // skip '-1' or other invalid indices.
      mBoundingSphere->enclose(points.item(index));
  }
}

QStringList WbIndexedLineSet::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "coordIndex";
  return fields;
}

////////////////////////
// Friction Direction //
////////////////////////

WbVector3 WbIndexedLineSet::computeFrictionDirection(const WbVector3 &normal) const {
  parsingWarn(tr("A IndexedLineSet is used in a Bounding object using an asymmetric friction. IndexedLineSet does not support "
                 "asymmetric friction"));
  return WbVector3(0, 0, 0);
}
