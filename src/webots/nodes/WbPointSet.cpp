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

#include "WbPointSet.hpp"

#include "WbBoundingSphere.hpp"
#include "WbColor.hpp"
#include "WbCoordinate.hpp"
#include "WbField.hpp"
#include "WbMFColor.hpp"
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

void WbPointSet::init() {
  mColor = findSFNode("color");
  mCoord = findSFNode("coord");
}

WbPointSet::WbPointSet(WbTokenizer *tokenizer) : WbGeometry("PointSet", tokenizer) {
  init();
}

WbPointSet::WbPointSet(const WbPointSet &other) : WbGeometry(other) {
  init();
}

WbPointSet::WbPointSet(const WbNode &other) : WbGeometry(other) {
  init();
}

WbPointSet::~WbPointSet() {
  wr_static_mesh_delete(mWrenMesh);
}

void WbPointSet::postFinalize() {
  WbGeometry::postFinalize();

  connect(mCoord, &WbSFNode::changed, this, &WbPointSet::updateCoord);
  connect(mColor, &WbSFNode::changed, this, &WbPointSet::updateColor);

  if (coord())
    connect(coord(), &WbCoordinate::fieldChanged, this, &WbPointSet::updateCoord, Qt::UniqueConnection);

  if (color())
    connect(color(), &WbColor::fieldChanged, this, &WbPointSet::updateColor, Qt::UniqueConnection);
}

WbCoordinate *WbPointSet::coord() const {
  return static_cast<WbCoordinate *>(mCoord->value());
}

WbColor *WbPointSet::color() const {
  return static_cast<WbColor *>(mColor->value());
}

void WbPointSet::createWrenObjects() {
  WbGeometry::createWrenObjects();
  wr_config_enable_point_size(true);
  updateCoord();
  buildWrenMesh();
  emit wrenObjectsCreated();
}

void WbPointSet::setWrenMaterial(WrMaterial *material, bool castShadows) {
  WbGeometry::setWrenMaterial(material, castShadows);
  if (material) {
    wr_material_set_default_program(material, WbWrenShaders::pointSetShader());
    if (color())
      wr_phong_material_set_color_per_vertex(material, true);
    else
      wr_phong_material_set_color_per_vertex(material, false);
  }
}

bool WbPointSet::sanitizeFields() {
  if (!coord() || coord()->point().isEmpty()) {
    parsingWarn(tr("A non-empty 'Coordinate' node should be present in the 'coord' field."));
    return false;
  }

  if (color() && color()->color().size() != coord()->pointSize()) {
    parsingWarn(tr("If a 'Color' node is present in the 'color' field, it should have the same number of component as the "
                   "'Coordinate' node in the 'coord' field."));
    if (color()->color().isEmpty())
      return false;
    else
      parsingWarn(tr("Only the %1 first points will be drawn.").arg(qMin(color()->color().size(), coord()->point().size())));
  }

  return true;
}

void WbPointSet::buildWrenMesh() {
  WbGeometry::deleteWrenRenderable();

  wr_static_mesh_delete(mWrenMesh);
  mWrenMesh = NULL;

  if (!coord() || coord()->pointSize() == 0)
    return;

  WbGeometry::computeWrenRenderable();

  float *coordsData = new float[coord()->pointSize() * 3];
  float *colorData = NULL;
  if (color())
    colorData = new float[color()->color().size() * 3];
  int coordsCount = computeCoordsAndColorData(coordsData, colorData);

  mWrenMesh = wr_static_mesh_point_set_new(coordsCount, coordsData, colorData);

  delete[] coordsData;

  wr_renderable_set_cast_shadows(mWrenRenderable, false);
  wr_renderable_set_receive_shadows(mWrenRenderable, false);
  wr_renderable_set_drawing_mode(mWrenRenderable, WR_RENDERABLE_DRAWING_MODE_POINTS);
  wr_renderable_set_point_size(mWrenRenderable, 4.0f);
  wr_renderable_set_mesh(mWrenRenderable, WR_MESH(mWrenMesh));
}

int WbPointSet::computeCoordsAndColorData(float *coordsData, float *colorData) {
  if (!coord())
    return 0;

  WbMFVector3::Iterator coordsIt(coord()->point());
  int count = 0;
  if (colorData) {
    WbMFColor::Iterator colorIt(color()->color());
    while (coordsIt.hasNext() && colorIt.hasNext()) {
      WbVector3 v = coordsIt.next();
      WbRgb c = colorIt.next();
      coordsData[3 * count] = v.x();
      coordsData[3 * count + 1] = v.y();
      coordsData[3 * count + 2] = v.z();
      colorData[3 * count] = c.red();
      colorData[3 * count + 1] = c.green();
      colorData[3 * count + 2] = c.blue();
      count++;
    }
  } else {
    while (coordsIt.hasNext()) {
      WbVector3 v = coordsIt.next();
      coordsData[3 * count] = v.x();
      coordsData[3 * count + 1] = v.y();
      coordsData[3 * count + 2] = v.z();
      count++;
    }
  }
  return count;
}

void WbPointSet::updateCoord() {
  if (coord())
    connect(coord(), &WbCoordinate::fieldChanged, this, &WbPointSet::updateCoord, Qt::UniqueConnection);

  if (!sanitizeFields())
    return;

  if (areWrenObjectsInitialized())
    buildWrenMesh();

  if (mBoundingSphere)
    mBoundingSphere->setOwnerSizeChanged();

  emit changed();
}

void WbPointSet::updateColor() {
  if (color())
    connect(color(), &WbCoordinate::fieldChanged, this, &WbPointSet::updateColor, Qt::UniqueConnection);

  if (!sanitizeFields())
    return;

  if (areWrenObjectsInitialized())
    buildWrenMesh();

  emit changed();
}

void WbPointSet::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();

  if (!coord())
    return;

  const WbMFVector3 &points = coord()->point();
  if (points.size() == 0)
    return;

  // Ritter's bounding sphere approximation
  WbMFVector3::Iterator it(points);
  WbVector3 p2 = it.next();
  WbVector3 p1;
  double maxDistance;  // squared distance
  for (int i = 0; i < 2; ++i) {
    maxDistance = 0.0;
    p1 = p2;
    while (it.hasNext()) {
      const WbVector3 &point = it.next();
      const double d = p1.distance2(point);
      if (d > maxDistance) {
        maxDistance = d;
        p2 = point;
      }
    }
    it.toFront();
  }
  mBoundingSphere->set((p2 + p1) * 0.5, sqrt(maxDistance) * 0.5);

  while (it.hasNext())
    mBoundingSphere->enclose(it.next());
}
