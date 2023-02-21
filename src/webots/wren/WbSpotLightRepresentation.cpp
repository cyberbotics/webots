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

#include "WbSpotLightRepresentation.hpp"

#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <QtCore/QVector>

WbSpotLightRepresentation::WbSpotLightRepresentation(WrTransform *parent, const WbVector3 &position, float radius,
                                                     float cutOffAngle, const WbVector3 &direction) :
  WbLightRepresentation(parent, position),
  mRadius(radius),
  mCutOffAngle(cutOffAngle),
  mDirection(direction),
  mMesh(NULL) {
  mMaterial = wr_phong_material_new();
  wr_material_set_default_program(mMaterial, WbWrenShaders::lineSetShader());
  const float color[3] = {1.0f, 1.0f, 0.0f};
  wr_phong_material_set_color(mMaterial, color);
  wr_phong_material_set_transparency(mMaterial, 0.4f);

  mRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mRenderable, false);
  wr_renderable_set_receive_shadows(mRenderable, false);
  wr_renderable_set_material(mRenderable, mMaterial, NULL);
  wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VF_LIGHTS_POSITIONS);
  wr_renderable_set_drawing_mode(mRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);

  mTransform = wr_transform_new();
  wr_transform_attach_child(mTransform, WR_NODE(mRenderable));
  wr_transform_attach_child(parent, WR_NODE(mTransform));

  updateMesh();
}

WbSpotLightRepresentation::~WbSpotLightRepresentation() {
  wr_node_delete(WR_NODE(mTransform));
  wr_node_delete(WR_NODE(mRenderable));
  wr_material_delete(mMaterial);
  wr_static_mesh_delete(mMesh);
}

void WbSpotLightRepresentation::setVisible(bool visible) {
  WbLightRepresentation::setVisible(visible);
  wr_node_set_visible(WR_NODE(mTransform), visible);
}

void WbSpotLightRepresentation::setPosition(const WbVector3 &position) {
  WbLightRepresentation::setPosition(position);
  updateMesh();
}

static void addVertex(QVector<float> &vertices, const WbVector3 &vertex) {
  vertices.push_back(vertex.x());
  vertices.push_back(vertex.y());
  vertices.push_back(vertex.z());
}

void WbSpotLightRepresentation::updateMesh() {
  if (mMesh)
    wr_static_mesh_delete(mMesh);

  const float circleDistance = mRadius * cosf(mCutOffAngle);
  const float circleRadius = mRadius * sinf(mCutOffAngle);
  const WbVector3 circleCenter = mPosition + circleDistance * mDirection;

  // Vectors that define the plane on which the circle stands
  assert(!mDirection.isNull());
  WbVector3 planeVectorA;
  if (mDirection.z() != 0.0f && mDirection.x() != -mDirection.y())
    planeVectorA = WbVector3(mDirection.z(), mDirection.z(), -mDirection.x() - mDirection.y());
  else
    planeVectorA = WbVector3(-mDirection.y() - mDirection.z(), mDirection.x(), mDirection.x());

  planeVectorA.normalize();
  WbVector3 planeVectorB = mDirection.cross(planeVectorA).normalized();

  // Draw spotlight outline
  const int steps = 32;
  const float angleSteps = 2.0f * M_PI / steps;
  QVector<float> vertices;

  WbVector3 point = circleCenter + circleRadius * planeVectorA;
  for (int i = 1; i <= steps; ++i) {
    // Circle
    addVertex(vertices, point);
    const WbVector3 circleVector = cosf(angleSteps * i) * planeVectorA + sinf(angleSteps * i) * planeVectorB;
    point = circleCenter + circleRadius * circleVector;
    addVertex(vertices, point);

    // Draw line from light to current position
    if ((i - 1) % (steps / 4) == 0) {
      addVertex(vertices, mPosition);
      addVertex(vertices, point);

      // Draw circle arc
      if (i < steps / 2) {
        const float startAngle = -mCutOffAngle;
        const float angleStep = 4.0f * mCutOffAngle / steps;
        WbVector3 arcPoint = mPosition + mRadius * (cosf(startAngle) * mDirection + sinf(startAngle) * circleVector);
        for (int j = 1; j <= steps / 2; ++j) {
          addVertex(vertices, arcPoint);
          arcPoint = mPosition + mRadius * (cosf(startAngle + angleStep * j) * mDirection +
                                            sinf(startAngle + angleStep * j) * circleVector);
          addVertex(vertices, arcPoint);
        }
      }
    }
  }
  mMesh = wr_static_mesh_line_set_new(vertices.size() / 3, vertices.data(), NULL);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
}
