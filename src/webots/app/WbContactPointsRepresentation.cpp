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

#include "WbContactPointsRepresentation.hpp"

#include "WbControlledWorld.hpp"
#include "WbOdeContact.hpp"
#include "WbSimulationState.hpp"
#include "WbSimulationWorld.hpp"
#include "WbWorldInfo.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <ode/fluid_dynamics/ode_fluid_dynamics.h>
#include <ode/ode.h>

#include <wren/node.h>
#include <wren/scene.h>

static inline bool boxVersusPlane(dGeomID g1, dGeomID g2) {
  return (dGeomGetClass(g1) == dBoxClass && dGeomGetClass(g2) == dPlaneClass) ||
         (dGeomGetClass(g1) == dPlaneClass && dGeomGetClass(g2) == dBoxClass);
}

WbContactPointsRepresentation::WbContactPointsRepresentation(WbWrenRenderingContext *context) : mRenderingContext(context) {
  connect(WbSimulationState::instance(), &WbSimulationState::renderingStateChanged, this,
          &WbContactPointsRepresentation::handleSimulationState);
  connect(mRenderingContext, &WbWrenRenderingContext::optionalRenderingChanged, this,
          &WbContactPointsRepresentation::updateOptionalRendering);
  if (mRenderingContext->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_CONTACT_POINTS))
    connect(WbSimulationWorld::instance(), &WbSimulationWorld::physicsStepEnded, this,
            &WbContactPointsRepresentation::updateRendering, Qt::UniqueConnection);

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this,
          &WbContactPointsRepresentation::updateRendering);

  mContactMesh = wr_dynamic_mesh_new(false, false, false);
  mImmersionMesh = wr_dynamic_mesh_new(false, false, false);

  mContactMaterial = wr_phong_material_new();
  const float contactColor[3] = {0.0f, 1.0f, 1.0f};
  wr_phong_material_set_color(mContactMaterial, contactColor);
  wr_phong_material_set_transparency(mContactMaterial, 0.7f);
  wr_material_set_default_program(mContactMaterial, WbWrenShaders::lineSetShader());

  mImmersionMaterial = wr_phong_material_new();
  const float immersionColor[3] = {0.6f, 0.9f, 1.0f};
  wr_phong_material_set_color(mImmersionMaterial, immersionColor);
  wr_phong_material_set_transparency(mImmersionMaterial, 0.7f);
  wr_material_set_default_program(mImmersionMaterial, WbWrenShaders::lineSetShader());

  mContactRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mContactRenderable, false);
  wr_renderable_set_receive_shadows(mContactRenderable, false);
  wr_renderable_set_mesh(mContactRenderable, WR_MESH(mContactMesh));
  wr_renderable_set_material(mContactRenderable, mContactMaterial, NULL);
  wr_renderable_set_drawing_mode(mContactRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_visibility_flags(mContactRenderable, WbWrenRenderingContext::VF_CONTACT_POINTS);
  wr_renderable_set_drawing_order(mContactRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);

  mImmersionRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mImmersionRenderable, false);
  wr_renderable_set_receive_shadows(mImmersionRenderable, false);
  wr_renderable_set_mesh(mImmersionRenderable, WR_MESH(mImmersionMesh));
  wr_renderable_set_material(mImmersionRenderable, mImmersionMaterial, NULL);
  wr_renderable_set_drawing_mode(mImmersionRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_visibility_flags(mImmersionRenderable, WbWrenRenderingContext::VF_CONTACT_POINTS);
  wr_renderable_set_drawing_order(mImmersionRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);

  mTransform = wr_transform_new();
  wr_node_set_visible(WR_NODE(mTransform), false);
  wr_transform_attach_child(mTransform, WR_NODE(mContactRenderable));
  wr_transform_attach_child(mTransform, WR_NODE(mImmersionRenderable));

  WrTransform *root = wr_scene_get_root(wr_scene_get_instance());
  wr_transform_attach_child(root, WR_NODE(mTransform));

  updateOptionalRendering(WbWrenRenderingContext::VF_CONTACT_POINTS);
}

WbContactPointsRepresentation::~WbContactPointsRepresentation() {
  wr_material_delete(mContactMaterial);
  wr_material_delete(mImmersionMaterial);

  wr_dynamic_mesh_delete(mContactMesh);
  wr_dynamic_mesh_delete(mImmersionMesh);

  wr_node_delete(WR_NODE(mContactRenderable));
  wr_node_delete(WR_NODE(mImmersionRenderable));
  wr_node_delete(WR_NODE(mTransform));
}

void WbContactPointsRepresentation::updateOptionalRendering(int option) {
  if (option != WbWrenRenderingContext::VF_CONTACT_POINTS)
    return;

  const WbSimulationWorld *const world = WbSimulationWorld::instance();
  if (world) {
    if (mRenderingContext->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_CONTACT_POINTS)) {
      wr_node_set_visible(WR_NODE(mTransform), true);
      connect(world, &WbSimulationWorld::physicsStepEnded, this, &WbContactPointsRepresentation::updateRendering,
              Qt::UniqueConnection);
      updateRendering();
    } else {
      wr_node_set_visible(WR_NODE(mTransform), false);
      disconnect(world, &WbSimulationWorld::physicsStepEnded, this, &WbContactPointsRepresentation::updateRendering);
    }
  }
}

void WbContactPointsRepresentation::updateRendering() {
  const WbSimulationWorld *const world = WbSimulationWorld::instance();

  const QList<WbOdeContact> &odeContacts = world->odeContacts();
  const int contactSize = odeContacts.size();
  wr_dynamic_mesh_clear(mContactMesh);

  const QList<dImmersionGeom> &immersionGeoms = world->immersionGeoms();
  const int immersionSize = immersionGeoms.size();
  wr_dynamic_mesh_clear(mImmersionMesh);

  // Makes sure to give WREN correct values (even if ODE has exploded)
  for (int i = 0; i < contactSize; ++i) {
    const dContactGeom &cg = odeContacts[i].contactGeom();
    const dReal *const pos = cg.pos;
    for (int j = 0; j < 3; ++j) {
      if (!std::isfinite((float)(pos[j])))
        return;
    }
  }

  int index = 0;
  // draw contact point normals
  const double L = 0.2 * world->worldInfo()->lineScale();
  for (int i = 0; i < contactSize; ++i) {
    const dContactGeom &cg = odeContacts[i].contactGeom();
    if (boxVersusPlane(cg.g1, cg.g2))
      continue;
    const dReal H[3] = {cg.normal[0] * L, cg.normal[1] * L, cg.normal[2] * L};
    const dReal *const pos = cg.pos;
    addVertex(mContactMesh, index++, pos[0] - H[0], pos[1] - H[1], pos[2] - H[2]);
    addVertex(mContactMesh, index++, pos[0] + H[0], pos[1] + H[1], pos[2] + H[2]);
  }

  // Draw lines between connected contact points
  if (contactSize >= 2) {
    int cg0 = 0;
    int cg1 = cg0 + 1;

    // Draw lines or polygons
    for (int i = 0; i < contactSize; ++i, ++cg1) {
      // Is this another geom/geom intersection ?
      if (i != contactSize - 1 && odeContacts[cg0].contactGeom().g1 == odeContacts[cg1].contactGeom().g1 &&
          odeContacts[cg0].contactGeom().g2 == odeContacts[cg1].contactGeom().g2)
        continue;

      // Counts the number of contact points in this geom intersection
      const int k = cg1 - cg0;
      // Draws a single line segment
      if (k == 2) {
        addVertex(mContactMesh, index++, odeContacts[cg0].contactGeom().pos[0], odeContacts[cg0].contactGeom().pos[1],
                  odeContacts[cg0].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0 + 1].contactGeom().pos[0], odeContacts[cg0 + 1].contactGeom().pos[1],
                  odeContacts[cg0 + 1].contactGeom().pos[2]);
      } else if (k == 4 && boxVersusPlane(odeContacts[cg0].contactGeom().g1, odeContacts[cg0].contactGeom().g2)) {
        addVertex(mContactMesh, index++, odeContacts[cg0].contactGeom().pos[0], odeContacts[cg0].contactGeom().pos[1],
                  odeContacts[cg0].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0 + 1].contactGeom().pos[0], odeContacts[cg0 + 1].contactGeom().pos[1],
                  odeContacts[cg0 + 1].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0 + 1].contactGeom().pos[0], odeContacts[cg0 + 1].contactGeom().pos[1],
                  odeContacts[cg0 + 1].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0 + 3].contactGeom().pos[0], odeContacts[cg0 + 3].contactGeom().pos[1],
                  odeContacts[cg0 + 3].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0 + 3].contactGeom().pos[0], odeContacts[cg0 + 3].contactGeom().pos[1],
                  odeContacts[cg0 + 3].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0 + 2].contactGeom().pos[0], odeContacts[cg0 + 1].contactGeom().pos[1],
                  odeContacts[cg0 + 2].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0 + 2].contactGeom().pos[0], odeContacts[cg0 + 1].contactGeom().pos[1],
                  odeContacts[cg0 + 2].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0].contactGeom().pos[0], odeContacts[cg0].contactGeom().pos[1],
                  odeContacts[cg0].contactGeom().pos[2]);
        // Draws a polygon
      } else if (k > 2) {
        int cg2 = cg0;
        int cg;

        for (cg = cg0 + 1; cg < cg1; ++cg) {
          addVertex(mContactMesh, index++, odeContacts[cg2].contactGeom().pos[0], odeContacts[cg2].contactGeom().pos[1],
                    odeContacts[cg2].contactGeom().pos[2]);
          addVertex(mContactMesh, index++, odeContacts[cg].contactGeom().pos[0], odeContacts[cg].contactGeom().pos[1],
                    odeContacts[cg].contactGeom().pos[2]);
          cg2 = cg;
        }

        // Closes the polygon
        addVertex(mContactMesh, index++, odeContacts[cg - 1].contactGeom().pos[0], odeContacts[cg - 1].contactGeom().pos[1],
                  odeContacts[cg - 1].contactGeom().pos[2]);
        addVertex(mContactMesh, index++, odeContacts[cg0].contactGeom().pos[0], odeContacts[cg0].contactGeom().pos[1],
                  odeContacts[cg0].contactGeom().pos[2]);
      }

      // Remembers the first contact point of the polygon
      cg0 = cg1;
    }
  }

  index = 0;
  for (int i = 0; i < immersionSize; ++i) {
    dImmersionOutlineID io = immersionGeoms[i].outline;
    const int ssize = dImmersionOutlineGetStraightEdgesSize(io);
    for (int j = 0; j < ssize; ++j) {
      const double *const origin = dImmersionOutlineGetStraightEdgeOrigin(io, j);
      addVertex(mImmersionMesh, index++, origin[0], origin[1], origin[2]);
      const double *const end = dImmersionOutlineGetStraightEdgeEnd(io, j);
      addVertex(mImmersionMesh, index++, end[0], end[1], end[2]);
    }

    static const int NUMBER_OF_THETA_STEPS = 35;
    const int csize = dImmersionOutlineGetCurvedEdgesSize(io);
    dCurvedEdge ce;
    dVector3 v;
    for (int j = 0; j < csize; ++j) {
      dImmersionOutlineGetCurvedEdge(io, j, &ce);
      double theta = ce.minAngle;
      const double *const c = ce.center;
      const double stepSize = (ce.maxAngle - theta) / NUMBER_OF_THETA_STEPS;
      for (int k = 0; k < NUMBER_OF_THETA_STEPS; ++k) {
        dAddScaledVectors3(v, ce.e1, ce.e2, cos(theta), sin(theta));
        addVertex(mImmersionMesh, index++, c[0] + v[0], c[1] + v[1], c[2] + v[2]);
        theta += stepSize;
        dAddScaledVectors3(v, ce.e1, ce.e2, cos(theta), sin(theta));
        addVertex(mImmersionMesh, index++, c[0] + v[0], c[1] + v[1], c[2] + v[2]);
      }
    }
  }
}

void WbContactPointsRepresentation::handleSimulationState() {
  const WbSimulationState *const state = WbSimulationState::instance();
  const WbSimulationWorld *const world = WbSimulationWorld::instance();

  if (!state->isRendering()) {
    disconnect(world, &WbSimulationWorld::physicsStepEnded, this, &WbContactPointsRepresentation::updateRendering);
    wr_node_set_visible(WR_NODE(mTransform), false);
  } else if (mRenderingContext->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_CONTACT_POINTS)) {
    connect(world, &WbSimulationWorld::physicsStepEnded, this, &WbContactPointsRepresentation::updateRendering,
            Qt::UniqueConnection);
    wr_node_set_visible(WR_NODE(mTransform), true);
    updateRendering();
  }
}

void WbContactPointsRepresentation::addVertex(WrDynamicMesh *mesh, int index, double x, double y, double z) const {
  const float vertex[3] = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
  wr_dynamic_mesh_add_vertex(mesh, vertex);
  wr_dynamic_mesh_add_index(mesh, index);
}
