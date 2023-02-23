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

#ifndef WB_CONTACT_POINTS_REPRESENTATION_HPP
#define WB_CONTACT_POINTS_REPRESENTATION_HPP

//
// Description: class handling rendering the contact points
//

#include <QtCore/QObject>

#include <wren/dynamic_mesh.h>
#include <wren/material.h>
#include <wren/renderable.h>
#include <wren/transform.h>

class WbWrenRenderingContext;

class WbContactPointsRepresentation : public QObject {
  Q_OBJECT;

public:
  explicit WbContactPointsRepresentation(WbWrenRenderingContext *context);
  virtual ~WbContactPointsRepresentation();

private slots:
  void updateRendering();
  void updateOptionalRendering(int);
  void handleSimulationState();

private:
  void addVertex(WrDynamicMesh *mesh, int index, double x, double y, double z) const;

  WbWrenRenderingContext *mRenderingContext;

  WrDynamicMesh *mContactMesh;
  WrRenderable *mContactRenderable;

  WrDynamicMesh *mImmersionMesh;
  WrRenderable *mImmersionRenderable;

  WrMaterial *mContactMaterial;
  WrMaterial *mImmersionMaterial;
  WrTransform *mTransform;
};

#endif
