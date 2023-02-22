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

#ifndef WB_SUPPORT_POLYGON_REPRESENTATION_HPP
#define WB_SUPPORT_POLYGON_REPRESENTATION_HPP

//
// Description: class handling the rendering of the support polygon of a robot
//

class WbPolygon;
class WbVector3;

struct WrTransform;
struct WrDynamicMesh;
struct WrStaticMesh;
struct WrMaterial;
struct WrRenderable;

class WbSupportPolygonRepresentation {
public:
  WbSupportPolygonRepresentation();
  virtual ~WbSupportPolygonRepresentation();

  void show(bool visible);
  void draw(const WbPolygon &p, float y, const WbVector3 &globalCenterOfMass, const WbVector3 *worldBasis);
  void setScale(const float *scale);

private:
  enum { X, Y, Z };  // used to index the world basis vectors

  WrTransform *mTransform;
  WrTransform *mCenterOfMassTransform;

  WrRenderable *mPolygonRenderable;
  WrRenderable *mPolygonOutlineRenderable;
  WrRenderable *mCenterOfMassRenderable;

  WrDynamicMesh *mPolygonMesh;
  WrDynamicMesh *mPolygonOutlineMesh;
  WrStaticMesh *mCenterOfMassMesh;

  WrMaterial *mPolygonMaterial;
  WrMaterial *mPolygonOutlineMaterial;
  WrMaterial *mCenterOfMassMaterial;

  void cleanup();
};

#endif  // WB_SUPPORT_POLYGON_REPRESENTATION_HPP
