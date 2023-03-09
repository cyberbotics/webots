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

#ifndef WB_SOLID_UTILITIES_HPP
#define WB_SOLID_UTILITIES_HPP

class WbGeometry;
class WbNode;

struct dMass;
typedef struct dxGeom *dGeomID;
typedef struct dxSpace *dSpaceID;

namespace WbSolidUtilities {
  void addMass(dMass *const mass, WbNode *const node, double density, bool warning = true);
  void subtractInertiaMatrix(double *I, const double *J);
  dSpaceID dynamicCastInSpaceID(dGeomID g);
  bool checkBoundingObject(WbNode *const node);  // debug method for testing the validity of a boundingObject
  // extracts the WbGeometry placed into a simple bounding object, i.e. a WbGeometry of a WbShape
  WbGeometry *geometry(WbNode *const node);
  bool isPermanentlyKinematic(const WbNode *node);  // depends on node parent, hence can't be called in node's destructor
  void setDefaultMass(dMass *m);
};  // namespace WbSolidUtilities

#endif  // WB_SOLID_UTILITIES_HPP
