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

import {WbTriangleMeshGeometry} from "./WbTriangleMeshGeometry.js"

class WbIndexedFaceSet extends WbTriangleMeshGeometry {
  constructor(id, isDefaultMapping, coordIndex, normalIndex, texCoordIndex, coord, texCoord, normal, creaseAngle, ccw, normalPerVertex) {
    super(id);
    this.isDefaultMapping = isDefaultMapping;

    this.coordIndex = coordIndex;
    this.normalIndex = normalIndex;
    this.texCoordIndex = texCoordIndex;

    this.coord = coord;
    this.texCoord = texCoord;
    this.normal = normal;

    this.creaseAngle = creaseAngle;
    this.ccw = ccw;
    this.normalPerVertex = normalPerVertex;
  }

  updateTriangleMesh() {
    this.triangleMesh.init(this.coord, this.coordIndex, this.normal, this.normalIndex, this.texCoord, this.texCoordIndex, this.creaseAngle, this.ccw, this.normalPerVertex);
  }
}

export {WbIndexedFaceSet}
