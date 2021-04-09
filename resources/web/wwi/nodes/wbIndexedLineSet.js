// Copyright 1996-2021 Cyberbotics Ltd.
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

import {arrayXPointerFloat} from './wbUtils.js';
import {WbGeometry} from './wbGeometry.js';

class WbIndexedLineSet extends WbGeometry {
  constructor(id, coord, coordIndex) {
    super(id);
    this.coord = coord;
    this.coordIndex = coordIndex;

    this.isShadedGeometryPickable = false;
  }

  delete() {
    _wr_static_mesh_delete(this.wrenMesh);

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();
    this.updateCoord();
    this.buildWrenMesh();
  }

  updateCoord() {
    if (this.wrenObjectsCreatedCalled)
      this.buildWrenMesh();
  }

  buildWrenMesh() {
    super.deleteWrenRenderable();

    if (typeof this.wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this.wrenMesh);
      this.wrenMesh = undefined;
    }

    super.computeWrenRenderable();

    _wr_renderable_set_drawing_mode(this.wrenRenderable, ENUM.WR_RENDERABLE_DRAWING_MODE_LINES);

    const coordsData = [];
    const coordsCount = this.computeCoordsData(coordsData);

    if (coordsCount > 0) {
      const coordsDataPointer = arrayXPointerFloat(coordsData);
      this.wrenMesh = _wr_static_mesh_line_set_new(coordsCount, coordsDataPointer, null);
      _wr_renderable_set_mesh(this.wrenRenderable, this.wrenMesh);
      _free(coordsDataPointer);
    }
  }

  computeCoordsData(data) {
    let count = 0;
    const size = this.coord.length;
    const invalidIndices = [];

    for (let i = 0; i < this.coordIndex.length - 1; i++) {
      let j = i + 1;
      if (this.coordIndex[i] >= 0 && this.coordIndex[j] >= 0 && this.coordIndex[i] < size && this.coordIndex[j] < size) {
        let v = this.coord[this.coordIndex[i] ];
        data[3 * count] = v.x;
        data[3 * count + 1] = v.y;
        data[3 * count + 2] = v.z;
        ++count;

        v = this.coord[this.coordIndex[j]];
        data[3 * count] = v.x;
        data[3 * count + 1] = v.y;
        data[3 * count + 2] = v.z;
        ++count;
      } else {
        if (this.coordIndex[i] < -1 || this.coordIndex[i] >= size)
          invalidIndices.push(this.coordIndex[i]);
        if (this.coordIndex[j] < -1 || this.coordIndex[j] >= size)
          invalidIndices.push(this.coordIndex[j]);
      }
    }

    if (invalidIndices.length > 0) {
      console.warn("The following indices are out of the range of coordinates specified in the 'IndexedLineSet.coord' field:");
      console.warn(invalidIndices);
    }

    return count;
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbIndexedLineSet(customID, this.coord, this.coordIndex);
  }
}

export {WbIndexedLineSet};
