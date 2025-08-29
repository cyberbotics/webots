import WbGeometry from './WbGeometry.js';

import {arrayXPointerFloat} from './utils/utils.js';
import {getAnId} from './utils/id_provider.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbIndexedLineSet extends WbGeometry {
  #coord;
  #coordIndex;
  constructor(id, coord, coordIndex) {
    super(id);
    this.#coord = coord;
    this.#coordIndex = coordIndex;

    this._isShadedGeometryPickable = false;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_INDEXED_LINE_SET;
  }

  get coordIndex() {
    return this.#coordIndex;
  }

  set coordIndex(newCoordIndex) {
    this.#coordIndex = newCoordIndex;

    if (this.wrenObjectsCreatedCalled)
      this.#updateCoordIndex();
  }

  clone(customID) {
    this.useList.push(customID);
    const newCoord = this.#coord?.clone(getAnId());
    return new WbIndexedLineSet(customID, newCoord, this.#coordIndex);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();

    this.#coord?.createWrenObjects();

    this.#buildWrenMesh();
  }

  delete() {
    _wr_static_mesh_delete(this._wrenMesh);

    this.#coord?.delete();

    super.delete();
  }

  preFinalize() {
    super.preFinalize();

    this.#coord?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#coord?.postFinalize();

    if (typeof this.#coord !== 'undefined') {
      this.#coord.onChange = () => {
        this.#buildWrenMesh();
        if (typeof this.onRecreated === 'function')
          this.onRecreated();
      };
    }
  }

  recomputeBoundingSphere() {
    this._boundingSphere.empty();

    if (typeof this.#coord === 'undefined')
      return;

    const points = this.#coord.point;
    if (points.length === 0)
      return;

    // Ritter's bounding sphere approximation
    // (see description in WbIndexedFaceSet::recomputeBoundingSphere)
    let p2 = points[this.#coordIndex[0]];
    let p1;
    let maxDistance; // squared distance
    for (let i = 0; i < 2; ++i) {
      maxDistance = 0;
      p1 = p2;
      for (let j = 0; j < this.#coordIndex.length; j++) {
        const index = this.#coordIndex[j];
        if (index >= 0 && index < points.length) { // skip '-1' or other invalid indices.
          const point = points[index];
          const d = p1.distance2(point);
          if (d > maxDistance) {
            maxDistance = d;
            p2 = point;
          }
        }
      }
    }

    this._boundingSphere.set(p2.add(p1).mul(0.5), Math.sqrt(maxDistance) * 0.5);

    for (let i = 0; i < this.#coordIndex.length; i++) {
      const index = this.#coordIndex[i];
      if (index >= 0 && index < points.length) // skip '-1' or other invalid indices.
        this._boundingSphere.enclose(points[index]);
    }
  }

  // Private functions
  #buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    super._computeWrenRenderable();

    _wr_renderable_set_drawing_mode(this._wrenRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);

    const coordsData = [];
    const coordsCount = this.#computeCoordsData(coordsData);

    if (coordsCount > 0) {
      const coordsDataPointer = arrayXPointerFloat(coordsData);
      this._wrenMesh = _wr_static_mesh_line_set_new(coordsCount, coordsDataPointer, null);
      _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
      _free(coordsDataPointer);
    }
  }

  #computeCoordsData(data) {
    if (typeof this.#coord === 'undefined')
      return;

    let count = 0;
    const size = this.#coord.point.length;
    const invalidIndices = [];

    for (let i = 0; i < this.#coordIndex.length - 1; i++) {
      let j = i + 1;
      if (this.#coordIndex[i] >= 0 && this.#coordIndex[j] >= 0 && this.#coordIndex[i] < size && this.#coordIndex[j] < size) {
        let v = this.#coord.point[this.#coordIndex[i] ];
        data[3 * count] = v.x;
        data[3 * count + 1] = v.y;
        data[3 * count + 2] = v.z;
        ++count;

        v = this.#coord.point[this.#coordIndex[j]];
        data[3 * count] = v.x;
        data[3 * count + 1] = v.y;
        data[3 * count + 2] = v.z;
        ++count;
      } else {
        if (this.#coordIndex[i] < -1 || this.#coordIndex[i] >= size)
          invalidIndices.push(this.#coordIndex[i]);
        if (this.#coordIndex[j] < -1 || this.#coordIndex[j] >= size)
          invalidIndices.push(this.#coordIndex[j]);
      }
    }

    if (invalidIndices.length > 0) {
      console.warn("The following indices are out of the range of coordinates specified in the 'IndexedLineSet.coord' field:");
      console.warn(invalidIndices);
    }

    return count;
  }

  #updateCoordIndex() {
    this.#buildWrenMesh();

    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
