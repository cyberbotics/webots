import {arrayXPointerFloat} from './utils/utils.js';
import WbGeometry from './WbGeometry.js';

export default class WbIndexedLineSet extends WbGeometry {
  constructor(id, coord, coordIndex) {
    super(id);
    this.coord = coord;
    this.coordIndex = coordIndex;

    this._isShadedGeometryPickable = false;
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbIndexedLineSet(customID, this.coord, this.coordIndex);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();
    this.#updateCoord();
    this.#buildWrenMesh();
  }

  delete() {
    _wr_static_mesh_delete(this._wrenMesh);

    super.delete();
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

  #updateCoord() {
    if (this.wrenObjectsCreatedCalled)
      this.#buildWrenMesh();
  }
}
