import {arrayXPointerFloat} from './utils/wbUtils.js';
import {WbGeometry} from './wbGeometry.js';
import {WbWrenShaders} from './../wren/wbWrenShaders.js';

class WbPointSet extends WbGeometry {
  constructor(id, coord, color) {
    super(id);
    this.coord = coord;
    this.color = color;

    this.isShadedGeometryPickable = false;
  }

  delete() {
    _wr_static_mesh_delete(this.wrenMesh);

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();
    _wr_config_enable_point_size(true);
    this._updateCoord();
    this._buildWrenMesh();
  }

  _updateCoord() {
    if (this.wrenObjectsCreatedCalled)
      this._buildWrenMesh();
  }

  setWrenMaterial(material, castShadows) {
    super.setWrenMaterial(material, castShadows);

    if (typeof material !== 'undefined') {
      _wr_material_set_default_program(material, WbWrenShaders.pointSetShader());
      if (typeof this.color !== 'undefined')
        _wr_phong_material_set_color_per_vertex(material, true);
      else
        _wr_phong_material_set_color_per_vertex(material, false);
    }
  }

  _buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this.wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this.wrenMesh);
      this.wrenMesh = undefined;
    }

    if (typeof this.coord === 'undefined' || this.coord.length === 0)
      return;

    super._computeWrenRenderable();

    const coordsData = [];
    let colorData;
    if (typeof this.color !== 'undefined')
      colorData = [];

    const coordsCount = this.computeCoordsAndColorData(coordsData, colorData);

    const coordsDataPointer = arrayXPointerFloat(coordsData);
    const colorDataPointer = arrayXPointerFloat(colorData);
    this.wrenMesh = _wr_static_mesh_point_set_new(coordsCount, coordsDataPointer, colorDataPointer);

    _free(coordsDataPointer);
    _free(colorDataPointer);

    _wr_renderable_set_cast_shadows(this.wrenRenderable, false);
    _wr_renderable_set_receive_shadows(this.wrenRenderable, false);
    _wr_renderable_set_drawing_mode(this.wrenRenderable, ENUM.WR_RENDERABLE_DRAWING_MODE_POINTS);
    _wr_renderable_set_point_size(this.wrenRenderable, 4.0);
    _wr_renderable_set_mesh(this.wrenRenderable, this.wrenMesh);
  }

  computeCoordsAndColorData(coordsData, colorData) {
    if (typeof this.coord === 'undefined')
      return 0;

    let count = 0;
    if (typeof colorData !== 'undefined') {
      const size = Math.min(this.coord.length, this.color.length);
      for (let i = 0; i < size; i++) {
        coordsData[3 * count] = this.coord[i].x;
        coordsData[3 * count + 1] = this.coord[i].y;
        coordsData[3 * count + 2] = this.coord[i].z;
        colorData[3 * count] = this.color[i].x;
        colorData[3 * count + 1] = this.color[i].y;
        colorData[3 * count + 2] = this.color[i].z;
        count++;
      }
    } else {
      for (let i = 0; i < this.coord.length; i++) {
        coordsData[3 * count] = this.coord[i].x;
        coordsData[3 * count + 1] = this.coord[i].y;
        coordsData[3 * count + 2] = this.coord[i].z;
        count++;
      }
    }
    return count;
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbPointSet(customID, this.coord, this.color);
  }
}

export {WbPointSet};
