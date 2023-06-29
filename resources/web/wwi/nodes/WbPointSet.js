import WbGeometry from './WbGeometry.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import {arrayXPointerFloat} from './utils/utils.js';
import {getAnId} from './utils/id_provider.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbPointSet extends WbGeometry {
  #color;
  #coord;
  constructor(id, coord, color) {
    super(id);
    this.#coord = coord;
    this.#color = color;

    this._isShadedGeometryPickable = false;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_POINT_SET;
  }

  clone(customID) {
    this.useList.push(customID);
    const newColor = this.#color?.clone(getAnId());
    const newCoord = this.#coord?.clone(getAnId());
    return new WbPointSet(customID, newCoord, newColor);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();

    this.#color?.createWrenObjects();
    this.#coord?.createWrenObjects();

    _wr_config_enable_point_size(true);

    this.#buildWrenMesh();
  }

  delete() {
    this.#color?.delete();
    this.#coord?.delete();

    super.delete();

    _wr_static_mesh_delete(this._wrenMesh);
  }

  setWrenMaterial(material, castShadows) {
    super.setWrenMaterial(material, castShadows);

    if (typeof material !== 'undefined') {
      _wr_material_set_default_program(material, WbWrenShaders.pointSetShader());
      if (typeof this.#color !== 'undefined')
        _wr_phong_material_set_color_per_vertex(material, true);
      else
        _wr_phong_material_set_color_per_vertex(material, false);
    }
  }

  preFinalize() {
    super.preFinalize();

    this.#color?.preFinalize();
    this.#coord?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#color?.postFinalize();
    this.#coord?.postFinalize();

    if (typeof this.#color !== 'undefined') {
      this.#color.onChange = () => {
        this.#buildWrenMesh();
        if (typeof this.onRecreated === 'function')
          this.onRecreated();
      };
    }

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
    let p2 = points[0];
    let p1;
    let maxDistance; // squared distance
    for (let i = 0; i < 2; ++i) {
      maxDistance = 0;
      p1 = p2;
      for (let j = 0; j < points.length; j++) {
        const point = points[j];
        const d = p1.distance2(point);
        if (d > maxDistance) {
          maxDistance = d;
          p2 = point;
        }
      }
    }

    this._boundingSphere.set(p2.add(p1).mul(0.5), Math.sqrt(maxDistance) * 0.5);

    for (let i = 0; i < points.length; i++)
      this._boundingSphere.enclose(points[i]);
  }

  // Private functions

  #buildWrenMesh() {
    super._deleteWrenRenderable();
    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    if (typeof this.#coord === 'undefined' || this.#coord.point.length === 0)
      return;

    super._computeWrenRenderable();

    const coordsData = [];
    let colorData;
    if (typeof this.#color !== 'undefined')
      colorData = [];

    const coordsCount = this.#computeCoordsAndColorData(coordsData, colorData);

    const coordsDataPointer = arrayXPointerFloat(coordsData);
    const colorDataPointer = arrayXPointerFloat(colorData);
    this._wrenMesh = _wr_static_mesh_point_set_new(coordsCount, coordsDataPointer, colorDataPointer);

    _free(coordsDataPointer);
    _free(colorDataPointer);

    _wr_renderable_set_cast_shadows(this._wrenRenderable, false);
    _wr_renderable_set_receive_shadows(this._wrenRenderable, false);
    _wr_renderable_set_drawing_mode(this._wrenRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_POINTS);
    _wr_renderable_set_point_size(this._wrenRenderable, 4.0);
    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  }

  #computeCoordsAndColorData(coordsData, colorData) {
    if (typeof this.#coord === 'undefined')
      return 0;

    let count = 0;
    if (typeof colorData !== 'undefined') {
      const size = Math.min(this.#coord.point.length, this.#color.color.length);
      for (let i = 0; i < size; i++) {
        coordsData[3 * count] = this.#coord.point[i].x;
        coordsData[3 * count + 1] = this.#coord.point[i].y;
        coordsData[3 * count + 2] = this.#coord.point[i].z;
        colorData[3 * count] = this.#color.color[i].x;
        colorData[3 * count + 1] = this.#color.color[i].y;
        colorData[3 * count + 2] = this.#color.color[i].z;
        count++;
      }
    } else {
      for (let i = 0; i < this.#coord.point.length; i++) {
        coordsData[3 * count] = this.#coord.point[i].x;
        coordsData[3 * count + 1] = this.#coord.point[i].y;
        coordsData[3 * count + 2] = this.#coord.point[i].z;
        count++;
      }
    }
    return count;
  }
}
