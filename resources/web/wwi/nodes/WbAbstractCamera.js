import WbDevice from './WbDevice.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import {arrayXPointerFloat} from './utils/utils.js';

export default class WbAbstractCamera extends WbDevice {
  #fieldOfView;
  #height;
  #material;
  #renderable;
  #transform;
  #width;
  constructor(id, translation, rotation, name, height, width, fieldOfView) {
    super(id, translation, rotation, name);

    this.#fieldOfView = fieldOfView;
    this.#height = height;
    this.#width = width;

    this._isRangeFinder = false;
    this._isFrustumEnabled = false;
    this._charType = '';
  }

  get fieldOfView() {
    return this.#fieldOfView;
  }

  set fieldOfView(newFieldOfView) {
    if (newFieldOfView <= 0 || newFieldOfView > Math.PI)
      newFieldOfView = Math.PI / 2;

    this.#fieldOfView = newFieldOfView;
    this._update();
  }

  get height() {
    return this.#height;
  }

  set height(newHeight) {
    if (newHeight <= 0)
      newHeight = 1;

    this.#height = newHeight;
    this._update();
  }

  get width() {
    return this.#width;
  }

  set width(newWidth) {
    if (newWidth <= 0)
      newWidth = 1;

    this.#width = newWidth;
    this._update();
  }

  createWrenObjects() {
    super.createWrenObjects();
    // Frustum
    this.#material = _wr_phong_material_new();
    _wr_material_set_default_program(this.#material, WbWrenShaders.lineSetShader());
    _wr_phong_material_set_color_per_vertex(this.#material, true);

    this.#renderable = _wr_renderable_new();
    _wr_renderable_set_cast_shadows(this.#renderable, false);
    _wr_renderable_set_receive_shadows(this.#renderable, false);
    _wr_renderable_set_material(this.#renderable, this.#material, null);
    _wr_renderable_set_drawing_mode(this.#renderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);

    this.#transform = _wr_transform_new();
    _wr_transform_attach_child(this.#transform, this.#renderable);
    _wr_transform_attach_child(this.wrenNode, this.#transform);
    this._applyFrustumToWren();
  }

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      _wr_static_mesh_delete(this._mesh);
      _wr_node_delete(this.#renderable);
      _wr_node_delete(this.#transform);
      _wr_material_delete(this.#material);
    }

    super.delete();
  }

  _applyFrustumToWren() {
    _wr_node_set_visible(this.#transform, false);

    if (typeof this._mesh !== 'undefined') {
      _wr_static_mesh_delete(this._mesh);
      this._mesh = undefined;
    }

    if (!this._isFrustumEnabled)
      return;

    let frustumColor;
    if (this._charType === 'c')
      frustumColor = [1, 0, 1];
    else if (this._charType === 'r')
      frustumColor = [1, 1, 0];

    const frustumColorRgb = _wrjs_array3(frustumColor[0], frustumColor[1], frustumColor[2]);

    _wr_phong_material_set_color(this.#material, frustumColorRgb);

    let drawFarPlane;
    let f;
    const n = this._minRange();
    // if the far is set to 0 it means the far clipping plane is set to infinity
    // so, the far distance of the colored frustum should be set arbitrarily
    if (this._charType === 'c' && this._maxRange() === 0) {
      f = n + 2 * _wr_config_get_line_scale();
      drawFarPlane = false;
    } else {
      f = this._maxRange();
      drawFarPlane = true;
    }

    const w = this.#width;
    const h = this.#height;
    const fovX = this.#fieldOfView;
    const t = Math.tan(fovX / 2);
    const dw1 = n * t;
    const dh1 = dw1 * h / w;
    const n1 = n;
    const dw2 = f * t;
    const dh2 = dw2 * h / w;
    const n2 = f;

    let vertices = [];
    let colors = [];
    let vertex = [0, 0, 0];
    this.#addVertex(vertices, colors, vertex, frustumColor);
    vertex[0] = n;
    this.#addVertex(vertices, colors, vertex, frustumColor);

    const pos = [[n1, dw1, dh1], [n1, dw1, -dh1], [n1, -dw1, -dh1], [n1, -dw1, dh1]];
    this.#drawRectangle(vertices, colors, pos, frustumColor);

    // Creation of the far plane
    // if the camera is not of the range-finder type, the far is set to infinity
    // so, the far rectangle of the colored frustum shouldn't be set
    if (drawFarPlane) {
      const pos = [[n2, dw2, dh2], [n2, dw2, -dh2], [n2, -dw2, -dh2], [n2, -dw2, dh2]];
      this.#drawRectangle(vertices, colors, pos, frustumColor);
    }
    // Creation of the external outline of the frustum (4 lines)
    const frustumOutline = [[n1, dw1, dh1], [n2, dw2, dh2], [n1, -dw1, dh1], [n2, -dw2, dh2], [n1, dw1, -dh1], [n2, dw2, -dh2],
      [n1, -dw1, -dh1], [n2, -dw2, -dh2]];
    for (let i = 0; i < 8; i++)
      this.#addVertex(vertices, colors, frustumOutline[i], frustumColor);

    const verticesPointer = arrayXPointerFloat(vertices);
    const colorsPointer = arrayXPointerFloat(colors);
    this._mesh = _wr_static_mesh_line_set_new(vertices.length / 3, verticesPointer, colorsPointer);
    _wr_renderable_set_mesh(this.#renderable, this._mesh);
    _free(verticesPointer);
    _free(colorsPointer);

    if (this._isRangeFinder)
      _wr_renderable_set_visibility_flags(this.#renderable, WbWrenRenderingContext.VM_REGULAR);
    else
      _wr_renderable_set_visibility_flags(this.#renderable, WbWrenRenderingContext.VM_REGULAR);

    _wr_node_set_visible(this.#transform, true);
  }

  _minRange() {
  }

  _maxRange() {
    return 1.0;
  }

  _update() {
    if (this.wrenObjectsCreatedCalled)
      this._applyFrustumToWren();
  }

  applyOptionalRendering(enable) {
    this._isFrustumEnabled = enable;
    this._applyFrustumToWren();
  }

  #addVertex(vertices, colors, vertex, color) {
    vertices.push(vertex[0]);
    vertices.push(vertex[1]);
    vertices.push(vertex[2]);

    colors.push(color[0]);
    colors.push(color[1]);
    colors.push(color[2]);
  }

  #drawRectangle(vertices, colors, v, color) {
    for (let i = 0; i < 4; ++i) {
      this.#addVertex(vertices, colors, v[i], color);
      this.#addVertex(vertices, colors, v[(i + 1) % 4], color);
    }
  }
}
