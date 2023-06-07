import WbDevice from './WbDevice.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import {arrayXPointerFloat} from './utils/utils.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbRadar extends WbDevice {
  #horizontalFieldOfView;
  #material;
  #maxRange;
  #mesh;
  #minRange;
  #renderable;
  #transform;
  #verticalFieldOfView;
  constructor(id, translation, rotation, name, horizontalFieldOfView, verticalFieldOfView, maxRange, minRange) {
    super(id, translation, rotation, name);

    this.#horizontalFieldOfView = horizontalFieldOfView;
    this.#verticalFieldOfView = verticalFieldOfView;
    this.#maxRange = maxRange;
    this.#minRange = minRange;
    this._isFrustumEnabled = false;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_RADAR;
  }

  get horizontalFieldOfView() {
    return this.#horizontalFieldOfView;
  }

  set horizontalFieldOfView(newHorizontalFieldOfView) {
    if (newHorizontalFieldOfView <= 0 || newHorizontalFieldOfView > Math.PI)
      newHorizontalFieldOfView = 0.78;

    this.#horizontalFieldOfView = newHorizontalFieldOfView;
    this.#update();
  }

  get maxRange() {
    return this.#maxRange;
  }

  set maxRange(newMaxRange) {
    if (this.#minRange > newMaxRange || newMaxRange <= 0)
      newMaxRange = this.#minRange + 1;

    this.#maxRange = newMaxRange;
    this.#update();
  }

  get minRange() {
    return this.#minRange;
  }

  set minRange(newMinRange) {
    if (newMinRange < 0)
      newMinRange = 0;
    else if (newMinRange > this.#maxRange)
      newMinRange = this.#maxRange < 1 ? 0 : this.#maxRange - 1;

    this.#minRange = newMinRange;
    this.#update();
  }

  get verticalFieldOfView() {
    return this.#verticalFieldOfView;
  }

  set verticalFieldOfView(newVerticalFieldOfView) {
    if (newVerticalFieldOfView <= 0 || newVerticalFieldOfView > Math.PI / 2)
      newVerticalFieldOfView = 0.1;

    this.#verticalFieldOfView = newVerticalFieldOfView;
    this.#update();
  }

  createWrenObjects() {
    this.#transform = _wr_transform_new();
    _wr_node_set_visible(this.#transform, false);

    this.#material = _wr_phong_material_new();
    _wr_material_set_default_program(this.#material, WbWrenShaders.lineSetShader());

    this.#renderable = _wr_renderable_new();
    _wr_renderable_set_visibility_flags(this.#renderable, WbWrenRenderingContext.VM_REGULAR);
    _wr_renderable_set_drawing_mode(this.#renderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
    _wr_renderable_set_material(this.#renderable, this.#material, undefined);

    super.createWrenObjects();

    _wr_transform_attach_child(this.#transform, this.#renderable);
    _wr_transform_attach_child(this.wrenNode, this.#transform);

    this._applyFrustumToWren();
  }

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      _wr_node_delete(this.#transform);
      _wr_node_delete(this.#renderable);
      _wr_material_delete(this.#material);
      _wr_static_mesh_delete(this.#mesh);
    }

    super.delete();
  }

  _applyFrustumToWren() {
    if (typeof this.#mesh !== 'undefined') {
      _wr_static_mesh_delete(this.#mesh);
      this.#mesh = undefined;
    }

    if (!this._isFrustumEnabled)
      return;

    const enabledColor = _wrjs_array3(0, 0, 1);
    _wr_phong_material_set_color(this.#material, enabledColor);

    const sinH = Math.sin(this.#horizontalFieldOfView / 2);
    const cosH = Math.cos(this.#horizontalFieldOfView / 2);
    const sinV = Math.sin(this.#verticalFieldOfView / 2);
    const cosV = Math.cos(this.#verticalFieldOfView / 2);
    const factorX = cosV * cosH;
    const factorY = cosV * sinH;
    const factorZ = sinV;
    const maxX = this.#maxRange * factorX;
    const maxZ = this.#maxRange * factorZ;
    const maxY = this.#maxRange * factorY;
    const minX = this.#minRange * factorX;
    const minZ = this.#minRange * factorZ;
    const minY = this.#minRange * factorY;

    // create the outlines
    const steps = 24;
    const vertices = [];

    this.#addVertex(vertices, 0, 0, 0);
    this.#addVertex(vertices, this.#minRange, 0, 0);
    this.#addVertex(vertices, minX, minY, -minZ);
    this.#addVertex(vertices, maxX, maxY, -maxZ);
    this.#addVertex(vertices, minX, -minY, -minZ);
    this.#addVertex(vertices, maxX, -maxY, -maxZ);
    this.#addVertex(vertices, minX, minY, minZ);
    this.#addVertex(vertices, maxX, maxY, maxZ);
    this.#addVertex(vertices, minX, -minY, minZ);
    this.#addVertex(vertices, maxX, -maxY, maxZ);

    // create top and bottom margin
    const ranges = [this.#minRange, this.#maxRange];
    for (let j = 0; j < steps; ++j) {
      const angle1 = this.#horizontalFieldOfView / 2 - j * this.#horizontalFieldOfView / steps;
      const angle2 = this.#horizontalFieldOfView / 2 - (j + 1) * this.#horizontalFieldOfView / steps;
      const factorX1 = cosV * Math.cos(angle1);
      const factorX2 = cosV * Math.cos(angle2);
      const factorY1 = cosV * Math.sin(angle1);
      const factorY2 = cosV * Math.sin(angle2);
      for (let k = 0; k < 2; ++k) {
        const range = ranges[k];
        const x1 = range * factorX1;
        const y1 = range * factorY1;
        const z1 = range * sinV;
        const x2 = range * factorX2;
        const y2 = range * factorY2;
        const z2 = range * sinV;
        // top
        this.#addVertex(vertices, x1, y1, z1);
        this.#addVertex(vertices, x2, y2, z2);
        // bottom
        this.#addVertex(vertices, x1, y1, -z1);
        this.#addVertex(vertices, x2, y2, -z2);
      }
    }

    // create right and left margin
    for (let j = 0; j < steps; ++j) {
      const angle1 = this.#verticalFieldOfView / 2 - j * this.#verticalFieldOfView / steps;
      const angle2 = this.#verticalFieldOfView / 2 - (j + 1) * this.#verticalFieldOfView / steps;
      const factorX1 = Math.cos(angle1) * cosH;
      const factorX2 = Math.cos(angle2) * cosH;
      const factorY1 = Math.cos(angle1) * sinH;
      const factorY2 = Math.cos(angle2) * sinH;
      const factorZ1 = Math.sin(angle1);
      const factorZ2 = Math.sin(angle2);
      for (let k = 0; k < 2; ++k) {
        const range = ranges[k];
        const x1 = range * factorX1;
        const z1 = range * factorZ1;
        const y1 = range * factorY1;
        const x2 = range * factorX2;
        const z2 = range * factorZ2;
        const y2 = range * factorY2;
        // right
        this.#addVertex(vertices, x1, -y1, z1);
        this.#addVertex(vertices, x2, -y2, z2);
        // left
        this.#addVertex(vertices, x1, y1, z1);
        this.#addVertex(vertices, x2, y2, z2);
      }
    }

    const verticesPointer = arrayXPointerFloat(vertices);
    this.#mesh = _wr_static_mesh_line_set_new(vertices.length / 3, verticesPointer, undefined);
    _wr_renderable_set_mesh(this.#renderable, this.#mesh);
    _free(verticesPointer);
  }

  applyOptionalRendering(enable) {
    this._isFrustumEnabled = enable;
    this._applyFrustumToWren();
    _wr_node_set_visible(this.#transform, enable);
  }

  #update() {
    if (this.wrenObjectsCreatedCalled)
      this._applyFrustumToWren();
  }

  #addVertex(vertices, x, y, z) {
    vertices.push(x);
    vertices.push(y);
    vertices.push(z);
  }
}
