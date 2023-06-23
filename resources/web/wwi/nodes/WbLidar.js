import WbAbstractCamera from './WbAbstractCamera.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import {arrayXPointerFloat} from './utils/utils.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbLidar extends WbAbstractCamera {
  #frustumMaterial;
  #frustumMesh;
  #frustumRenderable;
  #horizontalResolution;
  #maxRange;
  #minRange;
  #numberOfLayers;
  #tiltAngle;
  #verticalFieldOfView;
  constructor(id, translation, rotation, name, fieldOfView, maxRange, minRange, numberOfLayers, tiltAngle,
    verticalFieldOfView, horizontalResolution) {
    super(id, translation, rotation, name, undefined, undefined, fieldOfView);
    this.#horizontalResolution = horizontalResolution;
    this.#maxRange = maxRange;
    this.#minRange = minRange;
    this.#numberOfLayers = numberOfLayers;
    this.#tiltAngle = tiltAngle;
    this.#verticalFieldOfView = verticalFieldOfView;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_LIDAR;
  }

  get horizontalResolution() {
    return this.#horizontalResolution;
  }

  set horizontalResolution(newHorizontalResolution) {
    if (newHorizontalResolution <= 0)
      newHorizontalResolution = 1;

    this.#horizontalResolution = newHorizontalResolution;

    if (this.#numberOfLayers > 1 && this.height < this.#numberOfLayers)
      this.#horizontalResolution = Math.ceil((this.#numberOfLayers * this.fieldOfView) / this._verticalFieldOfView());

    this._update();
  }

  get maxRange() {
    return this.#maxRange;
  }

  set maxRange(newMaxRange) {
    if (this.#minRange > newMaxRange || newMaxRange <= 0)
      newMaxRange = this.#minRange + 1;

    this.#maxRange = newMaxRange;
    this._update();
  }

  get minRange() {
    return this.#minRange;
  }

  set minRange(newMinRange) {
    if (newMinRange <= 0)
      newMinRange = 0.01;
    if (newMinRange >= this.#maxRange)
      this.#maxRange += 1;

    this.#minRange = newMinRange;
    this._update();
  }

  get numberOfLayers() {
    return this.#numberOfLayers;
  }

  set numberOfLayers(newNumberOfLayers) {
    if (newNumberOfLayers <= 0)
      newNumberOfLayers = 1;

    this.#numberOfLayers = newNumberOfLayers;
    if (newNumberOfLayers !== 1) {
      const maxNumberOfLayers = this.height;

      if (newNumberOfLayers > maxNumberOfLayers)
        this.#numberOfLayers = maxNumberOfLayers;
    }

    this._update();
  }

  get tiltAngle() {
    return this.#tiltAngle;
  }

  set tiltAngle(newTiltAngle) {
    this.#tiltAngle = newTiltAngle;
    this._update();
  }

  get verticalFieldOfView() {
    return this.#verticalFieldOfView;
  }

  set verticalFieldOfView(newVerticalFieldOfView) {
    if (newVerticalFieldOfView > Math.PI)
      newVerticalFieldOfView = Math.PI;
    else if (newVerticalFieldOfView <= 0)
      newVerticalFieldOfView = 0.1;

    this.#verticalFieldOfView = newVerticalFieldOfView;
    if (this.#numberOfLayers > 1 && this.height < this.#numberOfLayers)
      this.#verticalFieldOfView = (this.#numberOfLayers * this.fieldOfView) / this.#horizontalResolution;

    this._update();
  }

  get height() {
    if (this.#numberOfLayers === 1)
      return 1;

    return Math.ceil((this.#verticalFieldOfView + this.fieldOfView / this.#horizontalResolution) *
      (this.#horizontalResolution / this.fieldOfView));
  }

  createWrenObjects() {
    // Lidar frustum
    this.#frustumMaterial = _wr_phong_material_new();
    _wr_material_set_default_program(this.#frustumMaterial, WbWrenShaders.lineSetShader());

    this.#frustumRenderable = _wr_renderable_new();
    _wr_renderable_set_cast_shadows(this.#frustumRenderable, false);
    _wr_renderable_set_receive_shadows(this.#frustumRenderable, false);
    _wr_renderable_set_visibility_flags(this.#frustumRenderable, WbWrenRenderingContext.VM_REGULAR);
    _wr_renderable_set_material(this.#frustumRenderable, this.#frustumMaterial, undefined);
    _wr_renderable_set_drawing_mode(this.#frustumRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
    _wr_node_set_visible(this.#frustumRenderable, false);

    super.createWrenObjects();

    _wr_transform_attach_child(this.wrenNode, this.#frustumRenderable);
  }

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      _wr_node_delete(this.#frustumRenderable);
      _wr_material_delete(this.#frustumMaterial);
      _wr_static_mesh_delete(this.#frustumMesh);
    }

    super.delete();
  }

  _applyFrustumToWren() {
    _wr_node_set_visible(this.#frustumRenderable, false);
    if (typeof this.#frustumMesh !== 'undefined') {
      _wr_static_mesh_delete(this.#frustumMesh);
      this.#frustumMesh = undefined;
    }

    if (!this._isFrustumEnabled)
      return;

    const frustumColorRgb = _wrjs_array3(0, 1, 1);
    _wr_phong_material_set_color(this.#frustumMaterial, frustumColorRgb);

    let i = 0;
    const n = this.#minRange;
    const f = this.#maxRange;
    const fovV = this._verticalFieldOfView();
    const fovH = this.fieldOfView;

    const intermediatePointsNumber = Math.floor(fovH / 0.2);
    const vertexCount = 4 * this.#numberOfLayers * (intermediatePointsNumber + 3);
    const vertices = [];

    for (let layer = 0; layer < this.#numberOfLayers; layer++) {
      let vAngle = 0;
      if (this.#numberOfLayers > 1)
        vAngle = fovV / 2.0 - (layer / (this.#numberOfLayers - 1.0)) * fovV + this.#tiltAngle;
      const cosV = Math.cos(vAngle);
      const sinV = Math.sin(vAngle);
      this.#pushVertex(vertices, i++, 0, 0, 0);
      // min range
      for (let j = 0; j < intermediatePointsNumber + 2; ++j) {
        const tmpHAngle = fovH / 2.0 - fovH * j / (intermediatePointsNumber + 1);
        const x = n * Math.cos(tmpHAngle) * cosV;
        const y = n * Math.sin(tmpHAngle) * cosV;
        const z = n * sinV;
        this.#pushVertex(vertices, i++, x, y, z);
        this.#pushVertex(vertices, i++, x, y, z);
      }

      this.#pushVertex(vertices, i++, 0, 0, 0);
      this.#pushVertex(vertices, i++, 0, 0, 0);

      // max range
      for (let j = 0; j < intermediatePointsNumber + 2; ++j) {
        const tmpHAngle = fovH / 2.0 - fovH * j / (intermediatePointsNumber + 1);
        const x = f * Math.cos(tmpHAngle) * cosV;
        const y = f * Math.sin(tmpHAngle) * cosV;
        const z = f * sinV;

        this.#pushVertex(vertices, i++, x, y, z);
        this.#pushVertex(vertices, i++, x, y, z);
      }
      this.#pushVertex(vertices, i++, 0, 0, 0);
    }

    const verticesPointer = arrayXPointerFloat(vertices);
    this.#frustumMesh = _wr_static_mesh_line_set_new(vertexCount, verticesPointer, undefined);
    _wr_renderable_set_mesh(this.#frustumRenderable, this.#frustumMesh);
    _wr_node_set_visible(this.#frustumRenderable, true);
    _free(verticesPointer);
  }

  #pushVertex(vertices, index, x, y, z) {
    vertices[3 * index] = x;
    vertices[3 * index + 1] = y;
    vertices[3 * index + 2] = z;
  }

  _verticalFieldOfView() {
    return this.fieldOfView * this.height / this.#horizontalResolution;
  }
}
