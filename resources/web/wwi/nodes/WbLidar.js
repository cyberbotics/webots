import WbAbstractCamera from './WbAbstractCamera.js';

// This class is used to retrieve the type of device
export default class WbLidar extends WbAbstractCamera {
  #frustumMaterial;
  #frustumMesh;
  #frustumRenderable;
  #maxRange;
  #minRange;
  #numberOfLayers;
  #tiltAngle;
  #verticalFieldOfView;
  constructor(id, translation, scale, rotation, name, fieldOfView, maxRange, minRange, numberOfLayers, tiltAngle,
    verticalFieldOfView) {
    super(id, translation, scale, rotation, name, undefined, undefined, fieldOfView);
    this.#maxRange = maxRange;
    this.#minRange = minRange;
    this.#numberOfLayers = numberOfLayers;
    this.#tiltAngle = tiltAngle;
    this.#verticalFieldOfView = verticalFieldOfView;
  }

  get maxRange() {
    return this.#maxRange;
  }

  set maxRange(newMaxRange) {
    this.#maxRange = newMaxRange;
    this._update();
  }

  get minRange() {
    return this.#minRange;
  }

  set minRange(newMinRange) {
    this.#minRange = newMinRange;
    this._update();
  }

  _applyFrustumToWren() {
    _wr_node_set_visible(this.#frustumRenderable, false);
    _wr_static_mesh_delete(this.#frustumMesh);
    this.#frustumMesh = undefined;

    if (!this._isFrustumEnabled)
      return;

    const frustumColor = [1, 0, 1];
    const frustumColorRgb = _wrjs_array3(frustumColor[0], frustumColor[1], frustumColor[2]);
    _wr_phong_material_set_color(this.#frustumMaterial, frustumColorRgb);

    let i = 0;
    const n = this.#minRange;
    const f = this.#maxRange;
    let fovH = this.fieldOfView;

    const intermediatePointsNumber = Math.floor(fovH / 0.2);
    const vertexCount = 4 * this.#numberOfLayers * (intermediatePointsNumber + 3);
    const vertices = [];

    for (let layer = 0; layer < this.#numberOfLayers; layer++) {
      let vAngle = 0;
      if (this.#numberOfLayers > 1)
        vAngle = this.#verticalFieldOfView / 2.0 - (layer / (this.#numberOfLayers - 1.0)) * this.#verticalFieldOfView +
        this.#tiltAngle;
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

    this.#frustumMesh = _wr_static_mesh_line_set_new(vertexCount, vertices, undefined);
    _wr_renderable_set_mesh(this.#frustumRenderable, this.#frustumMesh);
    _wr_node_set_visible(this.#frustumRenderable, true);
  }

  #pushVertex(vertices, index, x, y, z) {
    vertices[3 * index] = x;
    vertices[3 * index + 1] = y;
    vertices[3 * index + 2] = z;
  }
}
