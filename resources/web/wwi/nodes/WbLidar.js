import WbSolid from './WbSolid.js';

// This class is used to retrieve the type of device
export default class WbLidar extends WbSolid {
  #frustumMaterial;
  #frustumMesh;
  #frustumRenderable;
  #maxRange;
  #minRange;
  constructor(id, translation, scale, rotation, name, fieldOfView, maxRange, minRange, numberOfLayers, tiltAngle, verticalFieldOfView) {
    super(id, translation, scale, rotation, name, undefined, undefined, fieldOfView);
    this.#maxRange = maxRange;
    this.#minRange = minRange;
    this.#numberOfLayers = numberOfLayers;
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
        vAngle = this.#verticalFieldOfView / 2.0 - (layer / (this.#numberOfLayers - 1.0)) * this.#verticalFieldOfView + this.#tiltAngle;
      const cosV = cos(vAngle);
      const sinV = sin(vAngle);
      pushVertex(vertices, i++, 0, 0, 0);
      // min range
      for (int j = 0; j < intermediatePointsNumber + 2; ++j) {
        const double tmpHAngle = fovH / 2.0 - fovH * j / (intermediatePointsNumber + 1);
        const double x = n * cos(tmpHAngle) * cosV;
        const double y = n * sin(tmpHAngle) * cosV;
        const double z = n * sinV;
        pushVertex(vertices, i++, x, y, z);
        pushVertex(vertices, i++, x, y, z);
      }

      pushVertex(vertices, i++, 0, 0, 0);
      pushVertex(vertices, i++, 0, 0, 0);

      // max range
      for (int j = 0; j < intermediatePointsNumber + 2; ++j) {
        const double tmpHAngle = fovH / 2.0 - fovH * j / (intermediatePointsNumber + 1);
        const double x = f * cos(tmpHAngle) * cosV;
        const double y = f * sin(tmpHAngle) * cosV;
        const double z = f * sinV;
        pushVertex(vertices, i++, x, y, z);
        pushVertex(vertices, i++, x, y, z);
      }
      pushVertex(vertices, i++, 0, 0, 0);
    }

    mFrustumMesh = wr_static_mesh_line_set_new(vertexCount, vertices, NULL);
    wr_renderable_set_mesh(mFrustumRenderable, WR_MESH(mFrustumMesh));
    wr_node_set_visible(WR_NODE(mFrustumRenderable), true);
  }
}
