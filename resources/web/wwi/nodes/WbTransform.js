import WbPose from './WbPose.js';
import WbWorld from './WbWorld.js';

import { getAnId } from './utils/id_provider.js';
import { WbNodeType } from './wb_node_type.js';

export default class WbTransform extends WbPose {
  #scale;
  constructor(id, translation, rotation, scale) {
    super(id, translation, rotation);
    this.#scale = scale;
    this._absoluteScaleNeedUpdate = true;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_TRANSFORM;
  }

  get scale() {
    return this.#scale;
  }

  set scale(newScale) {
    this.#scale = newScale;

    this.#updateScale();
  }

  absoluteScale() {
    if (this._absoluteScaleNeedUpdate)
      this._updateAbsoluteScale();

    return this._absoluteScale;
  }

  _updateAbsoluteScale() {
    this._absoluteScale = this.#scale;
    // multiply with upper transform scale if any
    const up = this.upperTransform;
    if (typeof up !== 'undefined')
      this._absoluteScale = this._absoluteScale.mulByVector(up.absoluteScale());

    this._absoluteScaleNeedUpdate = false;
  }

  clone(customID) {
    const transform = new WbTransform(customID, this.translation, this.rotation, this.#scale);

    const length = this.children.length;
    for (let i = 0; i < length; i++) {
      const cloned = this.children[i].clone(getAnId());
      cloned.parent = customID;
      WbWorld.instance.nodes.set(cloned.id, cloned);
      transform.children.push(cloned);
    }

    this.useList.push(customID);
    return transform;
  }

  vrmlMatrix() {
    if (this._vrmlMatrixNeedUpdate) {
      this._vrmlMatrix.fromVrml(this.translation, this.rotation, this.#scale);
      this._vrmlMatrixNeedUpdate = false;
    }

    return this._vrmlMatrix;
  }

  createWrenObjects() {
    super.createWrenObjects();
    this.#applyScaleToWren();
  }

  #applyScaleToWren() {
    const scale = _wrjs_array3(this.#scale.x, this.#scale.y, this.#scale.z);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  #updateScale() {
    if (this.wrenObjectsCreatedCalled)
      this.#applyScaleToWren();

    this._matrixNeedUpdate = true;
  }
}
