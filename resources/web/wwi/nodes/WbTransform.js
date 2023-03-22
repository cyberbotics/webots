import WbPose from './WbPose.js';
import WbWorld from './WbWorld.js';

import { getAnId } from './utils/id_provider.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import { WbNodeType } from './wb_node_type.js';

export default class WbTransform extends WbPose {
  #scale;
  constructor(id, translation, scale, rotation) {
    super(id, translation, rotation);
    this.#scale = scale;
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

  #updateAbsoluteScale() {
    this.absoluteScale = this.#scale;
    // multiply with upper transform scale if any
    const up = this.upperPose();
    if (typeof up !== 'undefined')
      this.absoluteScale = this.#absoluteScale.mulByVector(up.absoluteScale());

    this.#absoluteScaleNeedUpdate = false;
  }

  clone(customID) {
    const transform = new WbTransform(customID, this.translation, this.#scale, this.rotation);

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

  createWrenObjects() {
    super.createWrenObjects(true);
    this.#applyScaleToWren();
  }

  delete(isBoundingObject) {
    if (this.wrenObjectsCreatedCalled)
      _wr_node_delete(this.wrenNode);

    super.delete(isBoundingObject);
  }

  matrix() {
    if (typeof this.#matrix === 'undefined') {
      this.#matrix = new WbMatrix4();
      this.#updateMatrix();
    } else if (this.#matrixNeedUpdate)
      this.#updateMatrix();

    return this.#matrix;
  }

  #applyScaleToWren() {
    const scale = _wrjs_array3(this.#scale.x, this.#scale.y, this.#scale.z);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  #updateScale() {
    if (this.wrenObjectsCreatedCalled)
      this.#applyScaleToWren();

    this.#matrixNeedUpdate = true;
  }
}
