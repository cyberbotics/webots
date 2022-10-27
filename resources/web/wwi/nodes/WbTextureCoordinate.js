import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';

// This class is used to retrieve the type of device
export default class WbTextureCoordinate extends WbBaseNode {
  #point;
  constructor(id, point) {
    super(id);
    this.#point = point;
  }

  get point() {
    return this.#point;
  }

  set point(newPoint) {
    this.#point = newPoint;

    if (this.wrenObjectsCreatedCalled)
      this.#updatePoint();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbTextureCoordinate(customID, this.#point);
  }

  delete() {
    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined')
        parent.texCoord = undefined;
    }

    super.delete();
  }

  #updatePoint() {
    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
