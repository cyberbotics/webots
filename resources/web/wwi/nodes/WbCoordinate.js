import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbCoordinate extends WbBaseNode {
  #point;
  constructor(id, point) {
    super(id);
    this.#point = point;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_COORDINATE;
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
    return new WbCoordinate(customID, this.#point);
  }

  delete() {
    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined')
        parent.coord = undefined;
    }

    super.delete();
  }

  #updatePoint() {
    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
