import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbNormal extends WbBaseNode {
  #vector;
  constructor(id, vector) {
    super(id);
    this.#vector = vector;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_NORMAL;
  }

  get vector() {
    return this.#vector;
  }

  set vector(newVector) {
    this.#vector = newVector;

    if (this.wrenObjectsCreatedCalled)
      this.#updateVector();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbNormal(customID, this.#vector);
  }

  delete() {
    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined')
        parent.normal = undefined;
    }

    super.delete();
  }

  #updateVector() {
    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
