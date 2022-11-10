import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';

export default class WbJointParameters extends WbBaseNode {
  #position;
  #axis;
  #minStop;
  #maxStop;
  constructor(id, position, axis, minStop, maxStop) {
    super(id);
    this.#position = position;
    this.#axis = axis;
    this.#minStop = minStop;
    this.#maxStop = maxStop;
  }

  get position() {
    return this.#position;
  }

  set position(newPosition) {
    this.#position = newPosition;

    if (typeof this.onChange === 'function')
      this.onChange();
  }

  get axis() {
    return this.#axis;
  }

  get minStop() {
    return this.#minStop;
  }

  get maxStop() {
    return this.#maxStop;
  }

  delete() {
    const parent = WbWorld.instance.nodes.get(this.parent);
    if (typeof parent !== 'undefined') {
      if (typeof parent.jointParameters !== 'undefined')
        parent.jointParameters = undefined;
    }

    super.delete();
  }
}
