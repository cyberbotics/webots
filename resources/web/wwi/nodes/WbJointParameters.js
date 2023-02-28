import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {WbNodeType} from './wb_node_type.js';

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

  get nodeType() {
    return WbNodeType.WB_NODE_JOINT_PARAMETERS;
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
      if (parent.jointParameters?.id === this.id)
        parent.jointParameters = undefined;
      else if (parent.jointParameters2?.id === this.id)
        parent.jointParameters2 = undefined;
      else if (parent.jointParameters3?.id === this.id)
        parent.jointParameters3 = undefined;
    }

    super.delete();
  }
}
