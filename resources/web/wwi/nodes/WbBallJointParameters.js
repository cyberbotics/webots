import WbJointParameters from './WbJointParameters.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbBallJointParameters extends WbJointParameters {
  #anchor;
  constructor(id, position, axis, anchor, minStop, maxStop) {
    super(id, position, axis, minStop, maxStop);
    this.#anchor = anchor;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_BALL_JOINT_PARAMETERS;
  }

  get anchor() {
    return this.#anchor;
  }
}
