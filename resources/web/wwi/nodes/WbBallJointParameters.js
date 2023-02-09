import WbJointParameters from './WbJointParameters.js';

export default class WbBallJointParameters extends WbJointParameters {
  #anchor;
  constructor(id, position, axis, anchor, minStop, maxStop) {
    super(id, position, axis, minStop, maxStop);
    this.#anchor = anchor;
  }

  get anchor() {
    return this.#anchor;
  }
}
