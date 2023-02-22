import WbJointParameters from './WbJointParameters.js';

export default class WbHingeJointParameters extends WbJointParameters {
  #anchor;
  constructor(id, position, axis, anchor, minStop, maxStop) {
    super(id, position, axis, minStop, maxStop);
    this.#anchor = anchor;
  }

  get anchor() {
    return this.#anchor;
  }
}
