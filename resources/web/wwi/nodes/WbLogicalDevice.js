import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';

export default class WbLogicalDevice extends WbBaseNode {
  #deviceName;
  constructor(id, deviceName) {
    super(id);
    this.#deviceName = deviceName;
  }

  get deviceName() {
    return this.#deviceName;
  }

  delete() {
    const parent = WbWorld.instance.nodes.get(this.parent);
    if (typeof parent !== 'undefined') {
      const index = parent?.device.indexOf(this);
      parent?.device.splice(index, 1);
    }

    super.delete();
  }
}
