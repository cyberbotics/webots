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
      let index = parent.device?.indexOf(this);
      if (index !== -1)
        parent.device?.splice(index, 1);
      else {
        index = parent.device2?.indexOf(this);
        if (index !== -1)
          parent.device2?.splice(index, 1);
        else {
          index = parent.device3?.indexOf(this);
          parent.device3?.splice(index, 1);
        }
      }
    }

    super.delete();
  }
}
