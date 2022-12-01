import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {getAnId} from './utils/id_provider.js';

export default class WbSlot extends WbBaseNode {
  constructor(id, type) {
    super(id);
    this.type = type;
  }

  clone(customID) {
    const slot = new WbSlot(customID, this.type);

    if (typeof this.endPoint !== 'undefined') {
      slot.endPoint = this.endPoint.clone(getAnId());
      slot.endPoint.parent = customID;
      WbWorld.instance.nodes.set(slot.id, slot);
    }

    this.useList.push(customID);
    return slot;
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.endPoint?.createWrenObjects();
  }

  delete() {
    const parent = WbWorld.instance.nodes.get(this.parent);
    if (typeof parent !== 'undefined') {
      if (typeof parent.endPoint !== 'undefined')
        parent.endPoint = undefined;
      else {
        const index = parent.children.indexOf(this);
        parent.children.splice(index, 1);
      }
    }

    this.endPoint?.delete();

    super.delete();
  }

  preFinalize() {
    super.preFinalize();

    this.endPoint?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.endPoint?.postFinalize();
  }

  updateBoundingObjectVisibility() {
    this.endPoint?.updateBoundingObjectVisibility();
  }
}
