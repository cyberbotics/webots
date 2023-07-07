import WbBaseNode from './WbBaseNode.js';
import WbSolid from './WbSolid.js';
import WbWorld from './WbWorld.js';
import {getAnId} from './utils/id_provider.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbSlot extends WbBaseNode {
  constructor(id, type) {
    super(id);
    this.type = type;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_SLOT;
  }

  boundingSphere() {
    return this.endPoint?.boundingSphere();
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

  solidEndPoint() {
    if (typeof this.endPoint !== 'undefined' && this.endPoint instanceof WbSolid)
      return this.endPoint;
  }

  slotEndPoint() {
    if (typeof this.endPoint !== 'undefined' && this.endPoint.nodeType === WbNodeType.WB_NODE_SLOT)
      return this.endPoint;
  }

  updateBoundingObjectVisibility() {
    this.endPoint?.updateBoundingObjectVisibility();
  }
}
