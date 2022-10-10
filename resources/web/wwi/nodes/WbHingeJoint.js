import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {getAnId} from './utils/id_provider.js';

export default class WbHingeJoint extends WbBaseNode {
  constructor(id, endPoint) {
    super(id);
    this.endPoint = endPoint;
  }

  clone(customID) {
    const hingeJoint = new WbHingeJoint(customID);

    if (typeof this.endPoint !== 'undefined') {
      hingeJoint.endPoint = this.endPoint.clone(getAnId());
      hingeJoint.endPoint.parent = customID;
      WbWorld.instance.nodes.set(hingeJoint.id, hingeJoint);
    }

    this.useList.push(customID);
    return hingeJoint;
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.endPoint?.createWrenObjects();
  }

  delete() {
    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.sceneTree.indexOf(this);
      WbWorld.instance.sceneTree.splice(index, 1);
    } else {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        if (typeof parent.endPoint !== 'undefined')
          parent.endPoint = undefined;
        else {
          const index = parent.children.indexOf(this);
          parent.children.splice(index, 1);
        }
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
}
