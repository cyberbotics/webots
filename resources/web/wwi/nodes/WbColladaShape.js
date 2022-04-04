import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {getAnId} from './utils/utils.js';

export default class WbColladaShape extends WbBaseNode {
  constructor(id, url, ccw, castShadows, isPickable) {
    super(id);

    this.url = url;
    this.ccw = ccw;
    this.castShadows = castShadows;
    this.isPickable = isPickable;

    this.children = [];
  }

  clone(customID) {
    const colladaShape = new WbColladaShape(customID, this.url, this.ccw, this.castShadows, this.isPickable);
    const length = this.children.length;
    for (let i = 0; i < length; i++) {
      const cloned = this.children[i].clone(getAnId());
      cloned.parent = customID;
      WbWorld.instance.nodes.set(cloned.id, cloned);
      colladaShape.children.push(cloned);
    }

    this.useList.push(customID);
    return colladaShape;
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.children.forEach(child => {
      child.createWrenObjects();
    });
  }

  delete() {
    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.sceneTree.indexOf(this);
      WbWorld.instance.sceneTree.splice(index, 1);
    } else {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        const index = parent.children.indexOf(this);
        parent.children.splice(index, 1);
      }
    }

    let index = this.children.length - 1;
    while (index >= 0) {
      this.children[index].delete();
      --index;
    }

    super.delete();
  }

  updateIsPickable() {
    this.children.forEach(child => child.updateIsPickable());
  }

  preFinalize() {
    super.preFinalize();

    this.children.forEach(child => child.preFinalize());
  }

  postFinalize() {
    super.postFinalize();

    this.children.forEach(child => child.postFinalize());
  }
}
