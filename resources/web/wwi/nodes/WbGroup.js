import WbBaseNode from './WbBaseNode.js';
import WbCadShape from './WbCadShape.js';
import WbLight from './WbLight.js';
import WbWorld from './WbWorld.js';
import {getAnId} from './utils/id_provider.js';

export default class WbGroup extends WbBaseNode {
  #device;
  constructor(id, isPropeller) {
    super(id);
    this.children = [];

    this.isPropeller = isPropeller;
    this.currentHelix = -1; // to switch between fast and slow helix
    if (isPropeller)
      this.#device = [];
  }

  get device() {
    return this.#device;
  }

  set device(device) {
    this.#device = device;
  }

  clone(customID) {
    const group = new WbGroup(customID, this.isPropeller);
    const length = this.children.length;
    for (let i = 0; i < length; i++) {
      const cloned = this.children[i].clone(getAnId());
      cloned.parent = customID;
      WbWorld.instance.nodes.set(cloned.id, cloned);
      group.children.push(cloned);
    }

    this.useList.push(customID);
    return group;
  }

  createWrenObjects(isTransform) {
    super.createWrenObjects();

    if (!isTransform) {
      this.children.forEach((child, i) => {
        if (typeof this.loadProgress !== 'undefined') {
          this.loadProgress++;
          const percentage = this.loadProgress * 100 / (3 * this.children.length) //70 + 30 * (i + 1) / this.children.length;
          const info = 'Create WREN object ' + child.id + ': ' + Math.round(percentage) + '%';
          WbWorld.instance.currentView.progress.setProgressBar('block', 'same', percentage, info);
        }
        child.createWrenObjects();
      });
    }
  }

  delete(isBoundingObject) {
    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.root.children.indexOf(this);
      WbWorld.instance.root.children.splice(index, 1);
    } else {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        if (isBoundingObject)
          parent.isBoundingObject = undefined;
        else if (typeof parent.endPoint !== 'undefined')
          parent.endPoint = undefined;
        else {
          const index = parent.children.indexOf(this);
          parent.children.splice(index, 1);
        }
      }
    }

    let index = this.children.length - 1;
    while (index >= 0) {
      this.children[index].delete();
      --index;
    }

    super.delete();
  }

  preFinalize() {
    super.preFinalize();

    if (this === WbWorld.instance.root)
      this.loadProgress = 0;

    this.children.forEach((child, i) => {
      if (typeof this.loadProgress !== 'undefined') {
        this.loadProgress++;
        const percentage = this.loadProgress * 100 / (3 * this.children.length) //70 + 30 * (i + 1) / this.children.length;
        const info = 'Pre-finalize node ' + child.id + ': ' + Math.round(percentage) + '%';
        WbWorld.instance.currentView.progress.setProgressBar('block', 'same', percentage, info);
      }
      child.preFinalize();
    });
  }

  postFinalize() {
    super.postFinalize();

    this.children.forEach((child, i) => {
      if (typeof this.loadProgress !== 'undefined') {
        this.loadProgress++;
        const percentage = this.loadProgress * 100 / (3 * this.children.length) //70 + 30 * (i + 1) / this.children.length;
        const info = 'Post-finalize node ' + child.id + ': ' + Math.round(percentage) + '%';
        WbWorld.instance.currentView.progress.setProgressBar('block', 'same', percentage, info);
      }
      child.postFinalize();
    });

    if (this.isPropeller === true) {
      if (typeof this.children[1] !== 'undefined')
        this.currentHelix = this.children[1].id;
      else if (typeof this.children[0] !== 'undefined')
        this.currentHelix = this.children[0].id;
      this.switchHelix(this.currentHelix, true);
    }
  }

  switchHelix(id, force) {
    if (id !== this.currentHelix || force) {
      this.currentHelix = id;
      this.children.forEach(child => {
        if (child.id === this.currentHelix)
          _wr_node_set_visible(child.wrenNode, true);
        else
          _wr_node_set_visible(child.wrenNode, false);
      });
    }
  }

  updateBoundingObjectVisibility() {
    this.children.forEach(child => {
      if (!(child instanceof WbLight || child instanceof WbCadShape))
        child.updateBoundingObjectVisibility();
    });
  }
}
