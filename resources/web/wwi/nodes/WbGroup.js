import WbBaseNode from './WbBaseNode.js';
import WbLight from './WbLight.js';
import WbSolid from './WbSolid.js';
import WbWorld from './WbWorld.js';
import WbBoundingSphere from './utils/WbBoundingSphere.js';
import { getAnId } from './utils/id_provider.js';
import { nodeIsInBoundingObject } from './utils/node_utilities.js';
import { WbNodeType } from './wb_node_type.js';

export default class WbGroup extends WbBaseNode {
  #boundingObjectFirstTimeSearch;
  #isInBoundingObject;
  constructor(id, isPropeller) {
    super(id);
    this.children = [];

    this.#boundingObjectFirstTimeSearch = true;
    this.#isInBoundingObject = false;
  }

  _updateProgress(message, element) {
    if (typeof this.loadProgress !== 'undefined') {
      this.loadProgress++;
      const percentage = this.loadProgress * 100 / (3 * this.children.length);
      const info = message + ' ' + element.id + ': ' + percentage.toFixed(0) + '%';
      WbWorld.instance.currentView.progress.setProgressBar('block', 'same', percentage, info);
    }
  }

  get nodeType() {
    return WbNodeType.WB_NODE_GROUP;
  }

  boundingSphere() {
    return this._boundingSphere;
  }

  clone(customID) {
    const group = new WbGroup(customID);
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

  createWrenObjects(isPose) {
    super.createWrenObjects();

    if (!isPose) {
      this.children.forEach(child => {
        this._updateProgress('Create WREN object', child);
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

        if (parent instanceof WbSolid && this.isInBoundingObject())
          parent.boundingObject = undefined;
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

    if (this === WbWorld.instance.root) {
      this.loadProgress = 0;
      WbWorld.instance.currentView.progress.setProgressBar('block', 'same', 0, 'Finalizing...');
    }

    this.children.forEach((child, i) => {
      this._updateProgress('Pre-finalize node', child);
      child.preFinalize();
    });
  }

  postFinalize() {
    super.postFinalize();

    this.children.forEach((child, i) => {
      this._updateProgress('Post-finalize node', child);
      child.postFinalize();
    });

    this.recomputeBoundingSphere();
  }

  recomputeBoundingSphere() {
    this._boundingSphere = new WbBoundingSphere(this);
    this._boundingSphere.empty();

    this.children.forEach(child => {
      if (!child.isPostFinalizedCalled)
        child.postFinalize();

      this._boundingSphere.addSubBoundingSphere(child.boundingSphere());
    });
  }

  isInBoundingObject() {
    if (this.#boundingObjectFirstTimeSearch) {
      this.#isInBoundingObject = nodeIsInBoundingObject(this);
      if (this.wrenObjectsCreatedCalled)
        this.#boundingObjectFirstTimeSearch = false;
    }

    return this.#isInBoundingObject;
  }

  updateBoundingObjectVisibility() {
    this.children.forEach(child => {
      if (!(child instanceof WbLight || child.nodeType === WbNodeType.WB_NODE_CAD_SHAPE))
        child.updateBoundingObjectVisibility();
    });
  }
}
