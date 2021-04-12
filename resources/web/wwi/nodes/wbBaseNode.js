import {findUpperTransform, nodeIsInBoundingObject} from './wbUtils.js';
import {WbWorld} from './wbWorld.js';

class WbBaseNode {
  constructor(id) {
    this.id = id;

    this.wrenObjectsCreatedCalled = false;
    this.isPreFinalizeCalled = false;
    this.isPostFinalizeCalled = false;

    this.upperTransformFirstTimeSearch = true;
    this.upperTransform = false;

    this.boundingObjectFirstTimeSearch = true;
    this.isInBoundingObject = false;

    this.useList = [];
  }

  delete() {
    if (this.useList.length !== 0) {
      let newDef = undefined;
      let index = 0;
      while (typeof newDef === 'undefined' && index < this.useList.length) {
        newDef = WbWorld.instance.nodes.get(this.useList[index]);
        this.useList.splice(index, 1);
        index++;
      }

      if (typeof newDef !== 'undefined') {
        newDef.useList = this.useList;
        console.log(newDef);
      }
    }

    WbWorld.instance.nodes.delete(this.id);
  }

  createWrenObjects() {
    this.wrenObjectsCreatedCalled = true;

    if (typeof this.parent !== 'undefined')
      this.wrenNode = WbWorld.instance.nodes.get(this.parent).wrenNode;
    else
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
  }

  upperTransform() {
    if (this.upperTransformFirstTimeSearch) {
      this.upperTransform = findUpperTransform(this);
      if (this.wrenObjectsCreatedCalled)
        this.upperTransformFirstTimeSearch = false;
    }

    return this.upperTransform;
  }

  isInBoundingObject() {
    if (this.boundingObjectFirstTimeSearch) {
      this.isInBoundingObject = nodeIsInBoundingObject(this);
      if (this.wrenObjectsCreatedCalled)
        this.boundingObjectFirstTimeSearch = false;
    }

    return this.isInBoundingObject;
  }

  finalize() {
    if (!this.isPreFinalizeCalled)
      this.preFinalize();

    if (!this.wrenObjectsCreatedCalled)
      this.createWrenObjects();

    if (!this.isPostFinalizeCalled)
      this.postFinalize();
  }

  preFinalize() {
    this.isPreFinalizeCalled = true;
  }

  postFinalize() {
    this.isPostFinalizeCalled = true;
  }
}

export {WbBaseNode};
