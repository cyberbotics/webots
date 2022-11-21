import WbWorld from './WbWorld.js';

export default class WbBaseNode {
  constructor(id) {
    this.id = id;

    this.wrenObjectsCreatedCalled = false;
    this.isPreFinalizedCalled = false;
    this.isPostFinalizedCalled = false;

    this.useList = [];
  }

  createWrenObjects() {
    //if (this.wrenObjectsCreatedCalled)
    //  return;

    this.wrenObjectsCreatedCalled = true;

    if (typeof this.parent !== 'undefined')
      this.wrenNode = WbWorld.instance.nodes.get(this.parent).wrenNode;
    else
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
  }

  delete() {
    console.log('delete basenode of ', this.id)
    /*
    if (this.useList.length !== 0) {
      let newDef;
      let index = 0;
      while (typeof newDef === 'undefined' && index < this.useList.length) {
        newDef = WbWorld.instance.nodes.get(this.useList[index]);
        this.useList.splice(index, 1);
        index++;
      }

      if (typeof newDef !== 'undefined')
        newDef.useList = this.useList;
    }

    WbWorld.instance.nodes.delete(this.id);
    */
    WbWorld.instance.nodes.delete(this.id);
  }

  finalize() {
    if (!this.isPreFinalizedCalled)
      this.preFinalize();

    if (!this.wrenObjectsCreatedCalled)
      this.createWrenObjects();

    if (!this.isPostFinalizedCalled)
      this.postFinalize();
  }

  preFinalize() {
    this.isPreFinalizedCalled = true;
  }

  postFinalize() {
    this.isPostFinalizedCalled = true;
  }
}
