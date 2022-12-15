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
    this.wrenObjectsCreatedCalled = true;

    if (typeof this.parent !== 'undefined')
      this.wrenNode = WbWorld.instance.nodes.get(this.parent).wrenNode;
    else
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
  }

  delete() {
    WbWorld.instance.nodes.delete(this.id);

    //for (const useId of this.useList) {
    //  const useNode = WbWorld.instance.nodes.get(useId);
    //  useNode?.delete();
    //}
  }

  finalize() {
    console.log('FINALIZE', this.id)
    if (!this.isPreFinalizedCalled)
      this.preFinalize();

    if (!this.wrenObjectsCreatedCalled)
      this.createWrenObjects();

    if (!this.isPostFinalizedCalled)
      this.postFinalize();

    for (const useId of this.useList) {
      const useNode = WbWorld.instance.nodes.get(useId);
      useNode.finalize();
    }
  }

  preFinalize() {
    this.isPreFinalizedCalled = true;
  }

  postFinalize() {
    this.isPostFinalizedCalled = true;
  }
}
