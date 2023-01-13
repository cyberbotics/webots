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
  }

  unfinalize() {
    this.isPreFinalizedCalled = false;
    this.wrenObjectsCreatedCalled = false;
    this.isPostFinalizedCalled = false;
    for (const useId of this.useList) {
      const useNode = WbWorld.instance.nodes.get(useId);
      useNode?.unfinalize();
    }
  }

  finalize() {
    if (!this.isPreFinalizedCalled)
      this.preFinalize();

    if (!this.wrenObjectsCreatedCalled)
      this.createWrenObjects();

    if (!this.isPostFinalizedCalled)
      this.postFinalize();

    for (const useId of this.useList) {
      const useNode = WbWorld.instance.nodes.get(useId);
      console.log('notifying', useNode.id)
      useNode.finalize();
    }
  }

  preFinalize() {
    this.isPreFinalizedCalled = true;
  }

  postFinalize() {
    this.isPostFinalizedCalled = true;
  }

  boundingSphere() {}
}
