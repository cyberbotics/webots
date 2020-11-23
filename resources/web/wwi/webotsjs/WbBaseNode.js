import {World} from "./World.js"

class WbBaseNode {
  constructor(id){
    this.id = id;
    this.parent = undefined;
    this.wrenNode = undefined;

    this.wrenObjectsCreatedCalled = false;
    this.isPreFinalizeCalled = false;
    this.isPostFinalizeCalled = false;
  }

  createWrenObjects() {
    this.wrenObjectsCreatedCalled = true;

    if (typeof this.parent !== 'undefined') {
      this.wrenNode = World.instance.nodes[this.parent].wrenNode;
    } else{
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
    }
  }

  finalize() {
    if (!this.isPreFinalizeCalled)
      this.preFinalize();

    if (!this.wrenObjectsCreatedCalled)
      this.createWrenObjects();

    if (!this.isPostFinalizedCalled)
      this.postFinalize();

    //emit finalizationCompleted(this);
  }

  preFinalize() {
    this.preFinalizeCalled = true;
  }

  postFinalize() {
    this.postFinalizeCalled = true;
  }
}

export{WbBaseNode}
