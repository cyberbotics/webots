import {World} from "./World.js"

class WbBaseNode {
  constructor(id){
    this.id = id;
    this.parent = undefined;
    this.wrenNode = undefined;

    this.wrenObjectsCreatedCalled = false;
    this.preFinalizeCalled = false;
    this.postFinalizeCalled = false;
  }

  createWrenObjects() {
    this.wrenObjectsCreatedCalled = true;

    if (typeof this.parent !== 'undefined') {
      this.wrenNode = World.instance.nodes[this.parent].wrenNode;
    } else{
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
    }
  }
}

export{WbBaseNode}
