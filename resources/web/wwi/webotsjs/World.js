class World {
  constructor () {
    //We can keep the viewpoint outside of the children list because
    //it is independant from Webots during the simulation so we don't need to retrieve it to apply update
    this.viewpoint = undefined;
    //Only the top level nodes are represented here
    this.sceneTree = [];

    //All the nodes are included here so it is easier to retrieve them for update
    //dict from id to node
    this.nodes = {}

    World.instance = this;
  }
}

World.instance = undefined;
export {World}
