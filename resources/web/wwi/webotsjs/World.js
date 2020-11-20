class World {
  constructor () {
    this.viewpoint = undefined;
    this.background = undefined;
    this.children = [];

    World.instance = this;
  }
}

World.instance = undefined;
export {World}
