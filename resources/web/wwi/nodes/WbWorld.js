export default class WbWorld {
  constructor() {
    this.hasFog = false;
    this.coordinateSystem = 'ENU';
    this.upVector = glm.vec3(0, 0, 1);
    this.root = undefined;
    // All the nodes are included here so it is easier to retrieve them for updates
    // map from id to node
    this.nodes = new Map();

    this.billboards = [];
    this.tracks = new Set();
    this.readyForUpdates = false;
    this.robots = [];
  }

  static init() {
    WbWorld.instance = new WbWorld();
    WbWorld.instance.version = '';
  }

  static computeUpVector() {
    const coords = WbWorld.instance.coordinateSystem;
    WbWorld.instance.upVector = glm.vec3(coords[0] === 'U' ? 1 : 0, coords[1] === 'U' ? 1 : 0, coords[2] === 'U' ? 1 : 0);
  }
}
