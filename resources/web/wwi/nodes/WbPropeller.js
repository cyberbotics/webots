import WbGroup from './WbGroup.js';

export default class WbPropeller extends WbGroup {
  #device;
  constructor(id) {
    super(id);
    this.currentHelix = -1; // to switch between fast and slow helix
    this.#device = [];
  }

  get device() {
    return this.#device;
  }

  set device(device) {
    this.#device = device;
  }

  postFinalize() {
    super.postFinalize();

    if (typeof this.children[1] !== 'undefined')
      this.currentHelix = this.children[1].id;
    else if (typeof this.children[0] !== 'undefined')
      this.currentHelix = this.children[0].id;
    this.switchHelix(this.currentHelix, true);
  }

  switchHelix(id, force) {
    if (id !== this.currentHelix || force) {
      this.currentHelix = id;
      this.children.forEach(child => {
        _wr_node_set_visible(child.wrenNode, child.id === this.currentHelix);
      });
    }
  }
}
