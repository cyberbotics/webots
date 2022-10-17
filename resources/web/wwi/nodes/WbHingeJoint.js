import WbJoint from './WbJoint.js';

export default class WbHingeJoint extends WbJoint {
  #device;
  constructor(id) {
    super(id);
    this.#device = [];
  }

  get device() {
    return this.#device;
  }

  set device(device) {
    this.#device = device;
  }

  preFinalize() {
    super.preFinalize();
    this.#device.forEach(child => child.preFinalize());
  }

  postFinalize() {
    super.postFinalize();

    this.#device.forEach(child => child.postFinalize());
  }

  delete() {
    let index = this.#device.length - 1;
    while (index >= 0) {
      this.#device[index].delete();
      --index;
    }

    super.delete();
  }
}
