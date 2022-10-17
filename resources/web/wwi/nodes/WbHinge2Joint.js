import WbHingeJoint from './WbHingeJoint.js';

export default class WbHinge2Joint extends WbHingeJoint {
  #device2;
  #jointParameters2;
  constructor(id) {
    super(id);
    this.#device2 = [];
  }

  get device2() {
    return this.#device2;
  }

  set device2(device) {
    this.#device2 = device;
  }

  get jointParameters2() {
    return this.#jointParameters2;
  }

  set jointParameters2(jointParameters) {
    this.#jointParameters2 = jointParameters;
  }

  preFinalize() {
    super.preFinalize();
    this.#device2.forEach(child => child.preFinalize());
    this.#jointParameters2?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#device2.forEach(child => child.postFinalize());
    this.#jointParameters2?.postFinalize();
  }

  delete() {
    let index = this.#device2.length - 1;
    while (index >= 0) {
      this.#device2[index].delete();
      --index;
    }

    this.#jointParameters2?.delete();

    super.delete();
  }
}
