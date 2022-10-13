import WbHinge2Joint from './WbHinge2Joint.js';

export default class WbBallJoint extends WbHinge2Joint {
  #device3;
  #jointParameters3;
  constructor(id) {
    super(id);
    this.#device3 = [];
  }

  get device2() {
    return this.#device3;
  }

  set device2(device) {
    return this.#device3;
  }

  get jointParameters3() {
    return this.#jointParameters3;
  }

  set jointParameters3(jointParameters) {
    this.#jointParameters3 = jointParameters;
  }

  preFinalize() {
    super.preFinalize();
    this.#device3.forEach(child => child.preFinalize());
    this.#jointParameters3?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#device3.forEach(child => child.postFinalize());
    this.#jointParameters3?.postFinalize();
  }

  delete() {
    let index = this.#device3.length - 1;
    while (index >= 0) {
      this.#device3[index].delete();
      --index;
    }

    this.#jointParameters3?.delete();

    super.delete();
  }
}
