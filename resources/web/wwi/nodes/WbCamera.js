import WbAbstractCamera from './WbAbstractCamera.js';

// This class is used to retrieve the type of device
export default class WbCamera extends WbAbstractCamera {
  #near;
  #far;
  constructor(id, translation, scale, rotation, name, height, width, fieldOfView, near, far) {
    super(id, translation, scale, rotation, name, height, width, fieldOfView);
    this.#near = near;
    this.#far = far;
    this._charType = 'c';
  }

  get near() {
    return this.#near;
  }

  set near(newNear) {
    this.#near = newNear;
    this._update();
  }

  get far() {
    return this.#far;
  }

  set far(newFar) {
    this.#far = newFar;
    this._update();
  }

  minRange() {
    return this.#near;
  }

  maxRange() {
    return this.#far;
  }
}
