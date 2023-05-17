import WbAbstractCamera from './WbAbstractCamera.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbCamera extends WbAbstractCamera {
  #near;
  #far;
  constructor(id, translation, rotation, name, height, width, fieldOfView, near, far) {
    super(id, translation, rotation, name, height, width, fieldOfView);
    this.#near = near;
    this.#far = far;
    this._charType = 'c';
  }

  get nodeType() {
    return WbNodeType.WB_NODE_CAMERA;
  }

  get near() {
    return this.#near;
  }

  set near(newNear) {
    if (newNear <= 0)
      newNear = 0.01;
    else if (newNear > this.#far && this.#far !== 0)
      newNear = this.#far;

    this.#near = newNear;
    this._update();
  }

  get far() {
    return this.#far;
  }

  set far(newFar) {
    if (newFar < 0)
      newFar = 0;
    else if (this.#near > newFar)
      newFar = this.#near + 1;

    this.#far = newFar;
    this._update();
  }

  _minRange() {
    return this.#near;
  }

  _maxRange() {
    return this.#far;
  }
}
