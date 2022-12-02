import WbAbstractCamera from './WbAbstractCamera.js';

// This class is used to retrieve the type of device
export default class WbRangeFinder extends WbAbstractCamera {
  #maxRange;
  #minRange;
  constructor(id, translation, scale, rotation, name, height, width, fieldOfView, maxRange, minRange) {
    super(id, translation, scale, rotation, name, height, width, fieldOfView);
    this.#maxRange = maxRange;
    this.#minRange = minRange;
    this._charType = 'r';
  }

  get maxRange() {
    return this.#maxRange;
  }

  set maxRange(newMaxRange) {
    this.#maxRange = newMaxRange;
    this._update();
  }

  get minRange() {
    return this.#minRange;
  }

  set minRange(newMinRange) {
    this.#minRange = newMinRange;
    this._update();
  }

  _minRange() {
    return this.#minRange;
  }

  _maxRange() {
    return this.#maxRange;
  }
}
