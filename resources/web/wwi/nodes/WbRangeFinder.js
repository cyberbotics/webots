import WbAbstractCamera from './WbAbstractCamera.js';

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
    if (this.#minRange > newMaxRange || newMaxRange <= 0)
      newMaxRange = this.#minRange + 1;

    this.#maxRange = newMaxRange;
    this._update();
  }

  get minRange() {
    return this.#minRange;
  }

  set minRange(newMinRange) {
    if (newMinRange <= 0 || newMinRange > this.#maxRange)
      newMinRange = 0.01;

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
