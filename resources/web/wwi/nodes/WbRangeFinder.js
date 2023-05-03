import WbAbstractCamera from './WbAbstractCamera.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbRangeFinder extends WbAbstractCamera {
  #maxRange;
  #minRange;
  constructor(id, translation, rotation, name, height, width, fieldOfView, maxRange, minRange) {
    super(id, translation, rotation, name, height, width, fieldOfView);
    this.#maxRange = maxRange;
    this.#minRange = minRange;
    this._charType = 'r';
  }

  get nodeType() {
    return WbNodeType.WB_NODE_RANGE_FINDER;
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
