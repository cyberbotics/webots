import WbLogicalDevice from './WbLogicalDevice.js';

export default class WbMotor extends WbLogicalDevice {
  #minPosition;
  #maxPosition;
  #multiplier;
  constructor(id, deviceName, minPosition, maxPosition, multiplier) {
    super(id, deviceName);
    this.#minPosition = minPosition;
    this.#maxPosition = maxPosition;
    this.#multiplier = multiplier;
  }

  get minPosition() {
    return this.#minPosition;
  }

  get maxPosition() {
    return this.#maxPosition;
  }

  get multiplier() {
    return this.#multiplier;
  }
}
