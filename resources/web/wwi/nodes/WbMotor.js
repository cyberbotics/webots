import WbLogicalDevice from './WbLogicalDevice.js';

export default class WbMotor extends WbLogicalDevice {
  #minPosition;
  #maxPosition;
  constructor(id, deviceName, minPosition, maxPosition) {
    super(id, deviceName);
    this.#minPosition = minPosition;
    this.#maxPosition = maxPosition;
  }

  get minPosition() {
    return this.#minPosition;
  }

  get maxPosition() {
    return this.#maxPosition;
  }
}
