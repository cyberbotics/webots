import WbDevice from './WbDevice.js';

// This class is used to retrieve the type of device
export default class WbTouchSensor extends WbDevice {
  constructor(id, translation, scale, rotation, name) {
    super(id, translation, scale, rotation, name);

    this.showOptionalRendering = false;
  }

  applyOptionalRendering(enable) {
    super.applyOptionalRendering(enable);
    this.showOptionalRendering = enable;
    this.updateBoundingObjectVisibility();
  }
}
