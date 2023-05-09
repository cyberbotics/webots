import WbDevice from './WbDevice.js';
import {WbNodeType} from './wb_node_type.js';

// This class is used to retrieve the type of device
export default class WbVacuumCup extends WbDevice {
  constructor(id, translation, scale, rotation, name) {
    super(id, translation, scale, rotation, name);

    this.showOptionalRendering = false;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_VACUUM_CUP;
  }

  applyOptionalRendering(enable) {
    super.applyOptionalRendering(enable);
    this.showOptionalRendering = enable;
    this.updateBoundingObjectVisibility();
  }
}

