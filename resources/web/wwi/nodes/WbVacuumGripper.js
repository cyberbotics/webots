import WbDevice from './WbDevice.js';
import {WbNodeType} from './wb_node_type.js';

// This class is used to retrieve the type of device
export default class WbVacuumGripper extends WbDevice {
  constructor(id, translation, rotation, name) {
    super(id, translation, rotation, name);

    this.showOptionalRendering = false;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_VACUUM_GRIPPER;
  }

  applyOptionalRendering(enable) {
    super.applyOptionalRendering(enable);
    this.showOptionalRendering = enable;
    this.updateBoundingObjectVisibility();
  }
}
