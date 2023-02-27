import WbDevice from './WbDevice.js';
import {WbNodeType} from './wb_node_type.js';

// This class is used to retrieve the type of device
export default class WbCharger extends WbDevice {
  get nodeType() {
    return WbNodeType.WB_NODE_CHARGER;
  }
}
