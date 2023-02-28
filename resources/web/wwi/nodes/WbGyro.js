import WbDevice from './WbDevice.js';
import {WbNodeType} from './wb_node_type.js';

// This class is used to retrieve the type of device
export default class WbGyro extends WbDevice {
  get nodeType() {
    return WbNodeType.WB_NODE_GYRO;
  }
}
