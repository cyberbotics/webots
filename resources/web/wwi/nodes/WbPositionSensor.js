import WbLogicalDevice from './WbLogicalDevice.js';
import {WbNodeType} from './wb_node_type.js';

// This class is used to retrieve the type of device present in the joint
export default class WbPositionSensor extends WbLogicalDevice {
  get nodeType() {
    return WbNodeType.WB_NODE_POSITION_SENSOR;
  }
}
