import WbMotor from './WbMotor.js';
import {WbNodeType} from './wb_node_type.js';

// This class is used to retrieve the type of device present in the joint
export default class WbLinearMotor extends WbMotor {
  get nodeType() {
    return WbNodeType.WB_NODE_LINEAR_MOTOR;
  }
}
