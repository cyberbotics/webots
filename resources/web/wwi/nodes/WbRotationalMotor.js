import WbMotor from './WbMotor.js';
import {WbNodeType} from './wb_node_type.js';

// This class is used to retrieve the type of device present in the joint
export default class WbRotationalMotor extends WbMotor {
  get nodeType() {
    return WbNodeType.WB_NODE_ROTATIONAL_MOTOR;
  }
}
