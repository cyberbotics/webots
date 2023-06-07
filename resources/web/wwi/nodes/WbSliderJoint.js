import WbJoint from './WbJoint.js';
import WbVector3 from './utils/WbVector3.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbSliderJoint extends WbJoint {
  #device;
  #endPointZeroTranslation;
  constructor(id) {
    super(id);
    this.#device = [];
  }

  get nodeType() {
    return WbNodeType.WB_NODE_SLIDER_JOINT;
  }

  get device() {
    return this.#device;
  }

  set device(device) {
    this.#device = device;
  }

  preFinalize() {
    super.preFinalize();
    this.device.forEach(child => child.preFinalize());
  }

  postFinalize() {
    super.postFinalize();

    this.device.forEach(child => child.postFinalize());
  }

  delete() {
    let index = this.device.length - 1;
    while (index >= 0) {
      this.device[index].delete();
      --index;
    }

    super.delete();
  }

  _updatePosition() {
    if (typeof this.endPoint !== 'undefined')
      this.#updatePosition(typeof this.jointParameters !== 'undefined' ? this.jointParameters.position : this.position);
  }

  _updateEndPointZeroTranslationAndRotation() {
    if (typeof this.solidEndPoint() === 'undefined')
      return;

    const solid = this.solidEndPoint();
    this.#endPointZeroTranslation = solid.translation.sub(this.#axis().mul(this.position));
  }

  #axis() {
    return typeof this.jointParameters !== 'undefined' ? this.jointParameters.axis.normalized() : WbSliderJoint.DEFAULT_AXIS;
  }

  #updatePosition(position) {
    // called after an artificial move
    this.position = position;
    const translation = this.#computeEndPointSolidPositionFromParameters();
    const solid = this.solidEndPoint();
    if (!translation.almostEquals(solid.translation))
      solid.translation = translation;
  }

  #computeEndPointSolidPositionFromParameters() {
    return this.#endPointZeroTranslation.add(this.#axis().mul(this.position));
  }
}

WbSliderJoint.DEFAULT_AXIS = new WbVector3(0, 0, 1);
