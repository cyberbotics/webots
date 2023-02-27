import WbHingeJoint from './WbHingeJoint.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbQuaternion from './utils/WbQuaternion.js';
import {isZeroAngle} from './utils/math_utilities.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbHinge2Joint extends WbHingeJoint {
  #device2;
  #jointParameters2;
  #position2;
  constructor(id) {
    super(id);
    this.#device2 = [];
  }

  get nodeType() {
    return WbNodeType.WB_NODE_HINGE_2_JOINT;
  }

  get device2() {
    return this.#device2;
  }

  set device2(device) {
    this.#device2 = device;
  }

  get jointParameters2() {
    return this.#jointParameters2;
  }

  set jointParameters2(jointParameters) {
    this.#jointParameters2 = jointParameters;

    if (typeof this.#jointParameters2 !== 'undefined')
      this.#jointParameters2.onChange = () => this._updatePosition();
  }

  get position2() {
    return this.#position2;
  }

  set position2(newPosition) {
    if (this.#position2 === newPosition)
      return;

    this.#position2 = newPosition;
    if (typeof this.#jointParameters2 === 'undefined')
      this._updatePosition();
  }

  preFinalize() {
    this.#position2 = typeof this.jointParameters2 === 'undefined' ? 0 : this.jointParameters2.position;

    super.preFinalize();
    this.#device2.forEach(child => child.preFinalize());
    this.#jointParameters2?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#device2.forEach(child => child.postFinalize());
    this.#jointParameters2?.postFinalize();
  }

  delete() {
    let index = this.#device2.length - 1;
    while (index >= 0) {
      this.#device2[index].delete();
      --index;
    }

    this.#jointParameters2?.delete();

    super.delete();
  }

  _updatePosition() {
    if (typeof this.endPoint !== 'undefined') {
      const position = typeof this.jointParameters !== 'undefined' ? this.jointParameters.position : this.position;
      const position2 = typeof this.jointParameters2 !== 'undefined' ? this.jointParameters2.position : this.#position2;
      this.#updatePositions(position, position2);
    }
  }

  _updateEndPointZeroTranslationAndRotation() {
    if (typeof this.solidEndPoint() === 'undefined')
      return;

    const solid = this.solidEndPoint();
    const ir = solid.rotation;
    const it = solid.translation;

    let qp = new WbQuaternion();
    if (isZeroAngle(this.position) && isZeroAngle(this.#position2))
      // Keeps track of the original axis if the angle is zero as it defines the second DoF axis
      this._endPointZeroRotation = ir;
    else {
      const axis = this.axis();
      const q = new WbQuaternion();
      q.fromAxisAngle(axis.x, axis.y, axis.z, -this.position);

      const axis2 = this.axis2();
      const q2 = new WbQuaternion();
      q2.fromAxisAngle(axis2.x, axis2.y, axis2.z, -this.#position2);

      qp = q2.mul(q);
      const iq = ir.toQuaternion();
      const qr = qp.mul(iq);
      qr.normalize();

      this._endPointZeroRotation = new WbVector4();
      this._endPointZeroRotation.fromQuaternion(qr);
    }
    const a = this.anchor();
    const t = it.sub(a);
    this._endPointZeroTranslation = qp.mulByVec3(t).add(a);
  }

  #updatePositions(position, position2) {
    this.position = position;
    this.#position2 = position2;

    let rotation = new WbVector4();
    const translation = this.#computeEndPointSolidPositionFromParameters(rotation);
    const solid = this.solidEndPoint();
    if (!translation.almostEquals(solid.translation) || !rotation.almostEquals(solid.rotation)) {
      solid.translation = translation;
      solid.rotation = rotation;
    }
  }

  #computeEndPointSolidPositionFromParameters(rotation) {
    // case where there was not endPoint when the joint was created but one was added later.
    if (typeof this._endPointZeroRotation === 'undefined')
      this._updateEndPointZeroTranslationAndRotation();

    const q = new WbQuaternion();
    const axis = this.axis();
    q.fromAxisAngle(axis.x, axis.y, axis.z, this.position);

    const q2 = new WbQuaternion();
    const axis2 = this.axis2();
    q2.fromAxisAngle(axis2.x, axis2.y, axis2.z, this.#position2);

    const qi = this._endPointZeroRotation.toQuaternion();
    let qp = q.mul(q2);
    const a = this.anchor();
    const t = this._endPointZeroTranslation.sub(a);
    const translation = qp.mulByVec3(t).add(a);
    qp = qp.mul(qi);
    qp.normalize();
    rotation.fromQuaternion(qp);
    return translation;
  }

  axis2() {
    return typeof this.jointParameters2 !== 'undefined' ? this.jointParameters2.axis : WbHinge2Joint.DEFAULT_AXIS_2;
  }
}

WbHinge2Joint.DEFAULT_AXIS_2 = new WbVector3(0, 0, 1);
