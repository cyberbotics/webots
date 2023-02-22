import WbHinge2Joint from './WbHinge2Joint.js';
import WbQuaternion from './utils/WbQuaternion.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import {isZeroAngle} from './utils/math_utilities.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbBallJoint extends WbHinge2Joint {
  #device3;
  #jointParameters3;
  #position3;
  constructor(id) {
    super(id);
    this.#device3 = [];
  }

  get nodeType() {
    return WbNodeType.WB_NODE_BALL_JOINT;
  }

  get device3() {
    return this.#device3;
  }

  set device3(device) {
    this.#device3 = device;
  }

  get jointParameters3() {
    return this.#jointParameters3;
  }

  set jointParameters3(jointParameters) {
    this.#jointParameters3 = jointParameters;
  }

  get position3() {
    return this.#position3;
  }

  set position3(newPosition) {
    if (this.#position3 === newPosition)
      return;

    this.#position3 = newPosition;
    if (typeof this.jointParameters3 === 'undefined')
      this._updatePosition();
  }

  preFinalize() {
    this.#position3 = typeof this.jointParameters3 === 'undefined' ? 0 : this.jointParameters3.position;

    super.preFinalize();
    this.#device3.forEach(child => child.preFinalize());
    this.#jointParameters3?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#device3.forEach(child => child.postFinalize());
    this.#jointParameters3?.postFinalize();
  }

  delete() {
    let index = this.#device3.length - 1;
    while (index >= 0) {
      this.#device3[index].delete();
      --index;
    }

    this.#jointParameters3?.delete();

    super.delete();
  }

  axis() {
    const p2 = this.jointParameters2;
    const p3 = this.jointParameters3;
    if (typeof p2 === 'undefined') {
      if (typeof p3 === 'undefined')
        return new WbVector3(1, 0, 0);
      else if (p3.axis().cross(new WbVector3(0, 0, 1)).isNull())
        return p3.axis().cross(new WbVector3(1, 0, 0));
      else
        return p3.axis().cross(new WbVector3(0, 0, 1));
    }
    return p2.axis();
  }

  axis2() {
    return this.axis3().cross(this.axis());
  }

  axis3() {
    const p2 = this.jointParameters2;
    const p3 = this.jointParameters3;
    if (typeof p3 === 'undefined') {
      if (typeof p2 === 'undefined')
        return new WbVector3(0, 0, 1);
      else if (p2.axis.cross(new WbVector3(1, 0, 0)).isNull())
        return p2.axis.cross(new WbVector3(0, 0, 1));
      else
        return p2.axis.cross(new WbVector3(1, 0, 0));
    }
    return p3.axis;
  }

  _updatePosition() {
    if (typeof this.endPoint !== 'undefined') {
      const position = typeof this.jointParameters !== 'undefined' ? this.jointParameters.position : this.position;
      const position2 = typeof this.jointParameters2 !== 'undefined' ? this.jointParameters2.position : this.position2;
      const position3 = typeof this.jointParameters3 !== 'undefined' ? this.jointParameters3.position : this.#position3;

      this.#updatePositions(position, position2, position3);
    }
  }

  _updateEndPointZeroTranslationAndRotation() {
    if (typeof this.solidEndPoint() === 'undefined')
      return;

    const solid = this.solidEndPoint();
    const ir = solid.rotation;
    const it = solid.translation;

    let qp = new WbQuaternion();
    if (isZeroAngle(this.position) && isZeroAngle(this.position2) && isZeroAngle(this.#position3))
      this._endPointZeroRotation = ir;
    else {
      const axis = this.axis();
      const q = new WbQuaternion();
      q.fromAxisAngle(axis.x, axis.y, axis.z, -this.position);

      const axis2 = this.axis2();
      const q2 = new WbQuaternion();
      q2.fromAxisAngle(axis2.x, axis2.y, axis2.z, -this.position2);

      const axis3 = this.axis3();
      const q3 = new WbQuaternion();
      q3.fromAxisAngle(axis3.x, axis3.y, axis3.z, -this.#position3);

      qp = q3.mul(q2).mul(q);
      const iq = ir.toQuaternion();
      const qr = qp * iq;
      qr.normalize();
      this._endPointZeroRotation = new WbVector4();
      this._endPointZeroRotation.fromQuaternion(qr);
    }
    const a = this.anchor();
    const t = it.sub(a);
    this._endPointZeroTranslation = qp.mulByVec3(t).add(a);
  }

  #updatePositions(position, position2, position3) {
    this.position = position;
    this.position2 = position2;
    this.#position3 = position3;
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
    q2.fromAxisAngle(axis2.x, axis2.y, axis2.z, this.position2);

    const q3 = new WbQuaternion();
    const axis3 = this.axis3();
    q3.fromAxisAngle(axis3.x, axis3.y, axis3.z, this.#position3);

    const qi = this._endPointZeroRotation.toQuaternion();
    let qp = q.mul(q2).mul(q3);
    const a = this.anchor();
    const t = this._endPointZeroTranslation.sub(a);
    const translation = qp.mulByVec3(t).add(a);
    qp = qp.mul(qi);
    qp.normalize();
    rotation.fromQuaternion(qp);
    return translation;
  }
}
