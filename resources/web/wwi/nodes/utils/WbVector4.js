import WbMatrix3 from './WbMatrix3.js';
import WbQuaternion from './WbQuaternion.js';
import {DOUBLE_EQUALITY_TOLERANCE} from './constants.js';

export default class WbVector4 {
  constructor(x = 0.0, y = 0.0, z = 0.0, w = 0.0) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }

  div(number) {
    return new WbVector4(this.x / number, this.y / number, this.z / number, this.w / number);
  }

  sub(vector) {
    return new WbVector4(this.x - vector.x, this.y - vector.y, this.z - vector.z, this.w - vector.w);
  }

  toMatrix3() {
    const c = Math.cos(this.w);
    const s = Math.sin(this.w);
    const t1 = 1 - c;
    const t2 = this.x * this.z * t1;
    const t3 = this.x * this.y * t1;
    const t4 = this.y * this.z * t1;

    let m = {};
    m[0] = this.x * this.x * t1 + c;
    m[1] = t3 - this.z * s;
    m[2] = t2 + this.y * s;
    m[3] = t3 + this.z * s;
    m[4] = this.y * this.y * t1 + c;
    m[5] = t4 - this.x * s;
    m[6] = t2 - this.y * s;
    m[7] = t4 + this.x * s;
    m[8] = this.z * this.z * t1 + c;

    return new WbMatrix3(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
  };

  clone() {
    return new WbVector4(this.x, this.y, this.z, this.w);
  }

  toString() {
    return this.x + ' ' + this.y + ' ' + this.z + ' ' + this.w;
  }

  toQuaternion() {
    const halfAngle = 0.5 * this.w;
    const sinusHalfAngle = Math.sin(halfAngle);
    const cosinusHalfAngle = Math.cos(halfAngle);
    return new WbQuaternion(this.x * sinusHalfAngle, this.y * sinusHalfAngle, this.z * sinusHalfAngle, cosinusHalfAngle);
  }

  fromQuaternion(q) {
    if (q.w >= 1.0)
      this.w = 0.0;
    else if (q.w <= -1.0)
      this.w = 2.0 * Math.PI;
    else
      this.w = 2.0 * Math.acos(q.w);
    if (this.w < DOUBLE_EQUALITY_TOLERANCE) {
      // if the angle is close to zero, then the direction of the axis is not important
      this.x = 0.0;
      this.y = 1.0;
      this.z = 0.0;
      this.w = 0.0;
      return;
    }

    // normalise axes
    const inv = 1.0 / Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    this.x = q.x * inv;
    this.y = q.y * inv;
    this.z = q.z * inv;
  }

  normalizeRotation() {
    this.normalizeAxis();
    this.normalizeAngle();
  }

  normalizeAxis() {
    if (!this.isValid()) {
      this.x = 0.0;
      this.y = 0.0;
      this.z = 1.0;
      return;
    }

    const invl = 1 / Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    if (Math.abs(invl - 1) > DOUBLE_EQUALITY_TOLERANCE) {
      this.x *= invl;
      this.y *= invl;
      this.z *= invl;
    }
  }

  normalizeAngle() {
    while (this.w < -Math.PI)
      this.w += 2 * Math.PI;
    while (this.w > Math.PI)
      this.w -= 2 * Math.PI;
  }

  isValid() {
    return !(this.x === 0 && this.y === 0 && this.z === 0) && !(typeof this.w === 'undefined');
  }
}
