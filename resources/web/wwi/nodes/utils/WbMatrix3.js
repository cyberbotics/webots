import WbVector3 from './WbVector3.js';
import WbVector4 from './WbVector4.js';
import {clampedAcos} from './math_utilities.js';
import {DOUBLE_EQUALITY_TOLERANCE} from './constants.js';

export default class WbMatrix3 {
  constructor(m0 = 1.0, m1 = 0, m2 = 0, m3 = 0, m4 = 1.0, m5 = 0, m6 = 0, m7 = 0, m8 = 1.0) {
    this.m = [9];
    this.m[0] = m0;
    this.m[1] = m1;
    this.m[2] = m2;
    this.m[3] = m3;
    this.m[4] = m4;
    this.m[5] = m5;
    this.m[6] = m6;
    this.m[7] = m7;
    this.m[8] = m8;
  }

  mulByVec3(vector) {
    return new WbVector3(this.m[0] * vector.x + this.m[1] * vector.y + this.m[2] * vector.z,
      this.m[3] * vector.x + this.m[4] * vector.y + this.m[5] * vector.z,
      this.m[6] * vector.x + this.m[7] * vector.y + this.m[8] * vector.z);
  }

  static fromEulerAngles(rx, ry, rz) {
    // Reference: https://www.geometrictools.com/Documentation/EulerAngles.pdf

    const cx = Math.cos(rx);
    const sx = Math.sin(rx);
    const cy = Math.cos(ry);
    const sy = Math.sin(ry);
    const cz = Math.cos(rz);
    const sz = Math.sin(rz);
    let m = [9];
    m[0] = cy * cz;
    m[1] = -cy * sz;
    m[2] = sy;
    m[3] = cz * sx * sy + cx * sz;
    m[4] = cx * cz - sx * sy * sz;
    m[5] = -cy * sx;
    m[6] = -cx * cz * sy + sx * sz;
    m[7] = cz * sx + cx * sy * sz;
    m[8] = cx * cy;
    return new WbMatrix3(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
  }

  mulByMat3(mat3) {
    return new WbMatrix3(this.m[0] * mat3.m[0] + this.m[1] * mat3.m[3] + this.m[2] * mat3.m[6],
      this.m[0] * mat3.m[1] + this.m[1] * mat3.m[4] + this.m[2] * mat3.m[7],
      this.m[0] * mat3.m[2] + this.m[1] * mat3.m[5] + this.m[2] * mat3.m[8],

      this.m[3] * mat3.m[0] + this.m[4] * mat3.m[3] + this.m[5] * mat3.m[6],
      this.m[3] * mat3.m[1] + this.m[4] * mat3.m[4] + this.m[5] * mat3.m[7],
      this.m[3] * mat3.m[2] + this.m[4] * mat3.m[5] + this.m[5] * mat3.m[8],

      this.m[6] * mat3.m[0] + this.m[7] * mat3.m[3] + this.m[8] * mat3.m[6],
      this.m[6] * mat3.m[1] + this.m[7] * mat3.m[4] + this.m[8] * mat3.m[7],
      this.m[6] * mat3.m[2] + this.m[7] * mat3.m[5] + this.m[8] * mat3.m[8]);
  }

  toRotation() {
    // Reference: https://www.geometrictools.com/Documentation/RotationRepresentations.pdf
    const theta = clampedAcos((this.m[0] + this.m[4] + this.m[8] - 1) / 2);
    let x = 0;
    let y = 0;
    let z = 0;
    if (theta < DOUBLE_EQUALITY_TOLERANCE) // If `theta == 0`
      return new WbVector4(0, 0, 1, 0);
    else if (Math.PI - theta < DOUBLE_EQUALITY_TOLERANCE) { // If `theta == pi`
      if (this.m[0] > this.m[4] && this.m[0] > this.m[8]) {
        x = Math.sqrt(this.m[0] - this.m[4] - this.m[8] + 1) / 2;
        y = this.m[1] / (2 * x);
        z = this.m[2] / (2 * x);
      } else if (this.m[4] > this.m[0] && this.m[4] > this.m[8]) {
        y = Math.sqrt(this.m[4] - this.m[0] - this.m[8] + 1) / 2;
        x = this.m[1] / (2 * y);
        z = this.m[5] / (2 * y);
      } else {
        z = Math.sqrt(this.m[8] - this.m[0] - this.m[4] + 1) / 2;
        x = this.m[2] / (2 * z);
        y = this.m[5] / (2 * z);
      }
    } else { // If `theta in (0, pi)`
      x = this.m[7] - this.m[5];
      y = this.m[2] - this.m[6];
      z = this.m[3] - this.m[1];
    }

    if ((x === 0.0 && y === 0.0 && z === 0.0) && !isNaN(theta)) {
      x = 0.0;
      y = 1.0;
      z = 0.0;
    }

    let invl = 1.0 / Math.sqrt(x * x + y * y + z * z);
    if (Math.abs(invl - 1.0) > DOUBLE_EQUALITY_TOLERANCE) {
      x *= invl;
      y *= invl;
      z *= invl;
    }
    return new WbVector4(x, y, z, theta);
  }
}
