import WbMatrix3 from './WbMatrix3.js';
import WbVector3 from './WbVector3.js';
import WbVector4 from './WbVector4.js';

export default class WbMatrix4 {
  constructor() {
    this.m = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
  }

  extracted3x3Matrix() {
    return new WbMatrix3(this.m[0], this.m[1], this.m[2], this.m[4], this.m[5], this.m[6], this.m[8], this.m[9], this.m[10]);
  }

  fromVrml(translation, rotation, scale) {
    const tx = translation.x;
    const ty = translation.y;
    const tz = translation.z;
    const rx = rotation.x;
    const ry = rotation.y;
    const rz = rotation.z;
    const angle = rotation.w;
    const sx = scale.x;
    const sy = scale.y;
    const sz = scale.z;

    const c = Math.cos(angle);
    const s = Math.sin(angle);

    const t1 = 1 - c;
    const t2 = rx * rz * t1;
    const t3 = rx * ry * t1;
    const t4 = ry * rz * t1;

    // translation
    this.m[3] = tx;
    this.m[7] = ty;
    this.m[11] = tz;
    this.m[12] = this.m[13] = this.m[14] = 0;
    this.m[15] = 1;

    // rotate and scale (assuming the rotation vector is normalized)
    this.m[0] = (rx * rx * t1 + c) * sx;
    this.m[1] = (t3 - rz * s) * sy;
    this.m[2] = (t2 + ry * s) * sz;
    this.m[4] = (t3 + rz * s) * sx;
    this.m[5] = (ry * ry * t1 + c) * sy;
    this.m[6] = (t4 - rx * s) * sz;
    this.m[8] = (t2 - ry * s) * sx;
    this.m[9] = (t4 + rx * s) * sy;
    this.m[10] = (rz * rz * t1 + c) * sz;
  }

  inverse() {
    const inv = new Array(16);

    inv[0] = this.m[5] * this.m[10] * this.m[15] - this.m[5] * this.m[11] * this.m[14] - this.m[9] * this.m[6] * this.m[15] +
      this.m[9] * this.m[7] * this.m[14] + this.m[13] * this.m[6] * this.m[11] - this.m[13] * this.m[7] * this.m[10];

    inv[4] = -this.m[4] * this.m[10] * this.m[15] + this.m[4] * this.m[11] * this.m[14] + this.m[8] * this.m[6] * this.m[15] -
      this.m[8] * this.m[7] * this.m[14] - this.m[12] * this.m[6] * this.m[11] + this.m[12] * this.m[7] * this.m[10];

    inv[8] = this.m[4] * this.m[9] * this.m[15] - this.m[4] * this.m[11] * this.m[13] - this.m[8] * this.m[5] * this.m[15] +
      this.m[8] * this.m[7] * this.m[13] + this.m[12] * this.m[5] * this.m[11] - this.m[12] * this.m[7] * this.m[9];

    inv[12] = -this.m[4] * this.m[9] * this.m[14] + this.m[4] * this.m[10] * this.m[13] + this.m[8] * this.m[5] * this.m[14] -
      this.m[8] * this.m[6] * this.m[13] - this.m[12] * this.m[5] * this.m[10] + this.m[12] * this.m[6] * this.m[9];

    inv[1] = -this.m[1] * this.m[10] * this.m[15] + this.m[1] * this.m[11] * this.m[14] + this.m[9] * this.m[2] * this.m[15] -
      this.m[9] * this.m[3] * this.m[14] - this.m[13] * this.m[2] * this.m[11] + this.m[13] * this.m[3] * this.m[10];

    inv[5] = this.m[0] * this.m[10] * this.m[15] - this.m[0] * this.m[11] * this.m[14] - this.m[8] * this.m[2] * this.m[15] +
      this.m[8] * this.m[3] * this.m[14] + this.m[12] * this.m[2] * this.m[11] - this.m[12] * this.m[3] * this.m[10];

    inv[9] = -this.m[0] * this.m[9] * this.m[15] + this.m[0] * this.m[11] * this.m[13] + this.m[8] * this.m[1] * this.m[15] -
      this.m[8] * this.m[3] * this.m[13] - this.m[12] * this.m[1] * this.m[11] + this.m[12] * this.m[3] * this.m[9];

    inv[13] = this.m[0] * this.m[9] * this.m[14] - this.m[0] * this.m[10] * this.m[13] - this.m[8] * this.m[1] * this.m[14] +
      this.m[8] * this.m[2] * this.m[13] + this.m[12] * this.m[1] * this.m[10] - this.m[12] * this.m[2] * this.m[9];

    inv[2] = this.m[1] * this.m[6] * this.m[15] - this.m[1] * this.m[7] * this.m[14] - this.m[5] * this.m[2] * this.m[15] +
      this.m[5] * this.m[3] * this.m[14] + this.m[13] * this.m[2] * this.m[7] - this.m[13] * this.m[3] * this.m[6];

    inv[6] = -this.m[0] * this.m[6] * this.m[15] + this.m[0] * this.m[7] * this.m[14] + this.m[4] * this.m[2] * this.m[15] -
      this.m[4] * this.m[3] * this.m[14] - this.m[12] * this.m[2] * this.m[7] + this.m[12] * this.m[3] * this.m[6];

    inv[10] = this.m[0] * this.m[5] * this.m[15] - this.m[0] * this.m[7] * this.m[13] - this.m[4] * this.m[1] * this.m[15] +
      this.m[4] * this.m[3] * this.m[13] + this.m[12] * this.m[1] * this.m[7] - this.m[12] * this.m[3] * this.m[5];

    inv[14] = -this.m[0] * this.m[5] * this.m[14] + this.m[0] * this.m[6] * this.m[13] + this.m[4] * this.m[1] * this.m[14] -
      this.m[4] * this.m[2] * this.m[13] - this.m[12] * this.m[1] * this.m[6] + this.m[12] * this.m[2] * this.m[5];

    inv[3] = -this.m[1] * this.m[6] * this.m[11] + this.m[1] * this.m[7] * this.m[10] + this.m[5] * this.m[2] * this.m[11] -
      this.m[5] * this.m[3] * this.m[10] - this.m[9] * this.m[2] * this.m[7] + this.m[9] * this.m[3] * this.m[6];

    inv[7] = this.m[0] * this.m[6] * this.m[11] - this.m[0] * this.m[7] * this.m[10] - this.m[4] * this.m[2] * this.m[11] +
      this.m[4] * this.m[3] * this.m[10] + this.m[8] * this.m[2] * this.m[7] - this.m[8] * this.m[3] * this.m[6];

    inv[11] = -this.m[0] * this.m[5] * this.m[11] + this.m[0] * this.m[7] * this.m[9] + this.m[4] * this.m[1] * this.m[11] -
      this.m[4] * this.m[3] * this.m[9] - this.m[8] * this.m[1] * this.m[7] + this.m[8] * this.m[3] * this.m[5];

    inv[15] = this.m[0] * this.m[5] * this.m[10] - this.m[0] * this.m[6] * this.m[9] - this.m[4] * this.m[1] * this.m[10] +
      this.m[4] * this.m[2] * this.m[9] + this.m[8] * this.m[1] * this.m[6] - this.m[8] * this.m[2] * this.m[5];

    let det = this.m[0] * inv[0] + this.m[1] * inv[4] + this.m[2] * inv[8] + this.m[3] * inv[12];

    if (det === 0)
      return false;

    det = 1.0 / det;

    for (let i = 0; i < 16; ++i)
      this.m[i] = inv[i] * det;

    return true;
  }

  mul(matrix) {
    const mat = new WbMatrix4();
    mat.set(this.m[0] * matrix.m[0] + this.m[1] * matrix.m[4] + this.m[2] * matrix.m[8] + this.m[3] * matrix.m[12],
      this.m[0] * matrix.m[1] + this.m[1] * matrix.m[5] + this.m[2] * matrix.m[9] + this.m[3] * matrix.m[13],
      this.m[0] * matrix.m[2] + this.m[1] * matrix.m[6] + this.m[2] * matrix.m[10] + this.m[3] * matrix.m[14],
      this.m[0] * matrix.m[3] + this.m[1] * matrix.m[7] + this.m[2] * matrix.m[11] + this.m[3] * matrix.m[15],

      this.m[4] * matrix.m[0] + this.m[5] * matrix.m[4] + this.m[6] * matrix.m[8] + this.m[7] * matrix.m[12],
      this.m[4] * matrix.m[1] + this.m[5] * matrix.m[5] + this.m[6] * matrix.m[9] + this.m[7] * matrix.m[13],
      this.m[4] * matrix.m[2] + this.m[5] * matrix.m[6] + this.m[6] * matrix.m[10] + this.m[7] * matrix.m[14],
      this.m[4] * matrix.m[3] + this.m[5] * matrix.m[7] + this.m[6] * matrix.m[11] + this.m[7] * matrix.m[15],

      this.m[8] * matrix.m[0] + this.m[9] * matrix.m[4] + this.m[10] * matrix.m[8] + this.m[11] * matrix.m[12],
      this.m[8] * matrix.m[1] + this.m[9] * matrix.m[5] + this.m[10] * matrix.m[9] + this.m[11] * matrix.m[13],
      this.m[8] * matrix.m[2] + this.m[9] * matrix.m[6] + this.m[10] * matrix.m[10] + this.m[11] * matrix.m[14],
      this.m[8] * matrix.m[3] + this.m[9] * matrix.m[7] + this.m[10] * matrix.m[11] + this.m[11] * matrix.m[15],

      this.m[12] * matrix.m[0] + this.m[13] * matrix.m[4] + this.m[14] * matrix.m[8] + this.m[15] * matrix.m[12],
      this.m[12] * matrix.m[1] + this.m[13] * matrix.m[5] + this.m[14] * matrix.m[9] + this.m[15] * matrix.m[13],
      this.m[12] * matrix.m[2] + this.m[13] * matrix.m[6] + this.m[14] * matrix.m[10] + this.m[15] * matrix.m[14],
      this.m[12] * matrix.m[3] + this.m[13] * matrix.m[7] + this.m[14] * matrix.m[11] + this.m[15] * matrix.m[15]);
    return mat;
  }

  mulByVec3(vector) {
    return new WbVector3(this.m[0] * vector.x + this.m[1] * vector.y + this.m[2] * vector.z + this.m[3],
      this.m[4] * vector.x + this.m[5] * vector.y + this.m[6] * vector.z + this.m[7],
      this.m[8] * vector.x + this.m[9] * vector.y + this.m[10] * vector.z + this.m[11]);
  }

  mulByVec4(vector) {
    return new WbVector4(this.m[0] * vector.x + this.m[1] * vector.y + this.m[2] * vector.z + this.m[3] * vector.w,
      this.m[4] * vector.x + this.m[5] * vector.y + this.m[6] * vector.z + this.m[7] * vector.w,
      this.m[8] * vector.x + this.m[9] * vector.y + this.m[10] * vector.z + this.m[11] * vector.w,
      this.m[12] * vector.x + this.m[13] * vector.y + this.m[14] * vector.z + this.m[15] * vector.w);
  }

  set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33) {
    this.m = [m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33];
  }

  setFromArray(array) {
    this.m = [array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8], array[9], array[10],
      array[11], array[12], array[13], array[14], array[15]];
  }
}
