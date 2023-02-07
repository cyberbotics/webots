import WbVector3 from './WbVector3.js';

export default class WbQuaternion {
  constructor(x = 0, y = 0, z = 0, angle = 1) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = angle;
  }

  fromAxisAngle(rx, ry, rz, angle) {
    let l = rx * rx + ry * ry + rz * rz;
    if (l > 0) {
      angle *= 0.5;
      this.w = Math.cos(angle);
      l = Math.sin(angle) / Math.sqrt(l);
      this.x = rx * l;
      this.y = ry * l;
      this.z = rz * l;
    } else {
      this.w = 1;
      this.x = 0;
      this.y = 0;
      this.z = 0;
    }
  }

  mul(q) {
    return new WbQuaternion(this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y,
      this.w * q.y + this.y * q.w + this.z * q.x - this.x * q.z, this.w * q.z + this.z * q.w + this.x * q.y - this.y * q.x,
      this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z);
  }

  mulByVec3(v) {
    const twoX = 2 * this.x;
    const twoY = 2 * this.y;
    const twoZ = 2 * this.z;
    const sX = twoX * this.x;
    const sY = twoY * this.y;
    const sZ = twoZ * this.z;
    const sW = 2 * this.w * this.w;
    const pXY = twoX * this.y;
    const pXZ = twoX * this.z;
    const pXW = twoX * this.w;
    const pYZ = twoY * this.z;
    const pYW = twoY * this.w;
    const pZW = twoZ * this.w;
    const m00 = -1 + sX + sW;
    const m01 = pXY - pZW;
    const m02 = pXZ + pYW;
    const m10 = pXY + pZW;
    const m11 = -1 + sY + sW;
    const m12 = pYZ - pXW;
    const m20 = pXZ - pYW;
    const m21 = pYZ + pXW;
    const m22 = -1 + sZ + sW;
    return new WbVector3(m00 * v.x + m01 * v.y + m02 * v.z, m10 * v.x + m11 * v.y + m12 * v.z,
      m20 * v.x + m21 * v.y + m22 * v.z);
  }

  normalize() {
    let d = this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w;
    if (d === 0) {
      this.w = 1;
      return;
    }
    // see explanation at https://stackoverflow.com/questions/11667783/quaternion-and-normalization
    if (Math.abs(1 - d) < 2.107342e-08) // 2.107342e-08 magic number (> ULP/2 for IEEE doubles)
      d = 2 / (1 + d); // first order Padé approximant
    else
      d = 1 / Math.sqrt(d);
    this.w *= d;
    this.x *= d;
    this.y *= d;
    this.z *= d;
  }
}
