import WbMatrix3 from './WbMatrix3.js';

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
}
