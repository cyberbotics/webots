class WbMatrix4 {
  constructor(){
    this.m = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
  }

  extracted3x3Matrix()  {
    return WbMatrix3(this.m[0], this.m[1], this.m[2], this.m[4], this.m[5], this.m[6], this.m[8], this.m[9], this.m[10]);
  }

  mulByVec4(v) {
    return WbVector4(this.m[0] * v.x + this.m[1] * v.y + this.m[2] * v.z + this.m[3] * v.w,
      this.m[4] * v.x + this.m[5] * v.y + this.m[6] * v.z + this.m[7] * v.w,
      this.m[8] * v.x + this.m[9] * v.y + this.m[10] * v.z + this.m[11] * v.w,
      this.m[12] * v.x + this.m[13] * v.y + this.m[14] * v.z + this.m[15] * v.w);
  }
}
export {WbMatrix4}
