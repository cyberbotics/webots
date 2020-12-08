class WbVector3 {
  constructor(x = 0.0, y = 0.0, z = 0.0){
    this.x = x;
    this.y = y;
    this.z = z;
  }

  sub(vec) {
    return new WbVector3(this.x - vec.x, this.y - vec.y, this.z -  vec.z);
  }

  div(number) {
    this.x /= number;
    this.y /= number;
    this.z /= number;

    return this;
  }
  // cross product
  cross(v) {
    return new WbVector3(this.y * v.z - this.z * v.y, this.z * v.x - this.x * v.z, this.x * v.y - this.y * v.x);
  }

  almostEquals(v, tolerance) {
    return Math.abs(this.x - v.x) < tolerance && Math.abs(this.y - v.y) < tolerance && Math.abs(this.z - v.z) < tolerance;
  }

  equal(v) {
    return this.x === v.x && this.y === v.y && this.z === v.z;
  }

  normalize() {
    return this.div(this.length());
  }

  length() {
     return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }

}
export {WbVector3}
