export default class WbVector4 {
  constructor(x, y, z, w) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }

  div(number) {
    return new WbVector4(this.x / number, this.y / number, this.z / number, this.w / number);
  }
}
