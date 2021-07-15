export default class WbVector4 {
  constructor(x = 0.0, y = 0.0, z = 0.0, w = 0.0) {
    if (typeof x !== 'number' || typeof y !== 'number' || typeof z !== 'number' || typeof w !== 'number')
      throw new Error('Expected Numbers in WbVector4 constructor');

    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }

  div(number) {
    return new WbVector4(this.x / number, this.y / number, this.z / number, this.w / number);
  }

  clone() {
    return new WbVector4(this.x, this.y, this.z, this.w);
  };

  asX3d() {
    return this.x + ' ' + this.y + ' ' + this.z + ' ' + this.w;
  };

  jsify() {
    return '{x: ' + this.x + ', y: ' + this.y + ', z: ' + this.z + ', w: ' + this.w + '}';
  };
}
