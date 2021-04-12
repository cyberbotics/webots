class WbVector2 {
  constructor(x = 0.0, y = 0.0) {
    this.x = x;
    this.y = y;
  }

  add(vector) {
    return new WbVector2(this.x + vector.x, this.y + vector.y);
  }

  sub(vector) {
    return new WbVector2(this.x - vector.x, this.y - vector.y);
  }

  mul(number) {
    return new WbVector2(this.x * number, this.y * number);
  }

  div(number) {
    return new WbVector2(this.x / number, this.y / number);
  }

  equal(vector) {
    return this.x === vector.x && this.y === vector.y;
  }

  normalize() {
    const result = this.div(this.length());
    this.x = result.x;
    this.y = result.y;
  }

  normalized() {
    return this.div(this.length());
  }

  length() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  }

  length2() {
    return this.x * this.x + this.y * this.y;
  }

  // dot product
  dot(v) {
    return this.x * v.x + this.y * v.y;
  }

  // null test
  isNull() {
    return this.x === 0.0 && this.y === 0.0;
  }

  // angle between two vectors (in radians)
  angle(vector) {
    const s = this.dot(vector) / Math.sqrt(this.length2() * vector.length2());
    return (s >= 1.0) ? 0 : (s <= -1.0) ? Math.PI : Math.acos(s);
  }

  get(index) {
    if (index === 0)
      return this.x;
    else if (index === 1)
      return this.y;
  }
}

export {WbVector2};
