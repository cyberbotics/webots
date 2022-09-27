export default class WbSFColor {
  constructor(r = 0.0, g = 0.0, b = 0.0) {
    if (typeof r !== 'number' || typeof g !== 'number' || typeof b !== 'number')
      throw new Error('Expected Numbers in WbColor constructor');
    if (r > 1 || r < 0 || g > 1 || g < 0 || b > 1 || b < 0)
      throw new Error('RGB values in WbSFColor constructor must be in [0, 1] range.');

    this.r = r;
    this.g = g;
    this.b = b;
  };

  clone() {
    return new WbSFColor(this.r, this.g, this.b);
  }

  asX3d() {
    return this.r + ' ' + this.g + ' ' + this.b;
  };

  jsify() {
    return '{r: ' + this.r + ', g: ' + this.g + ', b: ' + this.b + '}';
  };
};
