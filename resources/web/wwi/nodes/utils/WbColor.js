export default class WbColor {
  constructor(r = 0.0, g = 0.0, b = 0.0) {
    if (typeof r !== 'number' || typeof g !== 'number' || typeof b !== 'number')
      throw new Error('Expected Numbers in WbColor constructor');
    if (r > 1 || r < 0 || g > 1 || g < 0 || b > 1 || b < 0)
      throw new Error('RGB values in WbColor constructor must be in [0, 1] range.');

    this.r = r;
    this.g = g;
    this.b = b;
  }
};
