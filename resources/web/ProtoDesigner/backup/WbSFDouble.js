export default class WbSFDouble {
  constructor(value = 0.0) {
    if (typeof value !== 'number')
      throw new Error('Expected number in WbSFDouble constructor.');

    this.value = value;
  };

  clone() {
    return new WbSFDouble(this.value);
  };

  asX3d() {
    return this.value.toString();
  };

  jsify() {
    return this.value;
  };
};
