export default class WbSFDouble {
  constructor(value = 0.0) {
    if (typeof value !== 'number')
      throw new Error('Expected number in WbSFDouble constructor.');

    this.value = value;
  };

  asX3d() {
    return this.value.toString();
  };
};
