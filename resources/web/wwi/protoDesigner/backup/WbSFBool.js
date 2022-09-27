export default class WbSFBool {
  constructor(value = false) {
    if (typeof value !== 'boolean')
      throw new Error('Expected boolean in WbSFBool constructor.');

    this.value = value;
  }

  clone() {
    return new WbSFBool(this.value);
  };

  asX3d() {
    return this.value.toString();
  };

  jsify() {
    return this.value;
  };
};
