export default class WbSFString {
  constructor(value = '') {
    if (typeof value !== 'string')
      throw new Error('Expected string in WbSFString constructor.');

    if (value.startsWith('"') && value.endsWith('"'))
      this.value = value.substring(1, value.length - 1);
    else
      this.value = value;
  }

  clone() {
    return new WbSFString(this.value);
  };

  asX3d() {
    return this.value;
  };

  jsify() {
    return '\'' + this.value + '\'';
  };
};
