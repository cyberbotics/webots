export default class WbBool {
  constructor(value = false) {
    if (typeof value !== 'boolean')
      throw new Error('Expected boolean in WbBool constructor.');

    this.value = value;
  }
};
