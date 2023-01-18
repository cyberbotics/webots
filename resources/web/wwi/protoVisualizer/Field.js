'use strict';

export default class Field {
  #type;
  #name;
  #value;
  #defaultValue;
  #restrictions;
  constructor(name, type, value, defaultValue) {
    this.#name = name;
    this.#type = type;
    this.#value = value;
    this.#defaultValue = defaultValue;
    this.#restrictions = [];
  }

  get value() {
    return this.#value;
  }

  get defaultValue() {
    return this.#defaultValue;
  }

  get type() {
    return this.#type;
  }

  get name() {
    return this.#name;
  }

  get restrictions() {
    return this.#restrictions;
  }

  isDefault() {
    if (typeof this.defaultValue === 'undefined' || typeof this.value === 'undefined')
      throw new Error('Cannot check default-ness, either "value" or "defaultValue" is undefined.');

    return this.value.equals(this.defaultValue);
  }
}

export { Field };
