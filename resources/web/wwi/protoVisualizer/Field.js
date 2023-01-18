'use strict';

export default class Field {
  #node;
  #type;
  #name;
  #value;
  #defaultValue;
  #restrictions;
  constructor(node, name, type, value, defaultValue) {
    this.#node = node;
    this.#name = name;
    this.#type = type;
    this.#value = value;
    this.#defaultValue = defaultValue;
    this.#restrictions = [];
  }

  get node() {
    return this.#node;
  }

  get value() {
    return this.#value;
  }

  set value(newValue) {
    this.#value = newValue;
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
