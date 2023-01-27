'use strict';

import {stringifyType} from './Vrml.js';

export default class Field {
  #node;
  #type;
  #name;
  #value;
  #defaultValue;
  #restrictions;
  constructor(node, name, type, defaultValue, value) {
    this.#node = node;
    this.#name = name;
    this.#type = type;
    this.#value = value;
    this.#defaultValue = defaultValue;
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

  set defaultValue(newValue) {
    if (newValue.type() !== this.type)
      throw new Error(`Type mismatch, setting ${stringifyType(newValue.type())} to ${stringifyType(this.type)} parameter.`);

    this.#defaultValue = newValue;
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
