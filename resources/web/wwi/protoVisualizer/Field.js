'use strict';

export default class Field {
  #type;
  #name;
  #value;
  #defaultValue;
  constructor(name, type, defaultValue, value) {
    this.#name = name;
    this.#type = type;
    this.#defaultValue = defaultValue;
    this.#value = value;
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
}

export { Field };
