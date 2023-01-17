'use strict';

export default class Field {
  #type;
  #name;
  #value;
  constructor(name, type, value) {
    this.#name = name;
    this.#type = type;
    this.#value = value;
  }

  get value() {
    return this.#value;
  }

  get type() {
    return this.#type;
  }

  get name() {
    return this.#name;
  }
}

export { Field };
