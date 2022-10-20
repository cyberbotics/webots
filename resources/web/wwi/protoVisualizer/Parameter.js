'use strict';

import {getAParameterId} from '../nodes/utils/id_provider.js';

export default class Parameter {
  #value;
  #defaultValue;
  #id = getAParameterId();
  constructor(node, name, defaultValue, value, isTemplateRegenerator) {
    this.node = node; // node this parameter belongs to
    this.name = name;
    this.defaultValue = defaultValue;
    this.value = value;
    this.isTemplateRegenerator = isTemplateRegenerator;
  }

  get value() {
    return this.#value;
  }

  set value(v) {
    this.#value = v;
  }

  get defaultValue() {
    return this.#defaultValue;
  }

  set defaultValue(v) {
    this.#defaultValue = v;
  }

  get id() {
    return this.#id;
  }

  set id(value) {
    this.#id = value;
  }

  type() {
    if (typeof this.defaultValue === 'undefined')
      throw new Error('Requesting type of an undefined parameter');

    return this.defaultValue.type();
  }

  isDefault() {
    return this.value.equals(this.defaultValue);
  }

  clone() {
    const copy = new Parameter(this.node, this.name, this.defaultValue.clone(), this.defaultValue.clone(),
      this.isTemplateRegenerator);
    return copy;
  }
}

export { Parameter };
