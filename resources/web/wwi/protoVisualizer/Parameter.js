'use strict';

import {stringifyType} from './Vrml.js';

export default class Parameter {
  #type;
  #name;
  #value;
  #defaultValue;
  #isTemplateRegenerator;
  constructor(node, name, type, defaultValue, value, isTemplateRegenerator) {
    this.node = node; // node this parameter belongs to
    this.type = type;
    this.name = name;
    this.defaultValue = defaultValue;
    this.value = value;
    this.isTemplateRegenerator = isTemplateRegenerator;
    this.parentNode = undefined;
  }

  get value() {
    return this.#value;
  }

  set value(v) {
    if (v.type() !== this.type)
      throw new Error('Type mismatch, setting ' + stringifyType(v.type()) + ' to ' + stringifyType(this.type) + ' parameter.');

    this.#value = v;
  }

  get defaultValue() {
    return this.#defaultValue;
  }

  set defaultValue(v) {
    if (v.type() !== this.type)
      throw new Error('Type mismatch, setting ' + stringifyType(v.type()) + ' to ' + stringifyType(this.type) + ' parameter.');

    this.#defaultValue = v;
  }

  get type() {
    return this.#type;
  }

  set type(type) {
    this.#type = type;
  }

  get name() {
    return this.#name;
  }

  set name(value) {
    this.#name = value;
  }

  get isTemplateRegenerator() {
    return this.#isTemplateRegenerator;
  }

  set isTemplateRegenerator(value) {
    this.#isTemplateRegenerator = value;
  }

  setValueFromJavaScript(v) {
    this.#value.setValueFromJavaScript(v);

    if (this.isTemplateRegenerator)
      this.node.parseBody(true);
  }

  isDefault() {
    if (typeof this.defaultValue === 'undefined' || typeof this.value === 'undefined')
      throw new Error('Cannot check default-ness, either "value" or "defaultValue" is undefined.');

    return this.value.equals(this.defaultValue);
  }

  clone() {
    const copy = new Parameter(this.node, this.name, this.type, this.defaultValue.clone(), this.value.clone(),
      this.isTemplateRegenerator);
    copy.parentNode = this.parentNode;
    return copy;
  }
}

export {Parameter};
